/**
 * @file main.c
 * @brief 차량 고전압 배터리 화재 방재용 [UART 열화상 섭씨 정밀 매핑 X 가스] 하이브리드 제어 시스템
 */

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * [1] 하드웨어 배선 할당 (확정 핀맵)
 * ========================================================================= */
#define PIN_MOTOR_PUL 4
#define PIN_MOTOR_DIR 5
#define PIN_EMERGENCY_SWITCH 6

#define UART_PORT_NUM UART_NUM_2
#define PIN_UART_RX 17
#define PIN_UART_TX 18
#define UART_BUF_SIZE 2048

#define ADC_MQ_CHANNEL ADC_CHANNEL_6 // GPIO 7번 (ADC1)

/* =========================================================================
 * [2] 알고리즘 제어 매개변수 설정
 * ========================================================================= */
#define MOTOR_PULSES_PER_REV 800
#define MOTOR_TARGET_REV 20.0
#define MOTOR_PULSE_DELAY_US 400

// 고전압 배터리 방재 기준 임계값
#define MQ_FIRE_THRESHOLD 1200
#define MQ_DELTA_THRESHOLD 300
#define TEMP_FIRE_PIXEL 75.0    // 배터리 열폭주 판정 기준 섭씨온도 (75°C)
#define FIRE_MIN_AREA_PIXELS 12 // 화재 확정 최소 고온 픽셀 면적선

static const char *TAG = "EV_SAFETY_CORE";

typedef enum {
    STATE_READY_REVERSE,
    STATE_READY_FORWARD
} SystemState;

/* =========================================================================
 * [3] 글로벌 커널 공유 자원
 * ========================================================================= */
volatile SystemState current_state = STATE_READY_REVERSE;
volatile int mq_raw_value = 0;
volatile int mq_previous_value = 0;
volatile int high_temp_pixel_count = 0;
volatile float max_temp = 0.0f;
volatile bool hybrid_fire_triggered = false;

adc_oneshot_unit_handle_t adc1_handle;
int32_t max_target_steps = 0;
int32_t actual_moved_steps = 0;
bool stop_requested = false;

float mlx90640_frame[32 * 24];

void init_uart_module(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, PIN_UART_TX, PIN_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uint8_t cmd_auto_output[] = {0xA5, 0x15, 0xBA};
    uart_write_bytes(UART_PORT_NUM, (const char *)cmd_auto_output, sizeof(cmd_auto_output));
}

void init_hardware(void) {
    gpio_reset_pin(PIN_MOTOR_PUL);
    gpio_reset_pin(PIN_MOTOR_DIR);
    gpio_set_direction(PIN_MOTOR_PUL, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_MOTOR_DIR, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_EMERGENCY_SWITCH);
    gpio_set_direction(PIN_EMERGENCY_SWITCH, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_EMERGENCY_SWITCH, GPIO_PULLUP_ONLY);

    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1, .clk_src = 0};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t mq_chan_config = {.bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_MQ_CHANNEL, &mq_chan_config));

    init_uart_module();
    max_target_steps = (int32_t)(MOTOR_PULSES_PER_REV * MOTOR_TARGET_REV);
}

void wait_for_button_release(void) {
    vTaskDelay(pdMS_TO_TICKS(10));
    while (gpio_get_level(PIN_EMERGENCY_SWITCH) == 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(20));
}

void move_reverse_with_mon(int32_t max_steps) {
    gpio_set_level(PIN_MOTOR_DIR, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    actual_moved_steps = 0;
    stop_requested = false;

    for (int32_t i = 0; i < max_steps; i++) {
        gpio_set_level(PIN_MOTOR_PUL, 1);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        gpio_set_level(PIN_MOTOR_PUL, 0);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        actual_moved_steps++;

        if (gpio_get_level(PIN_EMERGENCY_SWITCH) == 0) {
            stop_requested = true;
            break;
        }
    }
}

void move_forward_return(int32_t steps_to_return) {
    gpio_set_level(PIN_MOTOR_DIR, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGE(TAG, "[!] 방재 리프트 비상 회피 가동 -> 고속 상승 스텝 수: %ld", steps_to_return);

    for (int32_t i = 0; i < steps_to_return; i++) {
        gpio_set_level(PIN_MOTOR_PUL, 1);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        gpio_set_level(PIN_MOTOR_PUL, 0);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
    }
}

/* =========================================================================
 * TASK 1: [최우선순위] 비상 수동 스위치 및 비상 대피 모터 기동 (Core 1)
 * ========================================================================= */
void vTaskEmergencyMonitor(void *pvParameters) {
    while (1) {
        bool current_switch_level = gpio_get_level(PIN_EMERGENCY_SWITCH);
        bool is_fire_detected = hybrid_fire_triggered;

        if (current_state == STATE_READY_REVERSE) {
            if (current_switch_level == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (gpio_get_level(PIN_EMERGENCY_SWITCH) == 0) {
                    wait_for_button_release();
                    move_reverse_with_mon(max_target_steps);
                    current_state = STATE_READY_FORWARD;
                }
            }
        } else if (current_state == STATE_READY_FORWARD) {
            if ((current_switch_level == 0) || is_fire_detected) {
                if (current_switch_level == 0) {
                    ESP_LOGW(TAG, "[!] 수동 스위치 강제 차단 개입");
                    wait_for_button_release();
                } else {
                    ESP_LOGE(TAG, "[!] 고전압 배터리 하이브리드 화재 판정 자동 시스템 복귀!");
                }
                move_forward_return(actual_moved_steps);
                actual_moved_steps = 0;
                current_state = STATE_READY_REVERSE;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* =========================================================================
 * TASK 2: [하위우선순위] 섭씨온도 정밀 보정 파싱 및 화재 격자 연산 (Core 0)
 * ========================================================================= */

void vTaskSensorAndImageProcessing(void *pvParameters) {
    // 1544바이트 패킷을 정량으로 담을 버퍼 선언
    static uint8_t rx_data[1544];

    while (1) {
        // [1] 가스 측정
        int temp_mq = 0;
        adc_oneshot_read(adc1_handle, ADC_MQ_CHANNEL, &temp_mq);
        mq_raw_value = temp_mq;

        // [2] 하드웨어 UART2 버퍼에 쌓인 데이터 크기 확인
        size_t available_length = 0;
        uart_get_buffered_data_len(UART_PORT_NUM, &available_length);

        if (available_length > 4000) {
            uart_flush_input(UART_PORT_NUM);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // 최소 1 프레임(1544바이트) 이상 쌓였을 때 판독 개시
        if (available_length >= 1544) {
            uint8_t head_byte = 0;

            // 0x5A 헤더가 버퍼 맨 앞으로 오도록 포인터 정렬
            uart_read_bytes(UART_PORT_NUM, &head_byte, 1, pdMS_TO_TICKS(1));

            if (head_byte == 0x5A) {
                uint8_t second_byte = 0;
                uart_read_bytes(UART_PORT_NUM, &second_byte, 1, pdMS_TO_TICKS(1));

                if (second_byte == 0x5A) {
                    rx_data[0] = 0x5A;
                    rx_data[1] = 0x5A;

                    // 나머지 1542바이트 정량 수신
                    int read_len = uart_read_bytes(UART_PORT_NUM, &rx_data[2], 1542, pdMS_TO_TICKS(50));

                    if (read_len == 1542) {
                        int pixel_idx = 0;

                        // ★ [파이썬 코드 기반 정밀 이식]
                        // 파이썬의 np.frombuffer(dtype=np.int16)와 완벽히 일치하도록
                        // 리틀 엔디안 순서(뒤 바이트가 상위 바이트)로 비트 결합을 수행합니다.
                        for (int i = 4; i < 1540; i += 2) {
                            if (pixel_idx < 768) {
                                // rx_data[i+1]을 상위로, rx_data[i]를 하위로 결합!
                                int16_t raw_temp = (int16_t)((rx_data[i + 1] << 8) | rx_data[i]);

                                // 공식 스케일러 적용하여 섭씨온도 배열 적재
                                mlx90640_frame[pixel_idx] = (float)raw_temp / 100.0f;
                                pixel_idx++;
                            }
                        }
                    }
                }
            }
        }

        // [3] 경량 공간 영상처리 연산 (고온 면적 카운팅)
        int hot_pixels = 0;
        float current_max = -40.0f;

        for (int i = 0; i < 32 * 24; i++) {
            float p = mlx90640_frame[i];

            // 데이터가 비어 있는 초기 상태(0.0)만 상온으로 일시 보정
            if (p == 0.0f) {
                mlx90640_frame[i] = 24.0f;
                p = 24.0f;
            }

            if (p > current_max)
                current_max = p;
            if (p >= TEMP_FIRE_PIXEL) {
                hot_pixels++;
            }
        }
        max_temp = current_max;
        high_temp_pixel_count = hot_pixels;

        // [4] 하이브리드 고전압 방재 판정식
        bool is_gas_emergency = (mq_raw_value >= MQ_FIRE_THRESHOLD);
        bool is_area_emergency = (high_temp_pixel_count >= FIRE_MIN_AREA_PIXELS);
    
        if (is_gas_emergency && is_area_emergency) {
            hybrid_fire_triggered = true;
        } else {
            hybrid_fire_triggered = false;
        }

        // [5] 섭씨 단위 클린 매트릭스 화면 출력
        printf("\n=== [MLX90640 Fixed Aligned Monitor (°C)] ===\n");
        for (int y = 0; y < 24; y += 3) {
            for (int x = 0; x < 32; x += 4) {
                float val = mlx90640_frame[y * 32 + x];
                if (val >= TEMP_FIRE_PIXEL) {
                    printf("!%3.0f! ", val);
                } else {
                    printf(" %3.0f  ", val);
                }
            }
            printf("\n");
        }
        printf("-----------------------------------------------------\n");
        printf("[DATA] Gas: %d | Peak: %.1f °C | Area: %d px\n",
               mq_raw_value, max_temp, high_temp_pixel_count);
        printf("[DIAG] Safety Status: %s\n", hybrid_fire_triggered ? "!! DANGER (TRIGGERED) !!" : "SAFE (MONITORING)");
        printf("-----------------------------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    init_hardware();
    xTaskCreatePinnedToCore(vTaskEmergencyMonitor, "EmergencyMonitor", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vTaskSensorAndImageProcessing, "SensorProcessor", 8192, NULL, 1, NULL, 0);
}