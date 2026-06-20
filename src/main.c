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
 * [1] 하드웨어 배선 할당 (확정 핀맵 고정)
 * ========================================================================= */
#define PIN_MOTOR_PUL 4        // [출력] TB6600 스텝 모터 펄스 신호선
#define PIN_MOTOR_DIR 5        // [출력] TB6600 스텝 모터 회전 방향선
#define PIN_EMERGENCY_SWITCH 6 // [입력] 수동 비상 복귀 및 하강 제어 스위치
#define PIN_COOLING_PUMP 8     // [출력] 소화수 펌프/릴레이 제어 핀

#define UART_PORT_NUM UART_NUM_2
#define PIN_UART_RX 17 // GY-MCU90640 모듈 TX (ESP32-S3 RX2)
#define PIN_UART_TX 18 // GY-MCU90640 모듈 RX (ESP32-S3 TX2)
#define UART_BUF_SIZE 4096   // 1544×2=3088 바이트 이상 확보 (4Hz 버퍼 여유)

#define ADC_MQ_CHANNEL ADC_CHANNEL_6 // GPIO 7번 (ADC1 단독 사용)

/* =========================================================================
 * [2] 알고리즘 제어 매개변수 및 임계값 설정
 * ========================================================================= */
#define MOTOR_PULSES_PER_REV 800 // TB6600 하드웨어 세팅 값
#define MOTOR_TARGET_REV 20.0    // 방수포 기구부 유효 이동 거리 (20바퀴)
#define MOTOR_PULSE_DELAY_US 400 // 스텝 모터 기동 주파수 (고속 복귀 세팅)

#define BUTTON_RESET_HOLD_MS 3000 // 상단 원점 강제 복귀를 위한 롱프레스 판정 시간

#define MQ_FIRE_THRESHOLD 1200
#define TEMP_FIRE_PIXEL 75.0
#define FIRE_MIN_AREA_PIXELS 12

static const char *TAG = "EV_SAFETY_CORE";

typedef enum {
    STATE_READY_REVERSE, // 상단 원점 대기 상태
    STATE_READY_FORWARD  // 하단 일반 감시 대기 상태
} SystemState;

/* =========================================================================
 * [3] 글로벌 커널 공유 자원 (volatile 필수 마킹)
 * ========================================================================= */
volatile SystemState current_state = STATE_READY_REVERSE;
volatile int mq_raw_value = 0;
volatile int high_temp_pixel_count = 0;
volatile float max_temp = 0.0f;
volatile bool hybrid_fire_triggered = false;

adc_oneshot_unit_handle_t adc1_handle;
int32_t max_target_steps = 0;
volatile int32_t actual_moved_steps = 0;
bool stop_requested = false;
bool needs_position_reset = false; // 화재 비상 복귀(move_emergency_return) 이후 true로 설정 -> 다음 버튼 입력 시 초기 위치(상단 원점)로 강제 복귀

float mlx90640_frame[32 * 24];

/* =========================================================================
 * [4] 하드웨어 및 통신 주변장치 드라이버 초기화
 * ========================================================================= */
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

    // GY-MCU90640 정확한 초기화 명령 (체크섬 = 하위바이트 합계)
    // 이전 코드 {0xA5,0x15,0xBA}는 존재하지 않는 명령 → 센서 미응답(모든 픽셀 fallback 24°C)
    uint8_t cmd_set_4hz[]  = {0xA5, 0x25, 0x01, 0xCB};  // 프레임 주기 4Hz 설정
    uint8_t cmd_start_auto[] = {0xA5, 0x35, 0x02, 0xDC}; // 자동 연속 스트리밍 시작
    uart_write_bytes(UART_PORT_NUM, (const char *)cmd_set_4hz,   sizeof(cmd_set_4hz));
    vTaskDelay(pdMS_TO_TICKS(120));
    uart_write_bytes(UART_PORT_NUM, (const char *)cmd_start_auto, sizeof(cmd_start_auto));
    ESP_LOGI(TAG, "GY-MCU90640 4Hz 자동 스트리밍 시작 (0xA5 0x25 0x01 / 0xA5 0x35 0x02).");
}

void init_hardware(void) {
    gpio_reset_pin(PIN_MOTOR_PUL);
    gpio_reset_pin(PIN_MOTOR_DIR);
    gpio_set_direction(PIN_MOTOR_PUL, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_MOTOR_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_MOTOR_PUL, 0);
    gpio_set_level(PIN_MOTOR_DIR, 1);

    gpio_reset_pin(PIN_EMERGENCY_SWITCH);
    gpio_set_direction(PIN_EMERGENCY_SWITCH, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_EMERGENCY_SWITCH, GPIO_PULLUP_ONLY);

    gpio_reset_pin(PIN_COOLING_PUMP);
    gpio_set_direction(PIN_COOLING_PUMP, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_COOLING_PUMP, 0);

    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1, .clk_src = 0};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t mq_chan_config = {.bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_MQ_CHANNEL, &mq_chan_config));

    init_uart_module();
    max_target_steps = (int32_t)(MOTOR_PULSES_PER_REV * MOTOR_TARGET_REV);
}

bool is_button_pressed_debounced(void) {
    if (gpio_get_level(PIN_EMERGENCY_SWITCH) != 0) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return gpio_get_level(PIN_EMERGENCY_SWITCH) == 0;
}

void wait_for_button_release_debounced(void) {
    vTaskDelay(pdMS_TO_TICKS(10));
    while (gpio_get_level(PIN_EMERGENCY_SWITCH) == 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(20));
}

// 스위치가 hold_ms 이상 연속으로 눌려있으면 true, 그 전에 떼면 false
bool is_button_held_long(uint32_t hold_ms) {
    TickType_t start_tick = xTaskGetTickCount();
    while (gpio_get_level(PIN_EMERGENCY_SWITCH) == 0) {
        if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(hold_ms)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

/* =========================================================================
 * [5] 기구부 및 밸브 하위 구동 제어부
 * ========================================================================= */
bool move_reverse_with_mon(int32_t max_steps) {
    gpio_set_level(PIN_MOTOR_DIR, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    actual_moved_steps = 0;
    stop_requested = false;

    for (int32_t i = 0; i < max_steps; i++) {
        asm volatile("memw");
        if (hybrid_fire_triggered) {
            stop_requested = true;
            break;
        }

        gpio_set_level(PIN_MOTOR_PUL, 1);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        gpio_set_level(PIN_MOTOR_PUL, 0);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        actual_moved_steps++;

        // [안전장치] 약 80ms마다 한 번씩 CPU를 양보하여 Watchdog 패닉을 방지합니다.
        if (i % 100 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (gpio_get_level(PIN_EMERGENCY_SWITCH) == 0) {
            stop_requested = true;
            break;
        }
    }
    return stop_requested;
}

// 비상 전개(DIR=0): 리프트 상승 → 수조가 차량을 덮음
// 전체 이동 거리의 1/3 지점에서 펌프 릴레이 기동 (PUL 루프 정지 후 전환 → EMI 간섭 방지)
void move_emergency_return(int32_t steps_to_return) {
    if (steps_to_return <= 0)
        steps_to_return = max_target_steps;
    ESP_LOGE(TAG, "[!] 비상 전개 개시 ➡️ 목표 스텝: %ld", steps_to_return);

    gpio_set_level(PIN_MOTOR_DIR, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    int32_t trigger_point = steps_to_return / 3;
    if (trigger_point < 50)
        trigger_point = 50;
    if (trigger_point > steps_to_return)
        trigger_point = steps_to_return;

    // [1단계] 0 → 1/3 구간 (펌프 OFF)
    for (int32_t i = 0; i < trigger_point; i++) {
        gpio_set_level(PIN_MOTOR_PUL, 1);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        gpio_set_level(PIN_MOTOR_PUL, 0);
        ets_delay_us(MOTOR_PULSE_DELAY_US);

        // [안전장치] Watchdog 패닉 방지
        if (i % 100 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    // 1/3 지점 도달: 루프 정지 상태에서 릴레이 기동
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGE(TAG, "[🌊 PUMP ON] 전개 1/3 통과 -> 소화수 펌프 릴레이 기동!!");
    gpio_set_level(PIN_COOLING_PUMP, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // [2단계] 1/3 → 끝 구간 (펌프 ON 유지)
    for (int32_t i = trigger_point; i < steps_to_return; i++) {
        gpio_set_level(PIN_MOTOR_PUL, 1);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        gpio_set_level(PIN_MOTOR_PUL, 0);
        ets_delay_us(MOTOR_PULSE_DELAY_US);

        // [안전장치] Watchdog 패닉 방지
        if (i % 100 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// 수조 원위치 복귀(DIR=1): 리프트 하강 → 내려가는 순간 즉시 펌프 차단
void move_forward_return(int32_t steps_to_return) {
    if (steps_to_return <= 0)
        steps_to_return = max_target_steps;
    gpio_set_level(PIN_COOLING_PUMP, 0); // 하강 시작 즉시 펌프 차단
    ESP_LOGE(TAG, "[!] 원위치 복귀 개시 ➡️ 목표 스텝: %ld", steps_to_return);
    gpio_set_level(PIN_MOTOR_DIR, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    for (int32_t i = 0; i < steps_to_return; i++) {
        gpio_set_level(PIN_MOTOR_PUL, 1);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        gpio_set_level(PIN_MOTOR_PUL, 0);
        ets_delay_us(MOTOR_PULSE_DELAY_US);

        // [안전장치] Watchdog 패닉 방지
        if (i % 100 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

/* =========================================================================
 * TASK 1: [최우선순위] 이동식 침수 수조 형성 및 무제한 소화수 순환 (Core 1)
 * ========================================================================= */
void vTaskEmergencyMonitor(void *pvParameters) {
    while (1) {
        asm volatile("memw");

        bool is_fire_detected = hybrid_fire_triggered;

        // -----------------------------------------------------------------
        // [A] 고전압 배터리 화재 트리거 시 무제한 순환 침수 소화 분기
        // -----------------------------------------------------------------
        if (is_fire_detected) {
            ESP_LOGE(TAG, "[🔥 EMERGENCY] 배터리 팩 열폭주 감지! 즉시 비상 복귀 동작 개시!!");

            // 하강 완료 상태이거나 중간에 걸친 발자취 스텝이 있다면 그만큼 복귀 연산
            int32_t run_steps = (actual_moved_steps > 0) ? actual_moved_steps : max_target_steps;

            // move_emergency_return 내부에서 1/3 지점 도달 시 펌프 자동 기동
            move_emergency_return(run_steps);

            needs_position_reset = true;
            ESP_LOGW(TAG, "[!] 화재 전개(%ld 스텝) 완료 -> 펌프 순환 유지, 버튼 입력 시 원위치 복귀", run_steps);

            actual_moved_steps = 0;
            current_state = STATE_READY_REVERSE;
            vTaskDelay(pdMS_TO_TICKS(100));

            // 화재 해제까지 펌프 ON 유지 (리프트 내려가기 전까지 계속 가동)
            while (hybrid_fire_triggered) {
                asm volatile("memw");
                ESP_LOGW(TAG, "[🔄 RECIRCULATING] 방수포 수조 내 순환 냉각 구동 중... (배터리 안정화 대기)");
                vTaskDelay(pdMS_TO_TICKS(2000));
            }

            // 화재 해제 후에도 펌프 ON 유지 → 버튼으로 리프트 하강 시작 즉시 move_forward_return 내부에서 차단
            ESP_LOGI(TAG, "[✔ SAFE] 화재 시그널 해제 -> 펌프 유지, 버튼으로 원위치 복귀 대기");
        }

        // -----------------------------------------------------------------
        // [B] 평상시 수동 제어 상태 머신 (화재 미감지 시에만 진입)
        // -----------------------------------------------------------------
        if (!is_fire_detected) {
            if (current_state == STATE_READY_REVERSE) {
                if (is_button_pressed_debounced()) {
                    if (needs_position_reset || is_button_held_long(BUTTON_RESET_HOLD_MS)) {
                        wait_for_button_release_debounced();
                        ESP_LOGW(TAG, "[⬆ RESET] %s -> 초기 위치(상단 원점)로 복귀 (리프트 하강 즉시 펌프 차단)",
                                 needs_position_reset ? "화재 복귀 후 초기화" : "롱프레스 감지");

                        // move_forward_return 내부에서 펌프 즉시 차단 후 복귀
                        move_forward_return(max_target_steps);

                        actual_moved_steps = 0;
                        current_state = STATE_READY_REVERSE;
                        needs_position_reset = false;
                        continue;
                    }

                    wait_for_button_release_debounced(); // 손을 완전히 뗄 때까지 대기

                    ESP_LOGW(TAG, "[↑ UP] 리프트 상승 지시 -> 수조 전개 기동");

                    move_reverse_with_mon(max_target_steps);

                    asm volatile("memw");
                    if (hybrid_fire_triggered)
                        continue; // 화재 발생 시 강제 분기

                    if (actual_moved_steps < max_target_steps) {
                        ESP_LOGI(TAG, "[!] 하강 중지됨 -> 상승 복귀");
                        wait_for_button_release_debounced(); // 재입력 후 손 뗄 때까지 대기
                        move_forward_return(actual_moved_steps);
                        actual_moved_steps = 0;
                        current_state = STATE_READY_REVERSE;
                    } else {
                        ESP_LOGI(TAG, "[✔ DOWN COMPLETE] 하강 완료 -> 하단 일반 감시 대기 모드 진입");
                        current_state = STATE_READY_FORWARD;
                    }
                }
            } else if (current_state == STATE_READY_FORWARD) {
                if (is_button_pressed_debounced()) {
                    ESP_LOGW(TAG, "[!] 작업자 수동 스위치 개입 -> 일반 리프트업 실행");
                    wait_for_button_release_debounced(); // 손을 완전히 뗄 때까지 대기

                    move_forward_return(actual_moved_steps);
                    actual_moved_steps = 0;
                    current_state = STATE_READY_REVERSE;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* =========================================================================
 * TASK 2: [하위우선순위] 리틀 엔디안 동기화 파싱 및 하이브리드 공간 화재 판정 (Core 0)
 * ========================================================================= */
void vTaskSensorAndImageProcessing(void *pvParameters) {
    static uint8_t rx_data[1544];

    while (1) {
        int temp_mq = 0;
        adc_oneshot_read(adc1_handle, ADC_MQ_CHANNEL, &temp_mq);
        mq_raw_value = temp_mq;

        // 가열 테스트 사양 (40도 / MQ 300 이상) 완전 하향 세팅 고정
        float current_temp_threshold = TEMP_FIRE_PIXEL;
        int current_area_threshold = FIRE_MIN_AREA_PIXELS;

        if (mq_raw_value >= 1200 || (mq_raw_value >= 300 && mq_raw_value < 600)) {
            current_temp_threshold = 40.0f;
            current_area_threshold = 4;
        } else if (mq_raw_value >= 600) {
            current_temp_threshold = 65.0f;
            current_area_threshold = 8;
        }

        size_t available_length = 0;
        uart_get_buffered_data_len(UART_PORT_NUM, &available_length);

        if (available_length > 4000) {
            uart_flush_input(UART_PORT_NUM);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (available_length >= 1544) {
            uint8_t head_byte = 0;
            uart_read_bytes(UART_PORT_NUM, &head_byte, 1, pdMS_TO_TICKS(1));

            if (head_byte == 0x5A) {
                uint8_t second_byte = 0;
                uart_read_bytes(UART_PORT_NUM, &second_byte, 1, pdMS_TO_TICKS(1));

                if (second_byte == 0x5A) {
                    rx_data[0] = 0x5A;
                    rx_data[1] = 0x5A;

                    int read_len = uart_read_bytes(UART_PORT_NUM, &rx_data[2], 1542, pdMS_TO_TICKS(50));

                    if (read_len == 1542) {
                        int pixel_idx = 0;
                        for (int i = 4; i < 1540; i += 2) {
                            if (pixel_idx < 768) {
                                int16_t raw_temp = (int16_t)((rx_data[i + 1] << 8) | rx_data[i]);
                                float real_calculated_temp = (float)raw_temp / 100.0f;

                                if (real_calculated_temp >= 5.0f && real_calculated_temp <= 120.0f) {
                                    mlx90640_frame[pixel_idx] = real_calculated_temp;
                                }
                                pixel_idx++;
                            }
                        }
                    }
                }
            }
        }

        int hot_pixels = 0;
        float current_max = -40.0f;

        for (int i = 0; i < 32 * 24; i++) {
            float p = mlx90640_frame[i];
            if (p == 0.0f) {
                mlx90640_frame[i] = 24.0f;
                p = 24.0f;
            }
            if (p > current_max)
                current_max = p;

            if (p >= current_temp_threshold) {
                hot_pixels++;
            }
        }
        max_temp = current_max;
        high_temp_pixel_count = hot_pixels;

        if (high_temp_pixel_count >= current_area_threshold) {
            hybrid_fire_triggered = true;
        } else {
            hybrid_fire_triggered = false;
        }

        printf("[FUSION] Thresh=%.1f°C Area=%dpx | Gas=%d Peak=%.1f°C Hot=%dpx | %s\n",
               current_temp_threshold, current_area_threshold,
               mq_raw_value, max_temp, high_temp_pixel_count,
               hybrid_fire_triggered ? "!! FIRE !!" : "SAFE");

        // PC 열화상 모니터 전송 — 컴팩트 hex 포맷 (참조: GY-MCU90640 프로토콜)
        // $H,<tick_ms>,<mq>,<maxtemp*100>,<hot_pixels>,<fire>,<3072 hex chars (768 x int16 x 100)>
        // %04X = big-endian hex, PC에서 dtype='>i2' 로 파싱, 총 ~3130자 @ 115200bps = 0.27초
        printf("$H,%lu,%d,%d,%d,%d,",
               (unsigned long)xTaskGetTickCount(),
               mq_raw_value, (int)(max_temp * 100.0f),
               high_temp_pixel_count, (int)hybrid_fire_triggered);
        for (int i = 0; i < 768; i++) {
            int16_t v = (int16_t)(mlx90640_frame[i] * 100.0f);
            printf("%04X", (uint16_t)v);
        }
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(250));  // 4Hz 센서 주기(250ms) 에 맞춘 폴링
    }
}

void app_main(void) {
    init_hardware();
    xTaskCreatePinnedToCore(vTaskEmergencyMonitor, "EmergencyMonitor", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vTaskSensorAndImageProcessing, "SensorProcessor", 8192, NULL, 1, NULL, 0);
}