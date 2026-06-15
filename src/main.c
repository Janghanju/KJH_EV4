/**
 * @file main.c
 * @brief 차량 고전압 배터리 화재 방재용 [이동식 방수포 수조 제어 X 무제한 순환 냉각 펌프] 통합 시스템
 * @note Core 1: 레이턴시 제로 수준의 리프트 및 펌프 하드웨어 안전 제어
 * Core 0: UART2 포트 기반 GY-MCU90640 리틀 엔디안 패킷 얼라인먼트 및 가스 ADC 분석 연산
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
 * [1] 하드웨어 배선 할당 (확정 핀맵 고정)
 * ========================================================================= */
#define PIN_MOTOR_PUL 4        // [출력] TB6600 스텝 모터 펄스 신호선
#define PIN_MOTOR_DIR 5        // [출력] TB6600 스텝 모터 회전 방향선
#define PIN_EMERGENCY_SWITCH 6 // [입력] 수동 비상 복귀 및 하강 제어 스위치
#define PIN_COOLING_PUMP 8     // [출력] 소화수 펌프/릴레이 제어 핀

#define UART_PORT_NUM UART_NUM_2
#define PIN_UART_RX 17 // GY-MCU90640 모듈 TX 직결선 (ESP32-S3 RX2)
#define PIN_UART_TX 18 // GY-MCU90640 모듈 RX 직결선 (ESP32-S3 TX2)
#define UART_BUF_SIZE 2048

#define ADC_MQ_CHANNEL ADC_CHANNEL_6 // GPIO 7번 (ADC1 단독 사용)

/* =========================================================================
 * [2] 알고리즘 제어 매개변수 및 임계값 설정
 * ========================================================================= */
#define MOTOR_PULSES_PER_REV 800 // TB6600 하드웨어 세팅 값
#define MOTOR_TARGET_REV 20.0    // 방수포 기구부 유효 이동 거리 (20바퀴)
#define MOTOR_PULSE_DELAY_US 400 // 스텝 모터 기동 주파수 (고속 복귀 세팅)

// 고전압 배터리 열폭주 전조 단계 특화 하이브리드 판정 임계치
#define MQ_FIRE_THRESHOLD 1200  // 배터리 셀 오프가스 감지 절대값
#define TEMP_FIRE_PIXEL 75.0    // 열폭주 의심 단일 격자 온도 기준점 (°C)
#define FIRE_MIN_AREA_PIXELS 12 // 화재 확정 및 노이즈 차단용 최소 픽셀 면적

static const char *TAG = "EV_SAFETY_CORE";

typedef enum {
    STATE_READY_REVERSE, // 상단 원점 대기 상태 (수조 형성 가능 상태)
    STATE_READY_FORWARD  // 하단 일반 감시 대기 상태
} SystemState;

/* =========================================================================
 * [3] 글로벌 커널 공유 자원 (volatile 필수 마킹)
 * ========================================================================= */
volatile SystemState current_state = STATE_READY_REVERSE;
volatile int mq_raw_value = 0;
volatile int high_temp_pixel_count = 0;
volatile float max_temp = 0.0f;
volatile bool hybrid_fire_triggered = false; // Core 0 -> Core 1 화재 공유 플래그

adc_oneshot_unit_handle_t adc1_handle;
int32_t max_target_steps = 0;   // 800 * 20 = 16000 스텝
int32_t actual_moved_steps = 0; // 하강 시 실제 이동한 거리를 저장 (가변 복귀용)
bool stop_requested = false;    // 하강 시 중도 멈춤 제어용 플래그

float mlx90640_frame[32 * 24]; // 768개 픽셀 섭씨 평탄화 온도 배열

/* =========================================================================
 * [4] 하드웨어 및 통신 주변장치 드라이버 초기화
 * ========================================================================= */
void init_uart_module(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200, // GY-MCU90640 모듈 고정 통신 속도
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, PIN_UART_TX, PIN_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 모듈 고유 명령어: 실시간 자동 프레임 전송 모드 가동 (3바이트 패킷)
    uint8_t cmd_auto_output[] = {0xA5, 0x15, 0xBA};
    uart_write_bytes(UART_PORT_NUM, (const char *)cmd_auto_output, sizeof(cmd_auto_output));
    ESP_LOGI(TAG, "GY-MCU90640 UART 통신 채널 스트리밍 개시.");
}

void init_hardware(void) {
    // 모터 구동 핀 설정
    gpio_reset_pin(PIN_MOTOR_PUL);
    gpio_reset_pin(PIN_MOTOR_DIR);
    gpio_set_direction(PIN_MOTOR_PUL, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_MOTOR_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_MOTOR_PUL, 0);
    gpio_set_level(PIN_MOTOR_DIR, 0);

    // 스위치 입력 핀 설정 (디바운싱 내부 풀업 활용)
    gpio_reset_pin(PIN_EMERGENCY_SWITCH);
    gpio_set_direction(PIN_EMERGENCY_SWITCH, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_EMERGENCY_SWITCH, GPIO_PULLUP_ONLY);

    // 펌프 릴레이 핀 설정
    gpio_reset_pin(PIN_COOLING_PUMP);
    gpio_set_direction(PIN_COOLING_PUMP, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_COOLING_PUMP, 0); // 초기에는 차단 상태 유지

    // ADC1 Oneshot 인스턴스 할당
    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1, .clk_src = 0};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // 가스 센서 감도 세팅 (12비트 스케일링, 감쇄량 최대 적용)
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

/* =========================================================================
 * [5] 기구부 및 밸브 하위 구동 제어부
 * ========================================================================= */
void move_reverse_with_mon(int32_t max_steps) {
    gpio_set_level(PIN_MOTOR_DIR, 0); // 방향: 하강 시퀀스
    vTaskDelay(pdMS_TO_TICKS(10));
    actual_moved_steps = 0;
    stop_requested = false;

    for (int32_t i = 0; i < max_steps; i++) {
        gpio_set_level(PIN_MOTOR_PUL, 1);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        gpio_set_level(PIN_MOTOR_PUL, 0);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        actual_moved_steps++;

        // 하강 구동 중 안전상의 이유로 스위치를 치면 즉각 그 자리에서 정지
        if (gpio_get_level(PIN_EMERGENCY_SWITCH) == 0) {
            stop_requested = true;
            break;
        }
    }
}

void move_forward_return(int32_t steps_to_return) {
    gpio_set_level(PIN_MOTOR_DIR, 1); // 방향: 상승 반전 복귀
    vTaskDelay(pdMS_TO_TICKS(10));

    // 현장 물리적 특성 반영: 최소한 수조 구조의 형태를 갖추기 시작하는 1/3 지점 트리거 연산
    int32_t trigger_point = steps_to_return / 3;
    ESP_LOGE(TAG, "[!] 수조 복귀 상승 작동 ➡️ 총 스텝: %ld | 1/3 주수 시동점: %ld", steps_to_return, trigger_point);

    for (int32_t i = 0; i < steps_to_return; i++) {
        gpio_set_level(PIN_MOTOR_PUL, 1);
        ets_delay_us(MOTOR_PULSE_DELAY_US);
        gpio_set_level(PIN_MOTOR_PUL, 0);
        ets_delay_us(MOTOR_PULSE_DELAY_US);

        // ★ [현장 맞춤 최적화 시퀀스]
        // 방수포가 1/3 가량 펼쳐지기 시작해 수조 외벽이 확보되는 즉시 소화 주수 기동
        if (i == trigger_point && hybrid_fire_triggered) {
            ESP_LOGE(TAG, "[🌊 PUMP ON] 리프트 1/3 통과! 방수포 수조 내 주수 및 배수 순환 개시!!");
            gpio_set_level(PIN_COOLING_PUMP, 1);
        }
    }
}

/* =========================================================================
 * TASK 1: [최우선순위] 이동식 침수 수조 형성 및 무제한 소화수 순환 (Core 1)
 * ========================================================================= */
void vTaskEmergencyMonitor(void *pvParameters) {
    while (1) {
        bool current_switch_level = gpio_get_level(PIN_EMERGENCY_SWITCH);
        bool is_fire_detected = hybrid_fire_triggered;

        // -----------------------------------------------------------------
        // [A] 고전압 배터리 화재 트리거 시 무제한 순환 침수 소화 분기
        // -----------------------------------------------------------------
        if (is_fire_detected) {
            ESP_LOGE(TAG, "[🔥 EMERGENCY] 배터리 팩 열폭주 확정 -> 방수포 이동식 수조 시스템 긴급 전개");

            // 1. 하강 상태였다면 리프트를 전개하여 침수용 외벽 수조 구축 개시 (상승하며 1/3에서 펌프 구동)
            if (current_state == STATE_READY_FORWARD) {
                move_forward_return(actual_moved_steps);
                actual_moved_steps = 0;
                current_state = STATE_READY_REVERSE;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            // 2. 이미 리프트가 상단에 대기 중이었다면 수조 형태가 완벽하므로 즉시 냉각수 투입
            else {
                if (gpio_get_level(PIN_COOLING_PUMP) == 0) {
                    ESP_LOGW(TAG, "[!] 수조 이미 구축 완료 상태 포착 -> 즉시 소화수 주수 개시");
                    gpio_set_level(PIN_COOLING_PUMP, 1);
                }
            }

            // 3. 멈추지 말고 부어야 한다: 온도가 떨어져 Task 2에서 화재 플래그를 내릴 때까지
            // 무제한으로 펌프 릴레이를 켜서 물을 순환 소화 시킵니다 (배수 및 냉각 지속).
            while (hybrid_fire_triggered) {
                ESP_LOGW(TAG, "[🔄 RECIRCULATING] 방수포 수조 내 순환 냉각 구동 중... (배터리 안정화 대기)");
                vTaskDelay(pdMS_TO_TICKS(2000));
            }

            // 4. 상황 종료 시 안전을 위해 즉시 주수 차단 후 원점 대기유지
            gpio_set_level(PIN_COOLING_PUMP, 0);
            ESP_LOGI(TAG, "[✔ SAFE] 화재 시그널 완전 해제 -> 펌프 차단 및 상단 안정화 대기");
        }

        // -----------------------------------------------------------------
        // [B] 평상시 안전 상태용 수동 제어 상태 머신 (화재 미감지 시)
        // -----------------------------------------------------------------
        if (!is_fire_detected) {
            // 원점 상단 대기 상태에서 하강 지시 스캔
            if (current_state == STATE_READY_REVERSE) {
                if (current_switch_level == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if (gpio_get_level(PIN_EMERGENCY_SWITCH) == 0) {
                        wait_for_button_release();

                        // ★ [주차장 넘침 방지] 리프트가 내려가는 연산 즉시 물부터 끄고 움직입니다.
                        gpio_set_level(PIN_COOLING_PUMP, 0);
                        ESP_LOGW(TAG, "[↓ DOWN] 리프트 하강 지시 -> 펌프 오프 및 하강 기동");

                        move_reverse_with_mon(max_target_steps);
                        current_state = STATE_READY_FORWARD;
                    }
                }
            }
            // 하단 감시 구역 대기 중 수동 강제 복귀 지시 스캔
            else if (current_state == STATE_READY_FORWARD) {
                if (current_switch_level == 0) {
                    ESP_LOGW(TAG, "[!] 작업자 수동 스위치 개입 -> 일반 리프트업 실행");
                    wait_for_button_release();

                    // 평상시 일반 복귀이므로 move_forward_return 함수 내부에서 펌프가 돌지 않습니다.
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
    static uint8_t rx_data[1544]; // 힙 오염 및 스택 오버플로우 완전 예방 정적 선언

    while (1) {
        // [1] MQ 가스 센서 아날로그 덤프
        int temp_mq = 0;
        adc_oneshot_read(adc1_handle, ADC_MQ_CHANNEL, &temp_mq);
        mq_raw_value = temp_mq;

        // [2] GY-MCU90640 UART 하드웨어 링 버퍼 크기 폴링
        size_t available_length = 0;
        uart_get_buffered_data_len(UART_PORT_NUM, &available_length);

        // 버퍼 가득 참 병목 및 과거 데이터 밀림 해결: 용량 포화 시 강제 플러시
        if (available_length > 4000) {
            uart_flush_input(UART_PORT_NUM);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // 정량 1프레임 바이트에 도달했을 때 정밀 동기화 판독 개시
        if (available_length >= 1544) {
            uint8_t head_byte = 0;

            // 버퍼 0번 인덱스 위치가 헤더의 시작점인 0x5A가 될 때까지 1바이트씩만 안전 차감 전진
            uart_read_bytes(UART_PORT_NUM, &head_byte, 1, pdMS_TO_TICKS(1));

            if (head_byte == 0x5A) {
                uint8_t second_byte = 0;
                uart_read_bytes(UART_PORT_NUM, &second_byte, 1, pdMS_TO_TICKS(1));

                if (second_byte == 0x5A) {
                    rx_data[0] = 0x5A;
                    rx_data[1] = 0x5A;

                    // 헤더 정렬 맞춤이 완료된 상태로 정확히 남은 1542바이트만 딱 도려내어 취득
                    int read_len = uart_read_bytes(UART_PORT_NUM, &rx_data[2], 1542, pdMS_TO_TICKS(50));

                    if (read_len == 1542) {
                        int pixel_idx = 0;

                        // ★ [리틀 엔디안 복구 엔진 이식 완료]
                        // 파이썬의 넘파이 frombuffer(int16) 구조식과 100% 매칭
                        // 뒤 바이트(i+1)를 쉬프트한 상위로, 앞 바이트(i)를 하위로 비트 마스킹 결합
                        for (int i = 4; i < 1540; i += 2) {
                            if (pixel_idx < 768) {
                                int16_t raw_temp = (int16_t)((rx_data[i + 1] << 8) | rx_data[i]);

                                // 분할 스케일러 적용하여 실제 섭씨온도 도출
                                float real_calculated_temp = (float)raw_temp / 100.0f;

                                // 고전압 배터리 팩 방재 스펙 이외의 일시적인 하드웨어 튐(노이즈) 억제 마진 필터
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

        // [3] 경량 공간 영상처리 (매 루프 변수 클리어 필수)
        int hot_pixels = 0;
        float current_max = -40.0f;

        for (int i = 0; i < 32 * 24; i++) {
            float p = mlx90640_frame[i];

            // 데이터 부재 상태인 완전 초기 공백만 상온 평탄화 보정
            if (p == 0.0f) {
                mlx90640_frame[i] = 24.0f;
                p = 24.0f;
            }

            if (p > current_max)
                current_max = p;
            if (p >= TEMP_FIRE_PIXEL) {
                hot_pixels++; // 고온 면적 누적 카운팅
            }
        }
        max_temp = current_max;
        high_temp_pixel_count = hot_pixels;

        // [4] 하이브리드 고전압 배터리 방재 최종 연산 (문법 컴파일 에러 수정 완결)
        bool is_gas_emergency = (mq_raw_value >= MQ_FIRE_THRESHOLD);
        bool is_area_emergency = (high_temp_pixel_count >= FIRE_MIN_AREA_PIXELS);

        if (is_gas_emergency && is_area_emergency) {
            hybrid_fire_triggered = true;
        } else {
            hybrid_fire_triggered = false;
        }

        // [5] 정수형 섭씨 스케일 클린 매트릭스 출력 뷰
        printf("\n=== [MLX90640 Aligned Subsystem Monitor (°C)] ===\n");
        for (int y = 0; y < 24; y += 3) {
            for (int x = 0; x < 32; x += 4) {
                float val = mlx90640_frame[y * 32 + x];
                if (val >= TEMP_FIRE_PIXEL) {
                    printf("!%3.0f! ", val); // 75도 이상 위험 구역 마킹
                } else {
                    printf(" %3.0f  ", val);
                }
            }
            printf("\n");
        }
        printf("-----------------------------------------------------\n");
        printf("[DATA] Gas: %d | Peak: %.1f °C | Area: %d px\n",
               mq_raw_value, max_temp, high_temp_pixel_count);
        printf("[DIAG] Safety Status: %s | Cooling Pump: %s\n",
               hybrid_fire_triggered ? "!! DANGER (TRIGGERED) !!" : "SAFE (MONITORING)",
               gpio_get_level(PIN_COOLING_PUMP) ? "RUNNING (순환 중)" : "STOP (대기)");
        printf("-----------------------------------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    init_hardware();
    // Core 1 할당: 오직 안전 스위치 폴링 및 모터/밸브 기구부 신속 제어 담당 (우선순위 최고)
    xTaskCreatePinnedToCore(vTaskEmergencyMonitor, "EmergencyMonitor", 4096, NULL, 3, NULL, 1);
    // Core 0 할당: 대용량 패킷 덤프 파싱 및 공간 행렬 연산 처리 담당 (스택 8KB 증설)
    xTaskCreatePinnedToCore(vTaskSensorAndImageProcessing, "SensorProcessor", 8192, NULL, 1, NULL, 0);
}