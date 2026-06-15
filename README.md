# KJH_EV4

전기차(EV) 고전압 배터리 화재 방재용 **이동식 방수포 수조 제어 시스템**입니다.
ESP32-S3 기반으로, 열화상 카메라와 가스 센서를 이용해 배터리 팩의 열폭주(thermal runaway)를 감지하고,
스텝 모터로 리프트를 구동해 침수 수조를 형성한 뒤 냉각수를 무제한 순환시켜 화재를 진압합니다.

## 하드웨어 구성

- **MCU**: ESP32-S3 (4D Systems Gen4 R8N16, N16R8)
- **스텝 모터 드라이버**: TB6600 (방수포 리프트 구동)
- **열화상 카메라**: GY-MCU90640 (MLX90640, UART2 통신, 32x24 픽셀)
- **가스 센서**: MQ 시리즈 (ADC1 입력)
- **비상 스위치**: 수동 하강/복귀 제어용 입력 (풀업, active-low)
- **냉각수 펌프/릴레이**: 화재 진압용 주수 제어

## 핀맵

| 기능 | GPIO | 설명 |
| --- | --- | --- |
| 모터 펄스 (PUL) | 4 | TB6600 스텝 신호 |
| 모터 방향 (DIR) | 5 | TB6600 회전 방향 신호 |
| 비상 스위치 | 6 | 수동 하강/복귀 제어 (풀업, active-low) |
| 냉각수 펌프 | 8 | 펌프/릴레이 제어 |
| UART2 RX | 17 | GY-MCU90640 TX |
| UART2 TX | 18 | GY-MCU90640 RX |
| 가스 센서 (ADC1) | CH6 | MQ 가스 센서 아날로그 입력 |

## 동작 개요

펌웨어는 FreeRTOS 위에서 두 개의 태스크로 동작합니다.

### Task 1 — `vTaskEmergencyMonitor` (Core 1, 최우선순위)

리프트/펌프 하드웨어를 제어하는 상태 머신입니다.

- **화재 감지 시**: 즉시 비상 복귀 동작(`move_emergency_return`)을 수행하고, 화재 신호(`hybrid_fire_triggered`)가
  해제될 때까지 냉각수 펌프를 무제한 순환 가동합니다. 화재 해제 후에는 `needs_position_reset` 플래그가
  설정되어, 다음 버튼 입력 시 무조건 초기 위치(상단 원점)로 강제 복귀합니다.
- **평상시 (화재 미감지)**:
  - `STATE_READY_REVERSE` (상단 원점 대기): 스위치를 짧게 누르면 리프트 하강(`move_reverse_with_mon`),
    `BUTTON_RESET_HOLD_MS`(기본 3초) 이상 길게 누르면 초기 위치(상단 원점)로 강제 복귀합니다.
  - `STATE_READY_FORWARD` (하단 감시 대기): 스위치를 누르면 리프트를 상단으로 복귀시킵니다
    (`move_forward_return`).
- 리프트가 복귀(상승) 방향으로 1/3 지점을 통과하면 냉각수 펌프를 자동으로 가동해 수조 주수를 시작합니다.

### Task 2 — `vTaskSensorAndImageProcessing` (Core 0)

- GY-MCU90640 UART 프레임(1544바이트, 헤더 `0x5A 0x5A`)을 동기화하여 32x24 온도 매트릭스로 파싱합니다.
- MQ 가스 센서 값에 따라 화재 판정 임계 온도/면적을 동적으로 조정합니다.
- 임계 온도 이상 픽셀 수가 기준 면적을 넘으면 `hybrid_fire_triggered`를 설정합니다.
- 디버깅용으로 온도 매트릭스와 센서 상태를 시리얼 모니터에 주기적으로 출력합니다.

## 주요 파라미터 (`src/main.c`)

| 매크로 | 기본값 | 설명 |
| --- | --- | --- |
| `MOTOR_PULSES_PER_REV` | 800 | 스텝 모터 1회전당 펄스 수 |
| `MOTOR_TARGET_REV` | 20.0 | 리프트 유효 이동 거리 (회전 수) |
| `MOTOR_PULSE_DELAY_US` | 400 | 스텝 펄스 지연 (구동 속도) |
| `BUTTON_RESET_HOLD_MS` | 3000 | 초기 위치 강제 복귀 롱프레스 판정 시간 (ms) |
| `TEMP_FIRE_PIXEL` | 75.0 | 기본 화재 판정 픽셀 온도 (°C) |
| `FIRE_MIN_AREA_PIXELS` | 12 | 화재 확정 최소 픽셀 면적 |

## 빌드 및 실행

이 프로젝트는 PlatformIO + ESP-IDF 프레임워크 기반입니다.

```bash
# 빌드
pio run

# 플래시 업로드
pio run -t upload

# 시리얼 모니터 (115200 baud)
pio device monitor
```

대상 보드: `4d_systems_esp32s3_gen4_r8n16` (16MB Flash / 8MB PSRAM, QIO/OPI)
