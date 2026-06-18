#!/usr/bin/env python3
"""
KJH_EV4 실시간 열화상 모니터 & 데이터 로거
=============================================
ESP32-S3 시리얼 출력에서 $DATA 라인을 파싱하여:
  - MLX90640 32×24 열화상 실시간 시각화
  - MQ 가스 센서 / 최고온도 / 화재 상태 실시간 표시
  - 모든 데이터를 CSV 파일로 자동 저장

사용법:
  pip install pyserial matplotlib numpy
  python thermal_monitor.py --port COM3          (Windows)
  python thermal_monitor.py --port /dev/ttyUSB0  (Linux)
  python thermal_monitor.py --port /dev/cu.usbmodem* (macOS)
  python thermal_monitor.py --list               (포트 목록 출력)
"""

import argparse
import csv
import os
import queue
import sys
import threading
import time
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial 이 설치되지 않았습니다.")
    print("        pip install pyserial")
    sys.exit(1)

# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
BAUD_RATE     = 115200
FRAME_ROWS    = 24
FRAME_COLS    = 32
FRAME_PIXELS  = FRAME_ROWS * FRAME_COLS   # 768
CSV_DIR       = "logs"
CMAP          = "inferno"          # 열화상 컬러맵: inferno / plasma / hot
VMIN_DEFAULT  = 20.0               # 컬러맵 하한 (°C)
VMAX_DEFAULT  = 80.0               # 컬러맵 상한 (°C)
MAX_HISTORY   = 300                # 시계열 그래프에 표시할 최대 샘플 수
UPDATE_MS     = 200                # 화면 갱신 주기 (ms)

# ---------------------------------------------------------------------------
# 전역 공유 상태
# ---------------------------------------------------------------------------
data_queue: "queue.Queue[dict]" = queue.Queue(maxsize=10)
latest_frame = {
    "pixels":    np.full((FRAME_ROWS, FRAME_COLS), 25.0),
    "mq":        0,
    "max_temp":  0.0,
    "hot_pixels":0,
    "fire":      False,
    "tick_ms":   0,
    "ts":        "",
}
history = {
    "time":      [],
    "max_temp":  [],
    "mq":        [],
    "hot_pixels":[],
}
csv_writer  = None
csv_file_h  = None
frame_count = 0
drop_count  = 0
last_fire   = False


# ---------------------------------------------------------------------------
# 직렬 포트 열기
# ---------------------------------------------------------------------------
def open_serial(port: str) -> serial.Serial:
    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2.0,
        )
        print(f"[OK] 포트 열림: {port}  ({BAUD_RATE} baud)")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] 포트 열기 실패: {e}")
        sys.exit(1)


# ---------------------------------------------------------------------------
# CSV 파일 초기화
# ---------------------------------------------------------------------------
def init_csv() -> None:
    global csv_writer, csv_file_h
    os.makedirs(CSV_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(CSV_DIR, f"thermal_{ts}.csv")
    csv_file_h = open(path, "w", newline="", encoding="utf-8")

    header = ["timestamp", "tick_ms", "mq_raw", "max_temp_c",
              "hot_pixels", "fire_triggered"]
    header += [f"t{i:03d}" for i in range(FRAME_PIXELS)]

    csv_writer = csv.writer(csv_file_h)
    csv_writer.writerow(header)
    csv_file_h.flush()
    print(f"[OK] CSV 로그 파일: {path}")


# ---------------------------------------------------------------------------
# $DATA 라인 파서
# ---------------------------------------------------------------------------
def parse_data_line(line: str) -> dict | None:
    """$DATA,tick,mq,max_temp,hot_pixels,fire,t0,...t767 → dict"""
    if not line.startswith("$DATA,"):
        return None
    parts = line.split(",")
    # 최소 길이: 6 헤더 필드 + 768 픽셀 = 774
    if len(parts) < 6 + FRAME_PIXELS:
        return None
    try:
        tick_ms   = int(parts[1])
        mq        = int(parts[2])
        max_temp  = float(parts[3])
        hot_px    = int(parts[4])
        fire      = bool(int(parts[5]))
        pixels    = [float(x) for x in parts[6 : 6 + FRAME_PIXELS]]
        return {
            "tick_ms":   tick_ms,
            "mq":        mq,
            "max_temp":  max_temp,
            "hot_pixels":hot_px,
            "fire":      fire,
            "pixels":    pixels,
            "ts":        datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        }
    except (ValueError, IndexError):
        return None


# ---------------------------------------------------------------------------
# 시리얼 수신 스레드
# ---------------------------------------------------------------------------
def serial_reader_thread(ser: serial.Serial) -> None:
    global frame_count, drop_count
    print("[INFO] 시리얼 수신 스레드 시작")
    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            parsed = parse_data_line(line)
            if parsed is None:
                continue
            frame_count += 1
            try:
                data_queue.put_nowait(parsed)
            except queue.Full:
                drop_count += 1
                data_queue.get_nowait()   # 오래된 프레임 제거
                data_queue.put_nowait(parsed)
        except serial.SerialException as e:
            print(f"[ERROR] 시리얼 오류: {e}")
            break
        except Exception as e:
            print(f"[WARN] 파싱 오류: {e}")


# ---------------------------------------------------------------------------
# matplotlib 실시간 업데이트
# ---------------------------------------------------------------------------
def build_figure():
    fig = plt.figure(figsize=(16, 8), facecolor="#1a1a2e")
    fig.canvas.manager.set_window_title("KJH_EV4 — 열화상 실시간 모니터")
    gs = gridspec.GridSpec(
        2, 3,
        figure=fig,
        left=0.06, right=0.97,
        top=0.92, bottom=0.08,
        wspace=0.38, hspace=0.40,
    )

    ax_thermal = fig.add_subplot(gs[:, 0])   # 열화상 (2행 차지)
    ax_temp    = fig.add_subplot(gs[0, 1])   # 최고온도 시계열
    ax_mq      = fig.add_subplot(gs[1, 1])   # 가스센서 시계열
    ax_status  = fig.add_subplot(gs[:, 2])   # 상태 패널

    # --- 열화상 ---
    init_data = np.full((FRAME_ROWS, FRAME_COLS), 25.0)
    im = ax_thermal.imshow(
        init_data,
        cmap=CMAP, vmin=VMIN_DEFAULT, vmax=VMAX_DEFAULT,
        aspect="auto", interpolation="bilinear",
    )
    cbar = fig.colorbar(im, ax=ax_thermal, fraction=0.046, pad=0.04)
    cbar.set_label("°C", color="white")
    cbar.ax.yaxis.set_tick_params(color="white")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="white")

    ax_thermal.set_title("MLX90640 열화상 (32×24)", color="white", pad=6)
    ax_thermal.set_xlabel("Column", color="white")
    ax_thermal.set_ylabel("Row", color="white")
    ax_thermal.tick_params(colors="white")
    for spine in ax_thermal.spines.values():
        spine.set_edgecolor("#444")

    # 십자선 (최고온도 위치 표시)
    hline = ax_thermal.axhline(y=0, color="cyan", linewidth=0.8, alpha=0.7)
    vline = ax_thermal.axvline(x=0, color="cyan", linewidth=0.8, alpha=0.7)

    # --- 최고온도 시계열 ---
    (line_temp,) = ax_temp.plot([], [], color="#ff6b6b", linewidth=1.2)
    ax_temp.set_facecolor("#16213e")
    ax_temp.set_title("최고 온도 (°C)", color="white", pad=4, fontsize=9)
    ax_temp.tick_params(colors="white", labelsize=7)
    ax_temp.set_ylabel("°C", color="white", fontsize=8)
    for spine in ax_temp.spines.values():
        spine.set_edgecolor("#444")
    fire_thresh_line = ax_temp.axhline(y=75, color="red", linestyle="--",
                                        linewidth=0.8, alpha=0.6, label="화재 기준")
    ax_temp.legend(fontsize=7, facecolor="#16213e", labelcolor="white",
                   loc="upper left")

    # --- MQ 가스 시계열 ---
    (line_mq,) = ax_mq.plot([], [], color="#4ecdc4", linewidth=1.2)
    ax_mq.set_facecolor("#16213e")
    ax_mq.set_title("MQ 가스 센서 (ADC)", color="white", pad=4, fontsize=9)
    ax_mq.tick_params(colors="white", labelsize=7)
    ax_mq.set_ylabel("ADC", color="white", fontsize=8)
    for spine in ax_mq.spines.values():
        spine.set_edgecolor("#444")
    ax_mq.axhline(y=300, color="orange", linestyle="--", linewidth=0.8,
                   alpha=0.6, label="경계(300)")
    ax_mq.axhline(y=1200, color="red", linestyle="--", linewidth=0.8,
                   alpha=0.6, label="위험(1200)")
    ax_mq.legend(fontsize=7, facecolor="#16213e", labelcolor="white",
                  loc="upper left")

    # --- 상태 패널 ---
    ax_status.set_facecolor("#0f3460")
    ax_status.set_xlim(0, 1)
    ax_status.set_ylim(0, 1)
    ax_status.axis("off")

    txt_fire   = ax_status.text(0.5, 0.88, "SAFE", ha="center", va="center",
                                 fontsize=22, fontweight="bold", color="#2ecc71",
                                 transform=ax_status.transAxes)
    txt_temp   = ax_status.text(0.5, 0.70, "Max: --.-°C", ha="center",
                                 fontsize=14, color="white",
                                 transform=ax_status.transAxes)
    txt_hot    = ax_status.text(0.5, 0.58, "Hot Pixels: --", ha="center",
                                 fontsize=12, color="white",
                                 transform=ax_status.transAxes)
    txt_gas    = ax_status.text(0.5, 0.46, "Gas ADC: ----", ha="center",
                                 fontsize=12, color="white",
                                 transform=ax_status.transAxes)
    txt_frames = ax_status.text(0.5, 0.32, "Frames: 0", ha="center",
                                 fontsize=10, color="#aaaaaa",
                                 transform=ax_status.transAxes)
    txt_drop   = ax_status.text(0.5, 0.22, "Drop: 0", ha="center",
                                 fontsize=9, color="#aaaaaa",
                                 transform=ax_status.transAxes)
    txt_tick   = ax_status.text(0.5, 0.12, "Tick: 0 ms", ha="center",
                                 fontsize=9, color="#aaaaaa",
                                 transform=ax_status.transAxes)

    fig.patch.set_facecolor("#1a1a2e")

    return (fig, im, hline, vline, line_temp, line_mq,
            txt_fire, txt_temp, txt_hot, txt_gas,
            txt_frames, txt_drop, txt_tick,
            ax_temp, ax_mq)


def update_plot(frame_idx, im, hline, vline, line_temp, line_mq,
                txt_fire, txt_temp, txt_hot, txt_gas,
                txt_frames, txt_drop, txt_tick,
                ax_temp, ax_mq):
    global csv_writer, csv_file_h, last_fire

    # 큐에서 최신 데이터 드레인 (가장 최근 것 사용)
    data = None
    while not data_queue.empty():
        try:
            data = data_queue.get_nowait()
        except queue.Empty:
            break

    if data is None:
        return (im, hline, vline, line_temp, line_mq,
                txt_fire, txt_temp, txt_hot, txt_gas,
                txt_frames, txt_drop, txt_tick)

    pixels_arr = np.array(data["pixels"]).reshape(FRAME_ROWS, FRAME_COLS)

    # --- 열화상 업데이트 ---
    im.set_data(pixels_arr)
    # 자동 컬러 범위 (여유 ±2°C)
    pmin = max(VMIN_DEFAULT, float(pixels_arr.min()) - 2)
    pmax = min(150.0, float(pixels_arr.max()) + 2)
    if pmax > pmin + 1:
        im.set_clim(vmin=pmin, vmax=pmax)

    # 최고온도 위치에 십자선
    hot_idx = int(pixels_arr.argmax())
    hot_row, hot_col = divmod(hot_idx, FRAME_COLS)
    hline.set_ydata([hot_row])
    vline.set_xdata([hot_col])

    # --- 시계열 업데이트 ---
    history["time"].append(data["tick_ms"] / 1000.0)
    history["max_temp"].append(data["max_temp"])
    history["mq"].append(data["mq"])
    history["hot_pixels"].append(data["hot_pixels"])

    # 표시 범위 제한
    for k in history:
        if len(history[k]) > MAX_HISTORY:
            history[k] = history[k][-MAX_HISTORY:]

    t = history["time"]
    line_temp.set_data(t, history["max_temp"])
    ax_temp.set_xlim(t[0], max(t[-1], t[0] + 1))
    ax_temp.set_ylim(
        max(0, min(history["max_temp"]) - 5),
        max(100, max(history["max_temp"]) + 5),
    )

    line_mq.set_data(t, history["mq"])
    ax_mq.set_xlim(t[0], max(t[-1], t[0] + 1))
    ax_mq.set_ylim(0, max(1500, max(history["mq"]) + 100))

    # --- 상태 패널 ---
    is_fire = data["fire"]
    if is_fire:
        txt_fire.set_text("🔥 FIRE !!")
        txt_fire.set_color("#e74c3c")
        if not last_fire:
            print(f"\n{'!'*50}\n[FIRE ALERT] {data['ts']}  Max={data['max_temp']}°C  Gas={data['mq']}\n{'!'*50}")
    else:
        txt_fire.set_text("✔  SAFE")
        txt_fire.set_color("#2ecc71")
    last_fire = is_fire

    txt_temp.set_text(f"Max: {data['max_temp']:.1f} °C")
    txt_hot.set_text(f"Hot Pixels: {data['hot_pixels']}")
    txt_gas.set_text(f"Gas ADC: {data['mq']}")
    txt_frames.set_text(f"Frames: {frame_count}")
    txt_drop.set_text(f"Drop: {drop_count}")
    txt_tick.set_text(f"Tick: {data['tick_ms']} ms")

    # --- CSV 로깅 ---
    if csv_writer is not None:
        row = [data["ts"], data["tick_ms"], data["mq"],
               f"{data['max_temp']:.2f}", data["hot_pixels"],
               int(data["fire"])]
        row += [f"{v:.2f}" for v in data["pixels"]]
        csv_writer.writerow(row)
        csv_file_h.flush()

    return (im, hline, vline, line_temp, line_mq,
            txt_fire, txt_temp, txt_hot, txt_gas,
            txt_frames, txt_drop, txt_tick)


# ---------------------------------------------------------------------------
# 진입점
# ---------------------------------------------------------------------------
def list_ports() -> None:
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("사용 가능한 시리얼 포트가 없습니다.")
        return
    print("사용 가능한 시리얼 포트:")
    for p in ports:
        print(f"  {p.device:20s} {p.description}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="KJH_EV4 열화상 실시간 모니터 & CSV 로거"
    )
    parser.add_argument("--port", "-p", help="시리얼 포트 (예: COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=BAUD_RATE,
                        help=f"보드레이트 (기본값: {BAUD_RATE})")
    parser.add_argument("--list", "-l", action="store_true",
                        help="포트 목록 출력 후 종료")
    parser.add_argument("--no-csv", action="store_true",
                        help="CSV 저장 비활성화")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        print("[ERROR] --port 옵션으로 포트를 지정하세요.")
        print("        python thermal_monitor.py --list  으로 포트 목록 확인")
        sys.exit(1)

    if not args.no_csv:
        init_csv()

    ser = open_serial(args.port)

    # 수신 스레드 시작
    t = threading.Thread(target=serial_reader_thread, args=(ser,), daemon=True)
    t.start()

    # matplotlib 그래프 생성
    (fig, im, hline, vline, line_temp, line_mq,
     txt_fire, txt_temp, txt_hot, txt_gas,
     txt_frames, txt_drop, txt_tick,
     ax_temp, ax_mq) = build_figure()

    ani = animation.FuncAnimation(
        fig,
        update_plot,
        fargs=(im, hline, vline, line_temp, line_mq,
               txt_fire, txt_temp, txt_hot, txt_gas,
               txt_frames, txt_drop, txt_tick,
               ax_temp, ax_mq),
        interval=UPDATE_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file_h:
            csv_file_h.close()
            print(f"\n[OK] CSV 저장 완료. 총 {frame_count} 프레임 기록.")
        ser.close()


if __name__ == "__main__":
    main()
