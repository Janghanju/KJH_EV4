#!/usr/bin/env python3
"""
KJH_EV4 실시간 열화상 모니터 GUI  v3.0
========================================
실행:  python3 thermal_monitor.py
설치:  python3 -m pip install pyserial matplotlib numpy

연결 모드
  - GY-MCU90640 직접: 센서를 USB-UART 어댑터로 PC에 직결, 1544바이트 바이너리 프레임 읽기
  - ESP32 릴레이:     ESP32 USB에 연결, $H/$DATA 텍스트 라인 파싱
  - 자동 감지:        연결 후 첫 바이트를 확인해 모드 자동 결정
"""

import csv
import os
import queue
import subprocess
import sys
import threading
import time as _time
from datetime import datetime
from tkinter import filedialog, messagebox
from tkinter.scrolledtext import ScrolledText
import tkinter as tk

# ─── 패키지 체크 ─────────────────────────────────────────────────────────────
_miss = []
try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    import matplotlib.font_manager as _fm
    import matplotlib.gridspec as gridspec
    import numpy as np

    _KO = next(
        (f for f in ["Apple SD Gothic Neo", "AppleGothic",
                     "Malgun Gothic", "NanumGothic", "Noto Sans KR"]
         if f in {x.name for x in _fm.fontManager.ttflist}), None
    )
    if _KO:
        matplotlib.rcParams["font.family"] = _KO
    matplotlib.rcParams["axes.unicode_minus"] = False
except ImportError:
    _miss.append("matplotlib  numpy")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    _miss.append("pyserial")

if _miss:
    import tkinter as _tk
    _r = _tk.Tk(); _r.withdraw()
    messagebox.showerror("패키지 누락",
        "필수 패키지 미설치:\n\n" + "\n".join(f"  • {m}" for m in _miss) +
        "\n\n터미널에서 실행:\n  python3 -m pip install pyserial matplotlib numpy")
    sys.exit(1)

# ─── 상수 ────────────────────────────────────────────────────────────────────
BAUD         = 115200
ROWS, COLS   = 24, 32
NPIX         = ROWS * COLS
MAX_HIST     = 300
DEFAULT_DIR  = os.path.join(os.path.expanduser("~"), "Desktop", "KJH_EV4_logs")

# 색상 팔레트
BG   = "#1a1a2e"   # 배경
MID  = "#16213e"   # 툴바/패널 배경
PNL  = "#0f3460"   # 상태 패널
DRK  = "#0a0a18"   # 로그 배경
WHT  = "#e0e0e0"
GRN  = "#2ecc71"
RED  = "#e74c3c"
ORG  = "#f39c12"
CYN  = "#4ecdc4"
ACC  = "#533483"


# ─── 메인 앱 ─────────────────────────────────────────────────────────────────
class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("KJH_EV4  —  열화상 실시간 모니터  v2.0")
        self.root.configure(bg=BG)
        self.root.geometry("1440x920")
        self.root.minsize(1100, 750)

        # 런타임 상태
        self.ser          = None
        self.running      = False
        self.binary_mode  = False   # True = GY-MCU90640 직접 바이너리, False = ESP32 텍스트
        self.data_q       = queue.Queue(maxsize=30)
        self.csv_file     = None
        self.csv_writer   = None
        self.frame_count  = 0
        self.drop_count   = 0
        self.parse_fail   = 0
        self.last_fire    = False
        self._fps_cnt     = 0
        self._fps_ts      = datetime.now().timestamp()
        self._frame_t0    = _time.time()  # FPS 계산용

        # 그래프 히스토리
        self.ht: list = []
        self.htemp: list = []
        self.hmq: list   = []

        # CSV 저장 경로 변수
        self.save_dir = tk.StringVar(value=DEFAULT_DIR)

        # UI 구성 — 순서 중요: BOTTOM 요소를 먼저 pack
        self._build_log()        # pack BOTTOM 1st
        self._build_data_bar()   # pack BOTTOM 2nd
        self._build_toolbar()    # pack TOP
        self._build_main()       # pack FILL/EXPAND last

        self._tick()

    # =========================================================================
    # 1. 툴바 (포트·연결·저장경로·CSV)
    # =========================================================================
    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg=MID, pady=7, padx=10)
        bar.pack(side=tk.TOP, fill=tk.X)

        # ── 포트 선택 ───────────────────────────────────────────────────────
        self._lb(bar, "포트:").pack(side=tk.LEFT, padx=(0, 3))

        self.port_var = tk.StringVar()
        self.port_cb  = tk.OptionMenu(bar, self.port_var, "")
        self.port_cb.configure(bg=PNL, fg=WHT, activebackground=ACC,
                               font=("Helvetica", 9), relief="flat",
                               highlightthickness=0, width=22)
        self.port_cb["menu"].configure(bg=PNL, fg=WHT)
        self.port_cb.pack(side=tk.LEFT, padx=(0, 3))

        self._btn(bar, "⟳ 새로고침", self._refresh_ports,
                  bg=PNL).pack(side=tk.LEFT, padx=(0, 6))

        # ── 연결 모드 선택 ──────────────────────────────────────────────────
        self._lb(bar, "모드:").pack(side=tk.LEFT, padx=(0, 3))
        self.mode_var = tk.StringVar(value="자동 감지")
        mode_menu = tk.OptionMenu(bar, self.mode_var,
                                  "자동 감지", "GY-MCU90640 직접", "ESP32 릴레이")
        mode_menu.configure(bg=PNL, fg=WHT, activebackground=ACC,
                            font=("Helvetica", 9), relief="flat",
                            highlightthickness=0, width=14)
        mode_menu["menu"].configure(bg=PNL, fg=WHT)
        mode_menu.pack(side=tk.LEFT, padx=(0, 8))

        self.conn_btn = self._btn(bar, "▶  연결", self._toggle_connect, bg="#1a6b3a")
        self.conn_btn.pack(side=tk.LEFT, padx=(0, 8))

        self._vsep(bar)

        # ── 저장 경로 ───────────────────────────────────────────────────────
        self._lb(bar, "저장 경로:").pack(side=tk.LEFT, padx=(0, 4))

        path_entry = tk.Entry(bar, textvariable=self.save_dir,
                              width=32, bg=DRK, fg=CYN, relief="flat",
                              font=("Helvetica", 9), insertbackground=WHT)
        path_entry.pack(side=tk.LEFT, padx=(0, 3), ipady=3)

        self._btn(bar, "폴더 선택", self._browse_dir,
                  bg=PNL).pack(side=tk.LEFT, padx=(0, 3))
        self._btn(bar, "📂 열기", self._open_dir,
                  bg=PNL).pack(side=tk.LEFT, padx=(0, 8))

        self._vsep(bar)

        # ── CSV 토글 ────────────────────────────────────────────────────────
        self.csv_var = tk.BooleanVar(value=True)
        tk.Checkbutton(bar, text="CSV 자동저장", variable=self.csv_var,
                       bg=MID, fg=WHT, activebackground=MID,
                       activeforeground=WHT, selectcolor=ACC,
                       font=("Helvetica", 9)).pack(side=tk.LEFT, padx=(0, 8))

        # ── 오른쪽: 상태 표시 ───────────────────────────────────────────────
        self.frame_lbl = self._lb(bar, "프레임: 0  |  드롭: 0",
                                  color="#888888", size=8)
        self.frame_lbl.pack(side=tk.RIGHT, padx=10)

        self.conn_lbl = self._lb(bar, "●  연결 끊김", color="#555555", bold=True)
        self.conn_lbl.pack(side=tk.RIGHT, padx=8)

        self._refresh_ports()

    # =========================================================================
    # 2. 메인 영역 (열화상 | 상태패널 + 그래프)
    # =========================================================================
    def _build_main(self):
        mid = tk.Frame(self.root, bg=BG)
        mid.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(6, 0))

        left  = tk.Frame(mid, bg=BG)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = tk.Frame(mid, bg=BG, width=420)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(8, 0))
        right.pack_propagate(False)

        self._build_thermal(left)
        self._build_status(right)
        self._build_graphs(right)

    # ── 열화상 패널 ──────────────────────────────────────────────────────────
    def _build_thermal(self, parent):
        self._lb(parent, "MLX90640 열화상  (32 × 24 pixels)",
                 size=11, color=CYN, bold=True).pack(pady=(0, 4))

        self.fig_th = Figure(figsize=(6.2, 5.2), facecolor=BG)
        self.ax_th  = self.fig_th.add_subplot(111)
        self.ax_th.set_facecolor(MID)

        self.im = self.ax_th.imshow(
            np.full((ROWS, COLS), 25.0),
            cmap="inferno", vmin=20, vmax=80,
            aspect="auto", interpolation="bicubic"  # 참조 구현처럼 부드러운 보간
        )

        cb = self.fig_th.colorbar(self.im, ax=self.ax_th, fraction=0.04, pad=0.02)
        cb.set_label("°C", color=WHT, fontsize=9)
        cb.ax.yaxis.set_tick_params(color=WHT, labelsize=7)
        for t in cb.ax.yaxis.get_ticklabels():
            t.set_color(WHT)

        self.ax_th.set_xlabel("Column (픽셀)", color=WHT, fontsize=8)
        self.ax_th.set_ylabel("Row (픽셀)",    color=WHT, fontsize=8)
        self.ax_th.tick_params(colors=WHT, labelsize=7)
        for sp in self.ax_th.spines.values():
            sp.set_edgecolor("#444")

        # 최고온도 위치 표시
        self.ch_h = self.ax_th.axhline(0, color="cyan", lw=1.0, ls="--", alpha=0.8)
        self.ch_v = self.ax_th.axvline(0, color="cyan", lw=1.0, ls="--", alpha=0.8)
        self.hot_txt = self.ax_th.text(1, 1, "", color="cyan", fontsize=9,
                                        fontweight="bold",
                                        bbox=dict(fc=BG, alpha=0.75, pad=2))

        canvas = FigureCanvasTkAgg(self.fig_th, master=parent)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas_th = canvas
        self.fig_th.tight_layout(pad=1.2)

    # ── 상태 패널 ────────────────────────────────────────────────────────────
    def _build_status(self, parent):
        pnl = tk.Frame(parent, bg=PNL, padx=14, pady=10)
        pnl.pack(fill=tk.X, pady=(0, 6))

        self.fire_lbl = tk.Label(pnl, text="✔  SAFE",
                                  bg=PNL, fg=GRN, font=("Helvetica", 28, "bold"))
        self.fire_lbl.pack()

        g = tk.Frame(pnl, bg=PNL)
        g.pack(fill=tk.X, pady=(8, 0))

        self.metrics = {}
        for i, (k, label, val) in enumerate([
            ("max_temp",   "최고 온도",   "--.- °C"),
            ("hot_pixels", "Hot Pixels",  "-- px"),
            ("gas_adc",    "Gas ADC",     "----"),
            ("fps",        "갱신 속도",   "-- fps"),
        ]):
            tk.Label(g, text=label + ":", bg=PNL, fg="#aaaaaa",
                     font=("Helvetica", 9), width=12, anchor="w").grid(
                row=i, column=0, sticky="w", pady=3)
            v = tk.Label(g, text=val, bg=PNL, fg=WHT,
                         font=("Helvetica", 12, "bold"), anchor="w")
            v.grid(row=i, column=1, sticky="w", pady=3)
            self.metrics[k] = v

    # ── 그래프 패널 ──────────────────────────────────────────────────────────
    def _build_graphs(self, parent):
        self.fig_g = Figure(figsize=(4.0, 4.0), facecolor=BG)
        gs = gridspec.GridSpec(2, 1, figure=self.fig_g,
                               hspace=0.55, top=0.93, bottom=0.10,
                               left=0.15, right=0.97)

        self.ax_temp = self._init_ax(self.fig_g.add_subplot(gs[0]), "최고 온도 (°C)")
        self.ax_temp.axhline(75, color="red", ls="--", lw=0.9, alpha=0.6, label="기준 75°C")
        self.ax_temp.legend(fontsize=6, facecolor=MID, labelcolor=WHT, loc="upper left")
        self.ln_temp, = self.ax_temp.plot([], [], color="#ff6b6b", lw=1.5)

        self.ax_mq = self._init_ax(self.fig_g.add_subplot(gs[1]), "Gas ADC")
        self.ax_mq.axhline(300,  color=ORG,   ls="--", lw=0.9, alpha=0.6, label="경계 300")
        self.ax_mq.axhline(1200, color="red", ls="--", lw=0.9, alpha=0.6, label="위험 1200")
        self.ax_mq.legend(fontsize=6, facecolor=MID, labelcolor=WHT, loc="upper left")
        self.ln_mq, = self.ax_mq.plot([], [], color=CYN, lw=1.5)

        canvas = FigureCanvasTkAgg(self.fig_g, master=parent)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas_g = canvas

    def _init_ax(self, ax, title: str):
        ax.set_facecolor(MID)
        ax.set_title(title, color=WHT, fontsize=8, pad=3)
        ax.tick_params(colors=WHT, labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor("#444")
        return ax

    # =========================================================================
    # 3. 수신 데이터 바 (하단 고정 — 파싱된 값 실시간 표시)
    # =========================================================================
    def _build_data_bar(self):
        bar = tk.Frame(self.root, bg=DRK, pady=5, padx=10)
        bar.pack(side=tk.BOTTOM, fill=tk.X)

        self._lb(bar, "수신 데이터:", color="#555", size=8).pack(side=tk.LEFT, padx=(0, 12))

        self.dlbl = {}
        for k, label in [
            ("tick",   "Tick(ms)"),
            ("mq",     "Gas ADC"),
            ("temp",   "최고온도"),
            ("hot",    "Hot px"),
            ("fire",   "화재"),
            ("frames", "프레임"),
        ]:
            self._lb(bar, label + ":", color="#555", size=8).pack(side=tk.LEFT)
            v = tk.Label(bar, text="---", bg=DRK, fg=CYN,
                         font=("Courier New", 9, "bold"), width=9, anchor="w")
            v.pack(side=tk.LEFT, padx=(2, 14))
            self.dlbl[k] = v

    # =========================================================================
    # 4. 시리얼 로그 패널 (하단 고정)
    # =========================================================================
    def _build_log(self):
        frm = tk.Frame(self.root, bg=BG)
        frm.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(2, 4))

        hdr = tk.Frame(frm, bg=BG)
        hdr.pack(fill=tk.X)
        self._lb(hdr, "시리얼 로그", color="#555", size=8).pack(side=tk.LEFT)
        self._btn(hdr, "지우기", self._clear_log, bg=BG,
                  fg="#555", size=7).pack(side=tk.RIGHT)

        self.log = ScrolledText(frm, height=5, bg=DRK, fg="#aaaaaa",
                                font=("Courier New", 8),
                                state=tk.DISABLED, wrap=tk.WORD)
        self.log.pack(fill=tk.X)
        self.log.tag_config("fire", foreground=RED)
        self.log.tag_config("warn", foreground=ORG)
        self.log.tag_config("info", foreground=CYN)

    # =========================================================================
    # 5. 포트 관리
    # =========================================================================
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        menu = self.port_cb["menu"]
        menu.delete(0, "end")
        for p in ports:
            menu.add_command(label=p, command=lambda v=p: self.port_var.set(v))
        if ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("(포트 없음)")
            self._log("연결된 시리얼 포트 없음 — USB 연결 후 ⟳ 클릭", "warn")

    # =========================================================================
    # 6. 연결 / 해제
    # =========================================================================
    def _toggle_connect(self):
        (self._disconnect if self.running else self._connect)()

    def _connect(self):
        port = self.port_var.get()
        if not port or port == "(포트 없음)":
            messagebox.showwarning("포트 없음",
                "디바이스를 USB로 연결한 뒤 ⟳ 클릭해 포트를 선택하세요.")
            return
        try:
            self.ser = serial.Serial(port=port, baudrate=BAUD,
                                     bytesize=serial.EIGHTBITS,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     timeout=2.0)
        except serial.SerialException as e:
            messagebox.showerror("연결 실패", str(e))
            return

        self.running = True
        self.binary_mode = False
        self.frame_count = self.drop_count = self._fps_cnt = self.parse_fail = 0
        self._fps_ts = self._frame_t0 = _time.time()

        # ── 모드 결정 ────────────────────────────────────────────────────────
        chosen = self.mode_var.get()
        if chosen == "GY-MCU90640 직접":
            self.binary_mode = True
        elif chosen == "ESP32 릴레이":
            self.binary_mode = False
        else:
            # 자동 감지: 첫 수신 바이트로 판단
            self.ser.timeout = 0.5
            probe = self.ser.read(16)
            self.ser.timeout = 2.0
            non_print = sum(1 for b in probe if b < 32 and b not in (0x0A, 0x0D))
            self.binary_mode = (non_print >= 3)
            self._log(
                f"자동 감지: {'바이너리(GY-MCU90640)' if self.binary_mode else '텍스트(ESP32)'} "
                f"[probe={probe.hex()}]", "info")

        mode_label = "GY-MCU90640 직접" if self.binary_mode else "ESP32 릴레이"
        self.conn_btn.configure(text="■  연결 해제", bg="#7b1c1c")
        self.conn_lbl.configure(text=f"●  {port}  [{mode_label}]", fg=GRN)
        self._log(f"포트 연결: {port}  ({BAUD} baud)  모드={mode_label}", "info")

        if self.csv_var.get():
            self._init_csv()

        reader = self._reader_binary if self.binary_mode else self._reader_text
        threading.Thread(target=reader, daemon=True).start()

    def _disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            if self.binary_mode:
                try:
                    # GY-MCU90640 자동 전송 중지 명령
                    self.ser.write(bytes([0xA5, 0x35, 0x01, 0xDB]))
                except Exception:
                    pass
            self.ser.close()
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = self.csv_writer = None
        self.conn_btn.configure(text="▶  연결", bg="#1a6b3a")
        self.conn_lbl.configure(text="●  연결 끊김", fg="#555555")
        self._log("포트 연결 해제", "warn")

    # =========================================================================
    # 7. CSV 관리
    # =========================================================================
    def _init_csv(self):
        d = self.save_dir.get()
        os.makedirs(d, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        mode = "bin" if self.binary_mode else "esp"
        fp  = os.path.join(d, f"thermal_{mode}_{ts}.csv")
        self.csv_file   = open(fp, "w", newline="", encoding="utf-8")
        hdr = ["timestamp", "tick_ms", "mq_raw", "max_temp_c",
               "hot_pixels", "fire_triggered", "ambient_c"]
        hdr += [f"t{i:03d}" for i in range(NPIX)]
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(hdr)
        self._log(f"CSV 시작: {fp}", "info")

    def _browse_dir(self):
        d = filedialog.askdirectory(initialdir=self.save_dir.get(),
                                    title="CSV 저장 폴더 선택")
        if d:
            self.save_dir.set(d)

    def _open_dir(self):
        d = self.save_dir.get()
        os.makedirs(d, exist_ok=True)
        if sys.platform == "darwin":
            subprocess.Popen(["open", d])
        elif sys.platform == "win32":
            subprocess.Popen(["explorer", d])
        else:
            subprocess.Popen(["xdg-open", d])

    # =========================================================================
    # 8a. 바이너리 수신 스레드 — GY-MCU90640 직접 연결 (참조 구현 기반)
    #     1544바이트 프레임: 0x5A 0x5A [2B] [768×int16 픽셀 LE] [2B ambient] [2B]
    # =========================================================================
    def _reader_binary(self):
        # 4Hz 자동 수집 시작 명령 (참조 코드와 동일)
        try:
            self.ser.write(bytes([0xA5, 0x25, 0x01, 0xCB]))  # set 4 Hz
            _time.sleep(0.1)
            self.ser.write(bytes([0xA5, 0x35, 0x02, 0xDC]))  # start auto
            self.root.after(0, self._log,
                "[바이너리] GY-MCU90640 4Hz 자동 수집 시작", "info")
        except Exception as e:
            self.root.after(0, self._log,
                f"[바이너리] 시작 명령 전송 실패: {e}", "warn")

        while self.running:
            try:
                # ── 0x5A 0x5A 동기 헤더 탐색 ─────────────────────────────
                b1 = self.ser.read(1)
                if not b1:
                    continue
                if b1[0] != 0x5A:
                    continue
                b2 = self.ser.read(1)
                if not b2 or b2[0] != 0x5A:
                    continue

                # ── 나머지 1542바이트 읽기 ───────────────────────────────
                rest = self.ser.read(1542)
                if len(rest) < 1542:
                    self.root.after(0, self._log,
                        f"[바이너리] 짧은 프레임: {len(rest)+2}/1544", "warn")
                    continue

                frame = bytes([0x5A, 0x5A]) + rest

                # ── 픽셀 온도 파싱 (bytes 4‥1539, int16 LE, ÷100 = °C) ──
                raw_pix = frame[4:1540]
                arr16   = np.frombuffer(raw_pix, dtype=np.int16)  # LE
                pixels  = (arr16.astype(np.float32) / 100.0).tolist()

                # ── 주변 온도 (bytes 1540‥1541, int16 LE) ────────────────
                ta = (int(frame[1540]) + int(frame[1541]) * 256) / 100.0

                max_t  = float(max(pixels))
                hot_px = sum(1 for p in pixels if p >= 45.0)
                now_t  = _time.time()
                fps    = 1.0 / max(now_t - self._frame_t0, 0.001)
                self._frame_t0 = now_t

                data = {
                    "tick_ms":    int(now_t * 1000) & 0x7FFFFFFF,
                    "mq":         0,
                    "max_temp":   max_t,
                    "hot_pixels": hot_px,
                    "fire":       max_t >= 50.0,
                    "pixels":     pixels,
                    "ambient":    ta,
                    "fps_bin":    fps,
                    "ts":         datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                }

                try:
                    self.data_q.put_nowait(data)
                except queue.Full:
                    self.drop_count += 1
                    try:
                        self.data_q.get_nowait()
                        self.data_q.put_nowait(data)
                    except queue.Empty:
                        pass

            except serial.SerialException:
                self.running = False
                self.root.after(0, self._disconnect)
                break
            except Exception as e:
                self.root.after(0, self._log, f"[binary error] {e}", "warn")

    # =========================================================================
    # 8b. 텍스트 수신 스레드 — ESP32 릴레이 ($H 신포맷 / $DATA 구포맷)
    # =========================================================================
    def _reader_text(self):
        while self.running:
            try:
                raw  = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                data = self._parse(line)
                if data:
                    try:
                        self.data_q.put_nowait(data)
                    except queue.Full:
                        self.drop_count += 1
                        try:
                            self.data_q.get_nowait()
                            self.data_q.put_nowait(data)
                        except queue.Empty:
                            pass
                elif line.startswith("$H,") or line.startswith("$DATA,"):
                    self.parse_fail += 1
                    if self.parse_fail <= 5 or self.parse_fail % 50 == 0:
                        self.root.after(0, self._log,
                            f"[parse FAIL #{self.parse_fail}] "
                            f"fmt={'$H' if line.startswith('$H') else '$DATA'} "
                            f"len={len(line)}", "warn")
                elif line and not line.startswith("$"):
                    self.root.after(0, self._log, line[:160])
            except serial.SerialException:
                self.running = False
                self.root.after(0, self._disconnect)
                break
            except Exception as e:
                self.root.after(0, self._log, f"[reader error] {e}", "warn")

    @staticmethod
    def _parse(line: str):
        """
        두 가지 ESP32 출력 포맷 모두 지원:
          $H  포맷 (신): $H,tick,mq,maxtemp*100,hot,fire,<3072 hex>
          $DATA 포맷 (구): $DATA,tick,mq,max_temp,hot,fire,t0,t1,...,t767
        """
        if line.startswith("$H,"):
            return App._parse_h(line)
        if line.startswith("$DATA,"):
            return App._parse_data(line)
        return None

    @staticmethod
    def _parse_h(line: str):
        """$H 컴팩트 hex 포맷 — ESP32 printf('%04X') = big-endian value"""
        p = line.split(",", 6)
        if len(p) < 7:
            return None
        try:
            hex_blob = p[6].strip()
            if len(hex_blob) < NPIX * 4:
                return None
            raw   = bytes.fromhex(hex_blob[:NPIX * 4])
            arr16 = np.frombuffer(raw, dtype=">i2")   # big-endian (MSB first)
            pixels = (arr16.astype(np.float32) / 100.0).tolist()
            return {
                "tick_ms":    int(p[1]),
                "mq":         int(p[2]),
                "max_temp":   int(p[3]) / 100.0,
                "hot_pixels": int(p[4]),
                "fire":       bool(int(p[5])),
                "pixels":     pixels,
                "ts":         datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            }
        except Exception:
            return None

    @staticmethod
    def _parse_data(line: str):
        """$DATA 레거시 float 텍스트 포맷 (구 펌웨어 호환)"""
        p = line.split(",")
        if len(p) < 6 + NPIX:
            return None
        try:
            pixels = [float(x) for x in p[6: 6 + NPIX]]
            return {
                "tick_ms":    int(p[1]),
                "mq":         int(p[2]),
                "max_temp":   float(p[3]),
                "hot_pixels": int(p[4]),
                "fire":       bool(int(p[5])),
                "pixels":     pixels,
                "ts":         datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            }
        except Exception:
            return None

    # =========================================================================
    # 9. 200ms 렌더링 루프
    # =========================================================================
    def _tick(self):
        try:
            data = None
            while not self.data_q.empty():
                try:
                    data = self.data_q.get_nowait()
                    self.frame_count += 1
                    self._fps_cnt    += 1
                except queue.Empty:
                    break

            if data:
                self._render(data)

            now = datetime.now().timestamp()
            if now - self._fps_ts >= 1.0:
                fps = self._fps_cnt / max(now - self._fps_ts, 0.001)
                self.metrics["fps"].configure(text=f"{fps:.1f} fps")
                self._fps_cnt = 0
                self._fps_ts  = now

            self.frame_lbl.configure(
                text=f"프레임: {self.frame_count}  |  드롭: {self.drop_count}")
        except Exception as e:
            # 에러를 로그에 표시 (silent swallow 금지)
            try:
                self._log(f"[tick error] {e}", "warn")
            except Exception:
                pass
        finally:
            self.root.after(200, self._tick)

    def _render(self, d: dict):
        # ── 열화상 업데이트 (참조 구현처럼 int16→float 배열 직접 렌더) ──────
        arr = np.array(d["pixels"], dtype=np.float32).reshape(ROWS, COLS)

        # 동적 범위: 실제 min/max 기반 (참조 구현의 Tmin/Tmax 고정 대신 자동)
        lo = float(arr.min())
        hi = float(arr.max())
        if hi - lo < 1.0:          # 차이가 너무 작으면 ±5 여유 추가
            lo -= 5.0; hi += 5.0
        self.im.set_data(arr)
        self.im.set_clim(lo, hi)

        # 최고온도 위치 십자선 + 오버레이
        r, c = divmod(int(arr.argmax()), COLS)
        self.ch_h.set_ydata([r])
        self.ch_v.set_xdata([c])
        self.hot_txt.set_position((min(c + 0.5, COLS - 5), max(r - 0.8, 0.5)))
        self.hot_txt.set_text(f"{arr[r, c]:.1f}°C")
        self.canvas_th.draw_idle()

        # ── 화재 상태 ──────────────────────────────────────────────────────────
        is_fire = d["fire"]
        self.fire_lbl.configure(
            text="🔥  FIRE !!" if is_fire else "✔  SAFE",
            fg=RED if is_fire else GRN)
        if is_fire and not self.last_fire:
            self._log(
                f"[FIRE] {d['ts']}  Max={d['max_temp']:.1f}°C  Gas={d['mq']}",
                "fire")
        self.last_fire = is_fire

        # ── 수치 패널 ──────────────────────────────────────────────────────────
        self.metrics["max_temp"].configure(
            text=f"{d['max_temp']:.1f} °C",
            fg=RED if d["max_temp"] >= 75 else WHT)
        self.metrics["hot_pixels"].configure(
            text=f"{d['hot_pixels']} px",
            fg=ORG if d["hot_pixels"] >= 4 else WHT)
        self.metrics["gas_adc"].configure(
            text=str(d["mq"]),
            fg=RED if d["mq"] >= 1200 else ORG if d["mq"] >= 300 else WHT)

        # ── 수신 데이터 바 ─────────────────────────────────────────────────────
        self.dlbl["tick"].configure(text=str(d["tick_ms"]))
        self.dlbl["mq"].configure(
            text=str(d["mq"]) if d["mq"] else
                 f'Ta={d.get("ambient", 0):.1f}°C')
        self.dlbl["temp"].configure(text=f"{d['max_temp']:.1f}°C",
                                     fg=RED if d["max_temp"] >= 75 else CYN)
        self.dlbl["hot"].configure(text=str(d["hot_pixels"]))
        self.dlbl["fire"].configure(
            text="FIRE" if d["fire"] else "SAFE",
            fg=RED if d["fire"] else GRN)
        self.dlbl["frames"].configure(
            text=f"{self.frame_count}"
                 + (f"  {d['fps_bin']:.1f}fps" if "fps_bin" in d else ""))

        # ── 시계열 그래프 ─────────────────────────────────────────────────────
        t = d["tick_ms"] / 1000.0
        self.ht.append(t)
        self.htemp.append(d["max_temp"])
        self.hmq.append(d["mq"])
        for lst in (self.ht, self.htemp, self.hmq):
            if len(lst) > MAX_HIST:
                del lst[:-MAX_HIST]

        self.ln_temp.set_data(self.ht, self.htemp)
        self.ax_temp.set_xlim(self.ht[0], max(self.ht[-1], self.ht[0] + 1))
        self.ax_temp.set_ylim(max(0, min(self.htemp) - 5),
                               max(100, max(self.htemp) + 5))
        self.ln_mq.set_data(self.ht, self.hmq)
        self.ax_mq.set_xlim(self.ht[0], max(self.ht[-1], self.ht[0] + 1))
        self.ax_mq.set_ylim(0, max(1500, max(self.hmq) + 100))
        self.canvas_g.draw_idle()

        # ── CSV 저장 ──────────────────────────────────────────────────────────
        if self.csv_writer:
            ambient = d.get("ambient", "")
            row = [d["ts"], d["tick_ms"], d["mq"],
                   f"{d['max_temp']:.2f}", d["hot_pixels"], int(d["fire"]),
                   f"{ambient:.2f}" if ambient != "" else ""]
            row += [f"{v:.2f}" for v in d["pixels"]]
            try:
                self.csv_writer.writerow(row)
                self.csv_file.flush()
            except Exception as e:
                self._log(f"[CSV error] {e}", "warn")

    # =========================================================================
    # 유틸
    # =========================================================================
    def _lb(self, parent, text, size=9, bold=False, color=None, **kw):
        return tk.Label(parent, text=text,
                        bg=parent.cget("bg"),
                        fg=color or WHT,
                        font=("Helvetica", size, "bold" if bold else "normal"),
                        **kw)

    def _btn(self, parent, text, cmd, bg=PNL, fg=WHT, size=9):
        return tk.Button(parent, text=text, command=cmd,
                         bg=bg, fg=fg, activebackground=ACC,
                         activeforeground=WHT, relief="flat",
                         font=("Helvetica", size, "bold"),
                         padx=8, pady=3, cursor="hand2")

    def _vsep(self, parent):
        tk.Frame(parent, bg="#444444", width=1).pack(
            side=tk.LEFT, fill=tk.Y, padx=8, pady=4)

    def _log(self, msg: str, tag: str = ""):
        self.log.configure(state=tk.NORMAL)
        ts = datetime.now().strftime("%H:%M:%S")
        self.log.insert(tk.END, f"[{ts}] {msg}\n", tag or ())
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def _clear_log(self):
        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.configure(state=tk.DISABLED)

    def on_close(self):
        self._disconnect()
        self.root.destroy()


# ─── 진입점 ──────────────────────────────────────────────────────────────────
def main():
    print("KJH_EV4 열화상 모니터 시작 중...", flush=True)
    root = tk.Tk()
    app  = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)

    # macOS: 창을 맨 앞으로 강제 표시
    root.lift()
    root.attributes("-topmost", True)
    root.after(200, lambda: root.attributes("-topmost", False))
    root.focus_force()

    print("창 표시 완료 — ESP32 USB 연결 후 포트를 선택하세요.", flush=True)
    root.mainloop()
    print("종료.", flush=True)


if __name__ == "__main__":
    main()
