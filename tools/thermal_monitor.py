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
import gc
import glob
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
    from PIL import Image, ImageTk

    _KO = next(
        (f for f in ["Apple SD Gothic Neo", "AppleGothic",
                     "Malgun Gothic", "NanumGothic", "Noto Sans KR"]
         if f in {x.name for x in _fm.fontManager.ttflist}), None
    )
    if _KO:
        matplotlib.rcParams["font.family"] = _KO
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.cm as _cm
    # get_cmap은 3.7+에서 deprecated → colormaps[] 우선 사용
    try:
        _INFERNO = matplotlib.colormaps["inferno"]
    except AttributeError:
        _INFERNO = _cm.get_cmap("inferno")
except ImportError:
    _miss.append("matplotlib  numpy  pillow")
    _INFERNO = None

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
        self.binary_mode  = False
        self.data_q       = queue.Queue(maxsize=5)   # 과도한 버퍼링 방지 (was 30)
        self.csv_file     = None
        self.csv_writer   = None
        self.frame_count  = 0
        self.drop_count   = 0
        self.parse_fail   = 0
        self.last_fire    = False
        self._fps_cnt     = 0
        self._fps_ts      = _time.time()
        self._frame_t0    = _time.time()
        self._pending     = None
        # PhotoImage 재사용 — 크기 바뀔 때만 새로 생성
        self._th_photo    = None
        self._th_w        = 0
        self._th_h        = 0
        self._gc_ts       = _time.time()

        # 열화상 렌더 전용 pre-allocated 버퍼 (프레임마다 ndarray 재할당 방지)
        self._norm_buf = np.zeros((ROWS, COLS), dtype=np.float32)

        # 그래프 히스토리 (deque: maxlen이 자동 트림 — del 슬라이싱 불필요)
        from collections import deque
        self.ht    = deque(maxlen=MAX_HIST)
        self.htemp = deque(maxlen=MAX_HIST)
        self.hmq   = deque(maxlen=MAX_HIST)

        # CSV 저장 경로 / 로테이션
        self.save_dir       = tk.StringVar(value=DEFAULT_DIR)
        self.csv_interval   = tk.StringVar(value="없음")
        self._csv_open_ts   = 0.0
        self._last_csv_ts   = 0.0

        # 이미지 캡처 (RAM에 PIL Image 보관 안 함 — 파일에만 저장)
        self.cap_var      = tk.BooleanVar(value=True)
        self._cap_dir     = ""
        self._last_cap_ts = 0.0
        self._cap_count   = 0   # 저장된 JPEG 개수 카운터만 유지

        # UI 구성 — 순서 중요: BOTTOM 요소를 먼저 pack
        self._build_log()        # pack BOTTOM 1st
        self._build_data_bar()   # pack BOTTOM 2nd
        self._build_toolbar()    # pack TOP
        self._build_main()       # pack FILL/EXPAND last

        self._tick()          # 100ms: labels + CSV + capture
        self._thermal_tick()  # 250ms: 열화상 이미지만 (빠른 FPS)
        self._graph_tick()    # 2000ms: matplotlib 그래프 (무거운 렌더)

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

        # ── 연결 모드 선택 (ESP32 릴레이 전용 — GY직접 모드는 별도 배선 필요) ──
        self.mode_var = tk.StringVar(value="ESP32 릴레이")

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

        # ── CSV 자동저장 + 파일 분할 + 이미지 캡처 ─────────────────────────
        self.csv_var = tk.BooleanVar(value=True)
        tk.Checkbutton(bar, text="CSV 자동저장", variable=self.csv_var,
                       bg=MID, fg=WHT, activebackground=MID,
                       activeforeground=WHT, selectcolor=ACC,
                       font=("Helvetica", 9)).pack(side=tk.LEFT, padx=(0, 3))

        self._lb(bar, "분할:", size=8, color="#888").pack(side=tk.LEFT, padx=(0, 2))
        intvl_menu = tk.OptionMenu(bar, self.csv_interval,
                                   "없음", "1분", "5분", "10분", "30분", "60분")
        intvl_menu.configure(bg=PNL, fg=WHT, activebackground=ACC,
                             font=("Helvetica", 8), relief="flat",
                             highlightthickness=0, width=4)
        intvl_menu["menu"].configure(bg=PNL, fg=WHT)
        intvl_menu.pack(side=tk.LEFT, padx=(0, 6))

        self._vsep(bar)

        tk.Checkbutton(bar, text="📸 이미지 캡처", variable=self.cap_var,
                       bg=MID, fg=WHT, activebackground=MID,
                       activeforeground=WHT, selectcolor=ACC,
                       font=("Helvetica", 9)).pack(side=tk.LEFT, padx=(0, 4))

        self._btn(bar, "🎬 영상 저장", self._save_video,
                  bg="#2c3e50").pack(side=tk.LEFT, padx=(0, 6))

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

    # ── 열화상 패널 (PIL/PhotoImage — matplotlib 불필요, CPU 저부하) ─────────
    def _build_thermal(self, parent):
        self._lb(parent, "MLX90640 열화상  (32 × 24 → 320 × 240)",
                 size=11, color=CYN, bold=True).pack(pady=(0, 4))

        # 열화상 이미지 표시용 Label
        # ※ <Configure> 바인딩 금지 — configure(image=) 호출이 이벤트를 재발화해
        #   무한루프로 ImageTk.PhotoImage가 수십GB까지 누적되는 치명적 메모리 누수 유발
        self.thermal_lbl = tk.Label(parent, bg=BG, cursor="crosshair", anchor="nw")
        self.thermal_lbl.pack(fill=tk.BOTH, expand=True)

        # 오버레이 정보 바 (최고온도 + FPS)
        info_bar = tk.Frame(parent, bg=DRK, pady=3)
        info_bar.pack(fill=tk.X)
        self.th_info = tk.Label(info_bar,
            text="Tmin=-- Tmax=-- Pos=(-,-) FPS=--",
            bg=DRK, fg=CYN, font=("Courier New", 9))
        self.th_info.pack()

        # 초기 placeholder 이미지 생성
        self._draw_placeholder()

    def _draw_placeholder(self):
        """연결 전 placeholder 렌더"""
        arr = np.full((ROWS, COLS), 25.0, dtype=np.float32)
        self._update_thermal_label(arr, 320, 240)

    def _arr_to_pil(self, arr: np.ndarray, width: int, height: int) -> "Image.Image":
        lo = float(arr.min()); hi = float(arr.max())
        if hi - lo < 1.0:
            lo -= 5.0; hi += 5.0
        # _norm_buf: pre-allocated (24×32 float32) — 매 프레임 재할당 제거
        np.clip((arr - lo) / (hi - lo), 0.0, 1.0, out=self._norm_buf)
        norm = self._norm_buf[:, ::-1]   # fliplr as a zero-copy view
        if _INFERNO is not None:
            rgba = _INFERNO(norm)                            # (24,32,4) float64
            rgb  = (rgba[:, :, :3] * 255).astype(np.uint8)
        else:
            r = np.clip(norm * 2.5,       0, 1)
            g = np.clip(norm * 2.5 - 1.0, 0, 1)
            b = np.clip(1.0 - norm * 2.0, 0, 1)
            rgb = (np.stack([r, g, b], axis=-1) * 255).astype(np.uint8)
        w = max(32, min(width,  1920))
        h = max(32, min(height, 1080))
        return Image.fromarray(rgb).resize((w, h), Image.BILINEAR)

    def _update_thermal_label(self, arr: np.ndarray, w: int, h: int):
        img = self._arr_to_pil(arr, w, h)
        iw, ih = img.size
        old = self._th_photo
        self._th_photo = ImageTk.PhotoImage(img)
        self._th_w, self._th_h = iw, ih
        self.thermal_lbl.configure(image=self._th_photo)
        # label이 new_photo를 참조한 뒤 old 즉시 해제 → Tcl 리소스 누수 방지
        if old is not None:
            try:
                old.__del__()
            except Exception:
                pass

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

        # ── 모드 결정 (ESP32 릴레이 고정 또는 자동 감지) ────────────────────
        chosen = self.mode_var.get()
        if chosen == "자동 감지":
            self.ser.timeout = 0.5
            probe = self.ser.read(16)
            self.ser.timeout = 2.0
            non_print = sum(1 for b in probe if b < 32 and b not in (0x0A, 0x0D))
            self.binary_mode = (non_print >= 3)
            self._log(
                f"자동 감지: {'바이너리' if self.binary_mode else 'ESP32 텍스트'} "
                f"[probe={probe.hex()}]", "info")
        else:
            self.binary_mode = False   # ESP32 릴레이

        mode_label = "ESP32 릴레이"
        self.conn_btn.configure(text="■  연결 해제", bg="#7b1c1c")
        self.conn_lbl.configure(text=f"●  {port}", fg=GRN)
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
                    self.ser.write(bytes([0xA5, 0x35, 0x01, 0xDB]))
                except Exception:
                    pass
            self.ser.close()
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = self.csv_writer = None
        # 미처리 큐 비우기 + pending 참조 해제 → data dict 즉시 GC
        while not self.data_q.empty():
            try:
                self.data_q.get_nowait()
            except queue.Empty:
                break
        self._pending = None
        gc.collect()
        self.conn_btn.configure(text="▶  연결", bg="#1a6b3a")
        self.conn_lbl.configure(text="●  연결 끊김", fg="#555555")
        self._log("포트 연결 해제", "warn")

    # =========================================================================
    # 7. CSV 관리
    # =========================================================================
    def _init_csv(self):
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
        d    = self.save_dir.get()
        os.makedirs(d, exist_ok=True)
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        mode = "bin" if self.binary_mode else "esp"
        fp   = os.path.join(d, f"thermal_{mode}_{ts}.csv")
        self.csv_file     = open(fp, "w", newline="", encoding="utf-8")
        self._csv_open_ts = _time.time()
        self._last_csv_ts = 0.0     # 새 파일 시작 시 즉시 첫 행 기록
        hdr = ["timestamp", "tick_ms", "mq_raw", "max_temp_c",
               "hot_pixels", "fire_triggered", "ambient_c"]
        hdr += [f"t{i:03d}" for i in range(NPIX)]
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(hdr)
        self._log(f"CSV 시작: {fp}", "info")

        # 이미지 캡처 디렉토리 (CSV와 같은 폴더 아래 frames/)
        self._cap_dir = os.path.join(d, f"frames_{ts}")
        os.makedirs(self._cap_dir, exist_ok=True)
        self._last_cap_ts = 0.0

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
                pixels = np.frombuffer(frame, dtype=np.int16,
                                       count=NPIX, offset=4).astype(np.float32) / 100.0

                # ── 주변 온도 (bytes 1540‥1541, int16 LE) ────────────────
                ta = int.from_bytes(frame[1540:1542], "little", signed=True) / 100.0

                max_t  = float(pixels.max())
                hot_px = int((pixels >= 45.0).sum())
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
            raw    = bytes.fromhex(hex_blob[:NPIX * 4])
            pixels = np.frombuffer(raw, dtype=">i2").astype(np.float32) / 100.0
            return {
                "tick_ms":    int(p[1]),
                "mq":         int(p[2]),
                "max_temp":   int(p[3]) / 100.0,
                "hot_pixels": int(p[4]),
                "fire":       bool(int(p[5])),
                "pixels":     pixels,  # ndarray (768,) float32 — Python list 불필요
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
            pixels = np.array([float(x) for x in p[6: 6 + NPIX]], dtype=np.float32)
            return {
                "tick_ms":    int(p[1]),
                "mq":         int(p[2]),
                "max_temp":   float(p[3]),
                "hot_pixels": int(p[4]),
                "fire":       bool(int(p[5])),
                "pixels":     pixels,  # ndarray (768,) float32
                "ts":         datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            }
        except Exception:
            return None

    # =========================================================================
    # 9. 렌더링 루프 (3-tier 분리)
    #    _tick()        100ms  — labels + CSV(1/sec) + capture(1/sec) + GC(5/sec)
    #    _thermal_tick() 250ms  — 열화상 PIL 이미지만 (≈4 FPS)
    #    _graph_tick()  2000ms — matplotlib 그래프 (무거운 렌더)
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
                self._update_labels(data)
                self._save_csv(data)        # 내부에서 1초 rate-limit
                self._capture_frame(data)   # 내부에서 1초 rate-limit
                self._pending = data        # thermal_tick이 열화상 렌더

            now_ts = _time.time()
            now_dt = now_ts

            if now_dt - self._fps_ts >= 1.0:
                fps = self._fps_cnt / max(now_dt - self._fps_ts, 0.001)
                self.metrics["fps"].configure(text=f"{fps:.1f} fps")
                self._fps_cnt = 0
                self._fps_ts  = now_dt

            self.frame_lbl.configure(
                text=f"프레임: {self.frame_count}  |  캡처: {self._cap_count}")

            if now_ts - self._gc_ts >= 3.0:   # 3초마다 GC (was 5s)
                gc.collect()
                self._gc_ts = now_ts
                self._trim_log()
        except Exception as e:
            try:
                self._log(f"[tick error] {e}", "warn")
            except Exception:
                pass
        finally:
            self.root.after(100, self._tick)

    def _thermal_tick(self):
        """열화상 이미지 전용 루프 (250ms ≈ 4 FPS)"""
        try:
            d = self._pending
            if d is not None:
                # pixels가 ndarray(768,)이면 reshape는 zero-copy view
                arr = d["pixels"].reshape(ROWS, COLS)
                hot_idx = int(arr.argmax())
                r_max, c_max = divmod(hot_idx, COLS)
                try:
                    w = self.thermal_lbl.winfo_width()
                    h = self.thermal_lbl.winfo_height()
                    if w < 32 or h < 32:
                        w, h = 320, 240
                    self._update_thermal_label(arr, w, h)
                except Exception as e:
                    self._log(f"[thermal render] {e}", "warn")
                fps_str = f" FPS={d.get('fps_bin',0):.1f}" if "fps_bin" in d else ""
                self.th_info.configure(
                    text=f"Tmin={arr.min():.1f}°C  Tmax={arr.max():.1f}°C"
                         f"  HotPos=({c_max},{r_max}){fps_str}")
        except Exception:
            pass
        finally:
            self.root.after(250, self._thermal_tick)

    def _graph_tick(self):
        """matplotlib 그래프 전용 루프 (2000ms)"""
        try:
            d = self._pending
            if d is not None:
                # deque(maxlen=MAX_HIST) — 자동 트림, 수동 del 불필요
                self.ht.append(d["tick_ms"] / 1000.0)
                self.htemp.append(d["max_temp"])
                self.hmq.append(d["mq"])
                ht = list(self.ht); htemp = list(self.htemp); hmq = list(self.hmq)
                self.ln_temp.set_data(ht, htemp)
                self.ax_temp.set_xlim(ht[0], max(ht[-1], ht[0]+1))
                self.ax_temp.set_ylim(max(0, min(htemp)-5), max(100, max(htemp)+5))
                self.ln_mq.set_data(ht, hmq)
                self.ax_mq.set_xlim(ht[0], max(ht[-1], ht[0]+1))
                self.ax_mq.set_ylim(0, max(1500, max(hmq)+100))
                self.canvas_g.draw_idle()
        except Exception as e:
            try:
                self._log(f"[graph error] {e}", "warn")
            except Exception:
                pass
        finally:
            self.root.after(2000, self._graph_tick)

    def _update_labels(self, d: dict):
        """빠른 tkinter 위젯 갱신 (matplotlib 렌더 없음)"""
        is_fire = d["fire"]
        self.fire_lbl.configure(
            text="🔥  FIRE !!" if is_fire else "✔  SAFE",
            fg=RED if is_fire else GRN)
        if is_fire and not self.last_fire:
            self._log(
                f"[FIRE] {d['ts']}  Max={d['max_temp']:.1f}°C  Gas={d['mq']}",
                "fire")
        self.last_fire = is_fire

        self.metrics["max_temp"].configure(
            text=f"{d['max_temp']:.1f} °C",
            fg=RED if d["max_temp"] >= 75 else WHT)
        self.metrics["hot_pixels"].configure(
            text=f"{d['hot_pixels']} px",
            fg=ORG if d["hot_pixels"] >= 4 else WHT)
        self.metrics["gas_adc"].configure(
            text=str(d["mq"]),
            fg=RED if d["mq"] >= 1200 else ORG if d["mq"] >= 300 else WHT)

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

    # _render_canvas 역할은 _thermal_tick / _graph_tick으로 분리됨 (레거시 호환용)

    def _save_csv(self, d: dict):
        """CSV 한 행 저장 + 인터벌 파일 로테이션"""
        if not self.csv_var.get():
            return
        # ── 1초 rate-limit: 정확히 1초에 1행 ──────────────────────────────
        now = _time.time()
        if now - self._last_csv_ts < 1.0:
            return
        self._last_csv_ts = now

        # ── 인터벌 파일 로테이션 ────────────────────────────────────────────
        interval_str = self.csv_interval.get()
        interval_sec = {"1분": 60, "5분": 300, "10분": 600,
                        "30분": 1800, "60분": 3600}.get(interval_str, 0)
        if interval_sec and self.csv_file:
            if now - self._csv_open_ts >= interval_sec:
                self._log(f"CSV 파일 분리 ({interval_str} 경과)", "info")
                self._init_csv()

        if not self.csv_writer:
            return
        ambient = d.get("ambient", "")
        row = [d["ts"], d["tick_ms"], d["mq"],
               f"{d['max_temp']:.2f}", d["hot_pixels"], int(d["fire"]),
               f"{ambient:.2f}" if ambient != "" else ""]
        # ndarray와 list 모두 동일하게 처리 — iteration이 ndarray에서도 동작
        row += [f"{v:.2f}" for v in d["pixels"]]
        try:
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        except Exception as e:
            self._log(f"[CSV error] {e}", "warn")

    def _capture_frame(self, d: dict):
        """1초마다 열화상 JPEG를 디스크에만 저장 (RAM에 PIL Image 누적 없음)"""
        if not self.cap_var.get() or not self._cap_dir:
            return
        now = _time.time()
        if now - self._last_cap_ts < 1.0:
            return
        self._last_cap_ts = now
        try:
            arr  = d["pixels"].reshape(ROWS, COLS)   # ndarray → zero-copy view
            img  = self._arr_to_pil(arr, 320, 240)
            ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = os.path.join(self._cap_dir, f"frame_{ts}.jpg")
            img.save(fname, "JPEG", quality=85)
            # img 즉시 버리고 카운터만 올림 → PIL Image가 RAM에 남지 않음
            self._cap_count += 1
        except Exception as e:
            self._log(f"[capture error] {e}", "warn")

    def _save_video(self):
        """캡처 JPEG 파일을 디스크에서 읽어 MP4 생성 (RAM 버퍼 없음)"""
        frame_dir = self._cap_dir
        if not frame_dir or not os.path.isdir(frame_dir):
            messagebox.showinfo("캡처 없음",
                "캡처 폴더가 없습니다.\n연결 후 '📸 이미지 캡처'를 활성화하세요.")
            return
        frames = sorted(glob.glob(os.path.join(frame_dir, "frame_*.jpg")))
        if not frames:
            messagebox.showinfo("캡처 없음", f"저장된 프레임이 없습니다.\n{frame_dir}")
            return
        d     = self.save_dir.get()
        ts    = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(d, exist_ok=True)
        vpath = os.path.join(d, f"thermal_timelapse_{ts}.mp4")
        n     = len(frames)
        try:
            import imageio
            # 프레임을 한 장씩 디스크에서 읽어 인코딩 → RAM에 전체 버퍼 없음
            with imageio.get_writer(vpath, fps=1, quality=7) as writer:
                for fp in frames:
                    writer.append_data(np.array(Image.open(fp)))
            self._log(f"영상 저장 완료: {vpath}  ({n}프레임)", "info")
            messagebox.showinfo("영상 저장", f"저장 완료 ({n}프레임):\n{vpath}")
        except ImportError:
            try:
                result = subprocess.run(
                    ["ffmpeg", "-y", "-framerate", "1",
                     "-pattern_type", "glob", "-i",
                     os.path.join(frame_dir, "frame_*.jpg"),
                     "-c:v", "libx264", "-pix_fmt", "yuv420p", vpath],
                    capture_output=True, text=True, timeout=120)
                if result.returncode == 0:
                    self._log(f"영상 저장(ffmpeg): {vpath}  ({n}프레임)", "info")
                    messagebox.showinfo("영상 저장", f"저장 완료 ({n}프레임):\n{vpath}")
                else:
                    raise RuntimeError(result.stderr[-300:])
            except Exception as e:
                self._log(f"[video error] {e}", "warn")
                messagebox.showwarning("영상 저장 실패",
                    "imageio 패키지 필요:\n  pip install imageio[ffmpeg]\n\n"
                    f"JPEG 프레임 위치: {frame_dir}")

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

    def _trim_log(self, max_lines: int = 200):
        """로그 패널 200줄 초과 시 앞부분 삭제"""
        try:
            lines = int(self.log.index("end-1c").split(".")[0])
            if lines > max_lines:
                self.log.configure(state=tk.NORMAL)
                self.log.delete("1.0", f"{lines - max_lines}.0")
                self.log.configure(state=tk.DISABLED)
        except Exception:
            pass

    def _clear_log(self):
        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.configure(state=tk.DISABLED)

    def on_close(self):
        try:
            self._disconnect()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass


# ─── 진입점 ──────────────────────────────────────────────────────────────────
def main():
    import argparse
    ap = argparse.ArgumentParser(description="KJH_EV4 열화상 모니터")
    ap.add_argument("--port", default="",
                    help="시리얼 포트 자동 연결 (예: /dev/cu.usbmodem...)")
    ap.add_argument("--mode", default="ESP32 릴레이",
                    choices=["자동 감지", "ESP32 릴레이"],
                    help="연결 모드 (기본: ESP32 릴레이)")
    args = ap.parse_args()

    print("KJH_EV4 열화상 모니터 시작 중...", flush=True)
    root = tk.Tk()
    app  = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)

    # macOS: 창을 맨 앞으로 강제 표시
    root.lift()
    root.attributes("-topmost", True)
    root.after(200, lambda: root.attributes("-topmost", False))
    root.focus_force()

    # 명령줄 인수로 자동 연결
    if args.port:
        app.port_var.set(args.port)
        app.mode_var.set(args.mode)
        root.after(500, app._connect)   # 창이 완전히 뜬 뒤 연결
        print(f"자동 연결: {args.port}  모드={args.mode}", flush=True)
    else:
        print("창 표시 완료 — 포트를 선택하고 ▶ 연결 버튼을 클릭하세요.", flush=True)

    root.mainloop()
    print("종료.", flush=True)


if __name__ == "__main__":
    main()
