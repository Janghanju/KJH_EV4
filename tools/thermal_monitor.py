#!/usr/bin/env python3
"""
KJH_EV4 실시간 열화상 모니터 GUI
=====================================
ESP32-S3 시리얼 $DATA 라인을 수신하여:
  - MLX90640 32x24 열화상 실시간 시각화
  - 온도 / 가스 센서 시계열 그래프
  - 화재 상태 패널
  - CSV 자동 저장

설치:
    python -m pip install pyserial matplotlib numpy
실행:
    python thermal_monitor.py
"""

import csv
import os
import queue
import sys
import threading
from datetime import datetime

import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText
from tkinter import ttk

# ─── 의존성 체크 ────────────────────────────────────────────────────────────
_missing = []
try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    import matplotlib.gridspec as gridspec
    import matplotlib.font_manager as fm
    import numpy as np

    # macOS/Windows/Linux 한글 폰트 자동 선택
    _ko_candidates = [
        "Apple SD Gothic Neo", "AppleGothic",          # macOS
        "Malgun Gothic", "Gulim",                       # Windows
        "NanumGothic", "NanumBarunGothic", "Noto Sans KR",  # Linux / 설치된 경우
    ]
    _available = {f.name for f in fm.fontManager.ttflist}
    _ko_font = next((f for f in _ko_candidates if f in _available), None)
    if _ko_font:
        matplotlib.rcParams["font.family"] = _ko_font
    matplotlib.rcParams["axes.unicode_minus"] = False

except ImportError:
    _missing.append("matplotlib  numpy")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    _missing.append("pyserial")

if _missing:
    root = tk.Tk()
    root.withdraw()
    messagebox.showerror(
        "패키지 누락",
        "필수 패키지가 설치되지 않았습니다:\n\n"
        + "\n".join(f"  • {m}" for m in _missing)
        + "\n\n터미널에서 다음 명령어를 실행하세요:\n\n"
        + "  python -m pip install pyserial matplotlib numpy",
    )
    sys.exit(1)

# ─── 상수 ────────────────────────────────────────────────────────────────────
BAUD_RATE     = 115200
FRAME_ROWS    = 24
FRAME_COLS    = 32
FRAME_PIXELS  = FRAME_ROWS * FRAME_COLS
MAX_HISTORY   = 300
LOG_DIR       = "logs"

# 다크 테마 팔레트
BG_DARK   = "#1a1a2e"
BG_MID    = "#16213e"
BG_PANEL  = "#0f3460"
FG_WHITE  = "#e0e0e0"
FG_GREEN  = "#2ecc71"
FG_RED    = "#e74c3c"
FG_ORANGE = "#f39c12"
FG_CYAN   = "#4ecdc4"
ACCENT    = "#533483"


# ─── 메인 앱 클래스 ──────────────────────────────────────────────────────────
class ThermalMonitorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("KJH_EV4 — 열화상 실시간 모니터  v1.1")
        self.root.configure(bg=BG_DARK)
        self.root.geometry("1280x840")
        self.root.minsize(1024, 720)

        # 상태
        self.ser           = None
        self.running       = False
        self.data_q        = queue.Queue(maxsize=20)
        self.csv_file      = None
        self.csv_writer    = None
        self.frame_count   = 0
        self.drop_count    = 0
        self.last_fire     = False

        # 히스토리
        self.hist_t    = []   # tick(s)
        self.hist_temp = []
        self.hist_mq   = []

        # FPS 카운터 (연결 전에도 _tick이 호출되므로 여기서 초기화)
        self._fps_count = 0
        self._fps_ts    = datetime.now().timestamp()

        self._build_styles()
        self._build_toolbar()
        self._build_main()
        self._build_log()
        self._tick()

    # ── 스타일 ───────────────────────────────────────────────────────────────
    def _build_styles(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure("D.TFrame",  background=BG_DARK)
        s.configure("D.TLabel",  background=BG_DARK,  foreground=FG_WHITE, font=("Helvetica", 9))
        s.configure("D.TButton", background=BG_PANEL, foreground=FG_WHITE,
                    font=("Helvetica", 9, "bold"), relief="flat", padding=4)
        s.map("D.TButton", background=[("active", ACCENT)])
        s.configure("G.TButton", background="#1a6b3a", foreground="white",
                    font=("Helvetica", 9, "bold"), relief="flat", padding=4)
        s.map("G.TButton", background=[("active", "#27ae60")])
        s.configure("R.TButton", background="#7b1c1c", foreground="white",
                    font=("Helvetica", 9, "bold"), relief="flat", padding=4)
        s.map("R.TButton", background=[("active", FG_RED)])
        s.configure("D.TCombobox", fieldbackground=BG_MID, background=BG_MID,
                    foreground=FG_WHITE, selectbackground=ACCENT)

    # ── 툴바 ─────────────────────────────────────────────────────────────────
    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg=BG_MID, pady=5)
        bar.pack(side=tk.TOP, fill=tk.X)

        # 포트
        tk.Label(bar, text=" 포트:", bg=BG_MID, fg=FG_WHITE,
                 font=("Helvetica", 9)).pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(bar, textvariable=self.port_var,
                                      width=24, state="readonly",
                                      style="D.TCombobox")
        self.port_cb.pack(side=tk.LEFT, padx=(2, 4))
        self._refresh_ports()

        ttk.Button(bar, text="⟳", style="D.TButton", width=2,
                   command=self._refresh_ports).pack(side=tk.LEFT, padx=(0, 10))

        # 연결 버튼
        self.conn_btn = ttk.Button(bar, text="▶  연결", style="G.TButton",
                                    width=10, command=self._toggle_connect)
        self.conn_btn.pack(side=tk.LEFT, padx=(0, 8))

        tk.Frame(bar, bg="#444", width=1).pack(side=tk.LEFT, fill=tk.Y, pady=3, padx=6)

        # CSV 체크
        self.csv_var = tk.BooleanVar(value=True)
        tk.Checkbutton(bar, text="CSV 저장", variable=self.csv_var,
                       bg=BG_MID, fg=FG_WHITE, activebackground=BG_MID,
                       activeforeground=FG_WHITE, selectcolor=ACCENT,
                       font=("Helvetica", 9)).pack(side=tk.LEFT, padx=(0, 10))

        # 상태 표시
        self.conn_lbl = tk.Label(bar, text="●  연결 끊김", bg=BG_MID,
                                  fg="#666666", font=("Helvetica", 9))
        self.conn_lbl.pack(side=tk.LEFT, padx=6)

        # 프레임 카운터
        self.frame_lbl = tk.Label(bar, text="프레임: 0 | 드롭: 0",
                                   bg=BG_MID, fg="#888888", font=("Helvetica", 8))
        self.frame_lbl.pack(side=tk.RIGHT, padx=10)

    # ── 메인 레이아웃 ─────────────────────────────────────────────────────────
    def _build_main(self):
        container = tk.Frame(self.root, bg=BG_DARK)
        container.pack(fill=tk.BOTH, expand=True, padx=8, pady=(6, 0))

        # 왼쪽: 열화상
        left = tk.Frame(container, bg=BG_DARK)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 오른쪽: 상태 + 그래프
        right = tk.Frame(container, bg=BG_DARK, width=400)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(8, 0))
        right.pack_propagate(False)

        self._build_thermal(left)
        self._build_status(right)
        self._build_graphs(right)

    # ── 열화상 패널 ───────────────────────────────────────────────────────────
    def _build_thermal(self, parent):
        tk.Label(parent, text="MLX90640 열화상  (32 × 24 pixels)",
                 bg=BG_DARK, fg=FG_CYAN,
                 font=("Helvetica", 10, "bold")).pack(pady=(0, 2))

        self.fig_th = Figure(figsize=(5.8, 4.8), facecolor=BG_DARK)
        self.ax_th  = self.fig_th.add_subplot(111)
        self.ax_th.set_facecolor(BG_MID)

        init = np.full((FRAME_ROWS, FRAME_COLS), 25.0)
        self.im = self.ax_th.imshow(init, cmap="inferno",
                                     vmin=20, vmax=80,
                                     aspect="auto", interpolation="bilinear")

        cb = self.fig_th.colorbar(self.im, ax=self.ax_th, fraction=0.04, pad=0.02)
        cb.set_label("°C", color=FG_WHITE, fontsize=9)
        cb.ax.yaxis.set_tick_params(color=FG_WHITE, labelsize=7)
        for lbl in cb.ax.yaxis.get_ticklabels():
            lbl.set_color(FG_WHITE)

        self.ax_th.set_xlabel("Column", color=FG_WHITE, fontsize=8)
        self.ax_th.set_ylabel("Row",    color=FG_WHITE, fontsize=8)
        self.ax_th.tick_params(colors=FG_WHITE, labelsize=7)
        for sp in self.ax_th.spines.values():
            sp.set_edgecolor("#444")

        # 최고온도 위치 십자선
        self.ch_h = self.ax_th.axhline(0, color="cyan", lw=0.9, alpha=0.8)
        self.ch_v = self.ax_th.axvline(0, color="cyan", lw=0.9, alpha=0.8)

        # 최고온도 텍스트 오버레이
        self.hot_text = self.ax_th.text(
            1, 1, "", color="cyan", fontsize=8,
            ha="left", va="top",
            bbox=dict(boxstyle="round,pad=0.2", fc=BG_DARK, alpha=0.7)
        )

        self.canvas_th = FigureCanvasTkAgg(self.fig_th, master=parent)
        self.canvas_th.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.fig_th.tight_layout(pad=1.2)

    # ── 상태 패널 ─────────────────────────────────────────────────────────────
    def _build_status(self, parent):
        pnl = tk.Frame(parent, bg=BG_PANEL)
        pnl.pack(fill=tk.X, pady=(0, 6))

        self.fire_lbl = tk.Label(pnl, text="✔  SAFE",
                                  bg=BG_PANEL, fg=FG_GREEN,
                                  font=("Helvetica", 26, "bold"))
        self.fire_lbl.pack(pady=(12, 6))

        grid = tk.Frame(pnl, bg=BG_PANEL)
        grid.pack(fill=tk.X, padx=16, pady=(0, 12))

        self.m = {}
        rows = [
            ("max_temp",   "최고 온도",   "--.- °C"),
            ("hot_pixels", "Hot Pixels",  "--  px"),
            ("gas_adc",    "Gas ADC",     "----"),
            ("fps",        "갱신 속도",   "-- fps"),
        ]
        for i, (key, label, default) in enumerate(rows):
            tk.Label(grid, text=label + ":", bg=BG_PANEL, fg="#aaaaaa",
                     font=("Helvetica", 9), anchor="w", width=12).grid(
                row=i, column=0, sticky="w", pady=2)
            v = tk.Label(grid, text=default, bg=BG_PANEL, fg=FG_WHITE,
                         font=("Helvetica", 11, "bold"), anchor="w")
            v.grid(row=i, column=1, sticky="w", pady=2)
            self.m[key] = v

    # ── 그래프 패널 ───────────────────────────────────────────────────────────
    def _build_graphs(self, parent):
        self.fig_g = Figure(figsize=(3.8, 3.6), facecolor=BG_DARK)
        gs = gridspec.GridSpec(2, 1, figure=self.fig_g,
                               hspace=0.5, top=0.92, bottom=0.10,
                               left=0.14, right=0.96)

        # 온도 그래프
        self.ax_temp = self.fig_g.add_subplot(gs[0])
        self.ax_temp.set_facecolor(BG_MID)
        self.ax_temp.set_title("최고 온도 (°C)", color=FG_WHITE, fontsize=8, pad=3)
        self.ax_temp.tick_params(colors=FG_WHITE, labelsize=7)
        self.ax_temp.set_ylabel("°C", color=FG_WHITE, fontsize=7)
        for sp in self.ax_temp.spines.values():
            sp.set_edgecolor("#444")
        self.ax_temp.axhline(75, color="red", ls="--", lw=0.8, alpha=0.6,
                              label="화재기준 75°C")
        self.ax_temp.legend(fontsize=6, loc="upper left",
                             facecolor=BG_MID, labelcolor=FG_WHITE)
        (self.ln_temp,) = self.ax_temp.plot([], [], color="#ff6b6b", lw=1.2)

        # 가스 그래프
        self.ax_mq = self.fig_g.add_subplot(gs[1])
        self.ax_mq.set_facecolor(BG_MID)
        self.ax_mq.set_title("MQ 가스 센서 (ADC)", color=FG_WHITE, fontsize=8, pad=3)
        self.ax_mq.tick_params(colors=FG_WHITE, labelsize=7)
        self.ax_mq.set_ylabel("ADC", color=FG_WHITE, fontsize=7)
        for sp in self.ax_mq.spines.values():
            sp.set_edgecolor("#444")
        self.ax_mq.axhline(300,  color=FG_ORANGE, ls="--", lw=0.8, alpha=0.6, label="경계 300")
        self.ax_mq.axhline(1200, color="red",      ls="--", lw=0.8, alpha=0.6, label="위험 1200")
        self.ax_mq.legend(fontsize=6, loc="upper left",
                           facecolor=BG_MID, labelcolor=FG_WHITE)
        (self.ln_mq,) = self.ax_mq.plot([], [], color=FG_CYAN, lw=1.2)

        self.canvas_g = FigureCanvasTkAgg(self.fig_g, master=parent)
        self.canvas_g.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ── 로그 패널 ─────────────────────────────────────────────────────────────
    def _build_log(self):
        frm = tk.Frame(self.root, bg=BG_DARK)
        frm.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(2, 6))

        tk.Label(frm, text="시리얼 로그", bg=BG_DARK, fg="#666",
                 font=("Helvetica", 8)).pack(anchor="w")

        self.log = ScrolledText(frm, height=5, bg="#0d0d1a", fg="#aaaaaa",
                                 font=("Courier New", 8),
                                 state=tk.DISABLED, wrap=tk.WORD,
                                 insertbackground="white")
        self.log.pack(fill=tk.X)
        self.log.tag_config("fire", foreground=FG_RED)
        self.log.tag_config("warn", foreground=FG_ORANGE)
        self.log.tag_config("info", foreground=FG_CYAN)

    # ── 포트 새로고침 ─────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_cb.current(0)
        if not ports:
            self._log("감지된 시리얼 포트 없음. USB 연결 확인 후 ⟳ 클릭.", "warn")

    # ── 연결 토글 ─────────────────────────────────────────────────────────────
    def _toggle_connect(self):
        if self.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("포트 없음", "연결할 포트를 선택하세요.")
            return
        try:
            self.ser = serial.Serial(
                port=port, baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=2.0,
            )
        except serial.SerialException as e:
            messagebox.showerror("연결 실패", str(e))
            return

        self.running = True
        self.frame_count = 0
        self.drop_count  = 0
        self._fps_count  = 0
        self._fps_ts     = datetime.now().timestamp()

        if self.csv_var.get():
            self._init_csv()

        threading.Thread(target=self._reader, daemon=True).start()

        self.conn_btn.configure(text="■  연결 해제", style="R.TButton")
        self.conn_lbl.configure(text=f"●  연결됨: {port}", fg=FG_GREEN)
        self._log(f"포트 연결: {port}  ({BAUD_RATE} baud)", "info")

    def _disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.csv_file:
            self.csv_file.close()
            self.csv_file   = None
            self.csv_writer = None
        self.conn_btn.configure(text="▶  연결", style="G.TButton")
        self.conn_lbl.configure(text="●  연결 끊김", fg="#666666")
        self._log("포트 연결 해제.", "warn")

    # ── CSV 초기화 ────────────────────────────────────────────────────────────
    def _init_csv(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(LOG_DIR, f"thermal_{ts}.csv")
        self.csv_file   = open(path, "w", newline="", encoding="utf-8")
        hdr  = ["timestamp", "tick_ms", "mq_raw", "max_temp_c",
                "hot_pixels", "fire_triggered"]
        hdr += [f"t{i:03d}" for i in range(FRAME_PIXELS)]
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(hdr)
        self._log(f"CSV 저장 시작: {path}", "info")

    # ── 시리얼 수신 스레드 ────────────────────────────────────────────────────
    def _reader(self):
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
                elif line and not line.startswith("$"):
                    self.root.after(0, self._log, line[:120])
            except serial.SerialException:
                self.running = False
                self.root.after(0, self._disconnect)
                break
            except Exception:
                pass

    @staticmethod
    def _parse(line):
        if not line.startswith("$DATA,"):
            return None
        p = line.split(",")
        if len(p) < 6 + FRAME_PIXELS:
            return None
        try:
            return {
                "tick_ms":    int(p[1]),
                "mq":         int(p[2]),
                "max_temp":   float(p[3]),
                "hot_pixels": int(p[4]),
                "fire":       bool(int(p[5])),
                "pixels":     [float(x) for x in p[6: 6 + FRAME_PIXELS]],
                "ts":         datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            }
        except (ValueError, IndexError):
            return None

    # ── 200ms 갱신 루프 ───────────────────────────────────────────────────────
    def _tick(self):
        try:
            data = None
            while not self.data_q.empty():
                try:
                    data = self.data_q.get_nowait()
                    self.frame_count += 1
                    self._fps_count  += 1
                except queue.Empty:
                    break

            if data:
                self._render(data)

            # FPS 계산 (1초마다)
            now = datetime.now().timestamp()
            if now - self._fps_ts >= 1.0:
                fps = self._fps_count / max(now - self._fps_ts, 0.001)
                self.m["fps"].configure(text=f"{fps:.1f} fps")
                self._fps_count = 0
                self._fps_ts    = now

            self.frame_lbl.configure(
                text=f"프레임: {self.frame_count} | 드롭: {self.drop_count}"
            )
        except Exception:
            pass
        finally:
            self.root.after(200, self._tick)

    # ── 렌더링 ────────────────────────────────────────────────────────────────
    def _render(self, data):
        arr = np.array(data["pixels"]).reshape(FRAME_ROWS, FRAME_COLS)

        # 열화상 업데이트
        self.im.set_data(arr)
        lo = max(20.0, float(arr.min()) - 2)
        hi = min(150.0, float(arr.max()) + 2)
        if hi > lo + 1:
            self.im.set_clim(lo, hi)

        # 최고온도 위치 십자선 + 텍스트
        idx      = int(arr.argmax())
        row, col = divmod(idx, FRAME_COLS)
        self.ch_h.set_ydata([row])
        self.ch_v.set_xdata([col])
        self.hot_text.set_position((col + 0.5, row - 0.5))
        self.hot_text.set_text(f"{arr[row, col]:.1f}°C")
        self.canvas_th.draw_idle()

        # 상태 패널
        is_fire = data["fire"]
        if is_fire:
            self.fire_lbl.configure(text="🔥  FIRE !!", fg=FG_RED)
            if not self.last_fire:
                self._log(
                    f"[FIRE ALERT] {data['ts']}  "
                    f"Max={data['max_temp']:.1f}°C  Gas={data['mq']}",
                    "fire",
                )
        else:
            self.fire_lbl.configure(text="✔  SAFE", fg=FG_GREEN)
        self.last_fire = is_fire

        self.m["max_temp"].configure(
            text=f"{data['max_temp']:.1f} °C",
            fg=FG_RED if data["max_temp"] >= 75 else FG_WHITE,
        )
        self.m["hot_pixels"].configure(
            text=f"{data['hot_pixels']}  px",
            fg=FG_ORANGE if data["hot_pixels"] >= 4 else FG_WHITE,
        )
        self.m["gas_adc"].configure(
            text=str(data["mq"]),
            fg=FG_RED if data["mq"] >= 1200
            else FG_ORANGE if data["mq"] >= 300
            else FG_WHITE,
        )

        # 시계열 업데이트
        t_s = data["tick_ms"] / 1000.0
        self.hist_t.append(t_s)
        self.hist_temp.append(data["max_temp"])
        self.hist_mq.append(data["mq"])
        for lst in (self.hist_t, self.hist_temp, self.hist_mq):
            if len(lst) > MAX_HISTORY:
                del lst[:-MAX_HISTORY]

        t = self.hist_t
        self.ln_temp.set_data(t, self.hist_temp)
        self.ax_temp.set_xlim(t[0], max(t[-1], t[0] + 1))
        self.ax_temp.set_ylim(
            max(0, min(self.hist_temp) - 5),
            max(100, max(self.hist_temp) + 5),
        )

        self.ln_mq.set_data(t, self.hist_mq)
        self.ax_mq.set_xlim(t[0], max(t[-1], t[0] + 1))
        self.ax_mq.set_ylim(0, max(1500, max(self.hist_mq) + 100))

        self.canvas_g.draw_idle()

        # CSV 저장
        if self.csv_writer:
            row = [
                data["ts"], data["tick_ms"], data["mq"],
                f"{data['max_temp']:.2f}", data["hot_pixels"],
                int(data["fire"]),
            ]
            row += [f"{v:.2f}" for v in data["pixels"]]
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    # ── 로그 출력 ─────────────────────────────────────────────────────────────
    def _log(self, msg, tag=""):
        self.log.configure(state=tk.NORMAL)
        ts = datetime.now().strftime("%H:%M:%S")
        self.log.insert(tk.END, f"[{ts}] {msg}\n", tag or ())
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    # ── 종료 ──────────────────────────────────────────────────────────────────
    def on_close(self):
        self._disconnect()
        self.root.destroy()


# ─── 진입점 ──────────────────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    app  = ThermalMonitorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)

    # 시작 시 FPS 변수 초기화
    app._fps_count = 0
    app._fps_ts    = datetime.now().timestamp()

    root.mainloop()


if __name__ == "__main__":
    main()
