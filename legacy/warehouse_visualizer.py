import json
import queue
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk

from scenario import (
    WIDTH,
    HEIGHT,
    OBSTACLES as STATIC_OBSTACLES,
    AMR_CONFIG,
    TELEMETRY_HOST,
    TELEMETRY_PORT,
)

AMR_IDS = list(AMR_CONFIG)
AMR_PRIORITY = {
    amr_id: robot["priority"]
    for amr_id, robot in AMR_CONFIG.items()
}

# Visual-only colors. They are presentation choices, not controller logic.

# ============================================================
# VISUAL THEME — professional operations dashboard
# ============================================================

APP_BG = "#eef2f7"
PANEL_BG = "#ffffff"
HEADER_BG = "#0b1220"
HEADER_MUTED = "#94a3b8"
TEXT = "#172033"
TEXT_MUTED = "#64748b"
GRID_BG = "#f8fafc"
GRID_LINE = "#d7dee8"
GRID_AXIS = "#c3ccd8"
STATIC_RACK = "#334155"
STATIC_RACK_TEXT = "#e2e8f0"
DYNAMIC_OBSTACLE = "#dc2626"

AMR_COLORS = {
    "AMR-1": "#2563eb",
    "AMR-2": "#16a34a",
    "AMR-3": "#9333ea",
}

STATUS_COLORS = {
    "BUSY": "#f59e0b",
    "MOVING": "#22c55e",
    "INTENT": "#a855f7",
    "WAITING": "#f97316",
    "PICKUP_REACHED": "#06b6d4",
    "TASK_COMPLETE": "#14b8a6",
    "READY": "#64748b",
    "IDLE": "#64748b",
}

RISK_COLORS = {
    "LOW": "#16a34a",
    "MEDIUM": "#d97706",
    "HIGH": "#dc2626",
    "UNKNOWN": "#94a3b8",
}

ACTION_COLORS = {
    "MOVE": "#16a34a",
    "WAIT": "#d97706",
    "REPLAN": "#dc2626",
    "MONITOR": "#64748b",
}

CELL_SIZE = 42
GRID_PADDING = 42
SIDE_PANEL_WIDTH = 465
TOP_BAR_HEIGHT = 78
KPI_BAR_HEIGHT = 82
CANVAS_GRID_WIDTH = GRID_PADDING * 2 + WIDTH * CELL_SIZE
CANVAS_GRID_HEIGHT = GRID_PADDING * 2 + HEIGHT * CELL_SIZE
CANVAS_WIDTH = CANVAS_GRID_WIDTH + 32
CANVAS_HEIGHT = TOP_BAR_HEIGHT + KPI_BAR_HEIGHT + CANVAS_GRID_HEIGHT + 18

CARD_BORDER = "#dbe3ed"
CARD_ALT = "#f8fafc"
SHADOW = "#d7dee8"


class WarehouseVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("Smart Warehouse Operations Center — Distributed AMR + Edge AI")
        self.root.geometry("1360x820")
        self.root.minsize(1180, 720)
        self.root.configure(bg=APP_BG)

        self.running = True
        self.message_queue = queue.Queue()
        self.telemetry_socket = None

        self.amrs = {
            amr_id: {
                "amr_id": amr_id,
                "position": None,
                "next_position": None,
                "target": None,
                "status": "OFFLINE",
                "fleet_status": "IDLE",
                "task_id": None,
                "assigned_task": None,
                "priority": AMR_PRIORITY[amr_id],
                "path": [],
                "path_index": 0,
                "metrics": {
                    "moves": 0,
                    "conflicts": 0,
                    "replans": 0,
                    "collisions": 0,
                },
                "edge_ai": {
                    "risk_level": "UNKNOWN",
                    "confidence": 0.0,
                    "action": "MONITOR",
                    "reason": "",
                    "decisions": 0,
                },
                "dynamic_obstacles": set(),
                "last_update": 0.0,
                "event": None,
                "event_data": None,
            }
            for amr_id in AMR_IDS
        }

        self.dynamic_obstacles = set()
        self.event_log = []
        self.total_completed_tasks = set()
        self.last_event_keys = set()
        self.packet_count = 0
        self.start_time = time.time()
        self._last_panel_refresh = 0.0
        self._last_ai_log = {}
        self.ai_seen = False

        self.metric_vars = {}
        self.ai_header_var = tk.StringVar(value="EDGE AI • STANDBY")
        self.connection_var = tk.StringVar(value="TELEMETRY • CONNECTING")

        self._configure_styles()
        self._build_ui()
        self._start_receiver()
        self._schedule_refresh()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ========================================================
    # UI
    # ========================================================

    def _configure_styles(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(
            "RiskLow.Horizontal.TProgressbar",
            troughcolor="#e8f5ed",
            background=RISK_COLORS["LOW"],
            bordercolor="#e8f5ed",
            lightcolor=RISK_COLORS["LOW"],
            darkcolor=RISK_COLORS["LOW"],
        )
        style.configure(
            "RiskMedium.Horizontal.TProgressbar",
            troughcolor="#fff5df",
            background=RISK_COLORS["MEDIUM"],
            bordercolor="#fff5df",
            lightcolor=RISK_COLORS["MEDIUM"],
            darkcolor=RISK_COLORS["MEDIUM"],
        )
        style.configure(
            "RiskHigh.Horizontal.TProgressbar",
            troughcolor="#fdeaea",
            background=RISK_COLORS["HIGH"],
            bordercolor="#fdeaea",
            lightcolor=RISK_COLORS["HIGH"],
            darkcolor=RISK_COLORS["HIGH"],
        )
        style.configure(
            "RiskUnknown.Horizontal.TProgressbar",
            troughcolor="#eef2f7",
            background=RISK_COLORS["UNKNOWN"],
            bordercolor="#eef2f7",
            lightcolor=RISK_COLORS["UNKNOWN"],
            darkcolor=RISK_COLORS["UNKNOWN"],
        )

    def _build_ui(self):
        # ---------- HEADER ----------
        header = tk.Frame(self.root, bg=HEADER_BG, height=TOP_BAR_HEIGHT)
        header.pack(fill="x")
        header.pack_propagate(False)

        left = tk.Frame(header, bg=HEADER_BG)
        left.pack(side="left", padx=22, pady=12)

        tk.Label(
            left,
            text="SMART WAREHOUSE",
            bg=HEADER_BG,
            fg="white",
            font=("Helvetica", 18, "bold"),
        ).pack(anchor="w")

        tk.Label(
            left,
            text="Distributed AMR Fleet Operations Center",
            bg=HEADER_BG,
            fg=HEADER_MUTED,
            font=("Helvetica", 9),
        ).pack(anchor="w", pady=(2, 0))

        badges = tk.Frame(header, bg=HEADER_BG)
        badges.pack(side="right", padx=20)

        self.header_pill = tk.Label(
            badges,
            textvariable=self.connection_var,
            bg="#172133",
            fg="#cbd5e1",
            font=("Helvetica", 9, "bold"),
            padx=10,
            pady=5,
        )
        self.header_pill.pack(side="left", padx=4)

        self.ai_header_pill = tk.Label(
            badges,
            textvariable=self.ai_header_var,
            bg="#172133",
            fg="#cbd5e1",
            font=("Helvetica", 9, "bold"),
            padx=10,
            pady=5,
        )
        self.ai_header_pill.pack(side="left", padx=4)

        tk.Label(
            badges,
            text="OBSERVATION ONLY",
            bg="#172133",
            fg="#cbd5e1",
            font=("Helvetica", 9, "bold"),
            padx=10,
            pady=5,
        ).pack(side="left", padx=4)

        # ---------- KPI STRIP ----------
        kpi_bar = tk.Frame(self.root, bg=APP_BG, height=KPI_BAR_HEIGHT)
        kpi_bar.pack(fill="x", padx=16, pady=(12, 6))
        kpi_bar.pack_propagate(False)

        self._create_kpi(kpi_bar, "ONLINE AMRs", "online", 0)
        self._create_kpi(kpi_bar, "COMPLETED TASKS", "completed", 1)
        self._create_kpi(kpi_bar, "MOVES", "moves", 2)
        self._create_kpi(kpi_bar, "CONFLICTS", "conflicts", 3)
        self._create_kpi(kpi_bar, "REPLANS", "replans", 4)
        self._create_kpi(kpi_bar, "COLLISIONS", "collisions", 5)

        # ---------- MAIN SPLIT ----------
        content = tk.Frame(self.root, bg=APP_BG)
        content.pack(fill="both", expand=True, padx=16, pady=(0, 14))

        # Left — warehouse board
        map_card = tk.Frame(
            content,
            bg=PANEL_BG,
            highlightbackground=CARD_BORDER,
            highlightthickness=1,
        )
        map_card.pack(side="left", fill="both", expand=True, padx=(0, 10))

        map_header = tk.Frame(map_card, bg=PANEL_BG, height=54)
        map_header.pack(fill="x")
        map_header.pack_propagate(False)

        tk.Label(
            map_header,
            text="LIVE WAREHOUSE MAP",
            bg=PANEL_BG,
            fg=TEXT,
            font=("Helvetica", 12, "bold"),
        ).pack(side="left", padx=18, pady=14)

        self.map_subtitle = tk.Label(
            map_header,
            text="15 × 10 grid  •  Local A* paths  •  P2P coordination",
            bg=PANEL_BG,
            fg=TEXT_MUTED,
            font=("Helvetica", 9),
        )
        self.map_subtitle.pack(side="left", padx=8)

        self.canvas = tk.Canvas(
            map_card,
            bg=GRID_BG,
            highlightthickness=0,
            bd=0,
        )
        self.canvas.pack(fill="both", expand=True, padx=12, pady=(0, 12))

        # Right — fleet cards + log
        side = tk.Frame(
            content,
            bg=APP_BG,
            width=SIDE_PANEL_WIDTH,
        )
        side.pack(side="right", fill="y")
        side.pack_propagate(False)

        section = tk.Frame(side, bg=APP_BG)
        section.pack(fill="x")

        tk.Label(
            section,
            text="FLEET + EDGE-AI",
            bg=APP_BG,
            fg=TEXT,
            font=("Helvetica", 12, "bold"),
        ).pack(side="left")

        tk.Label(
            section,
            text="live risk-aware coordination",
            bg=APP_BG,
            fg=TEXT_MUTED,
            font=("Helvetica", 9),
        ).pack(side="right")

        self.status_frame = tk.Frame(side, bg=APP_BG)
        self.status_frame.pack(fill="x", pady=(8, 10))

        metrics_card = tk.Frame(
            side,
            bg=PANEL_BG,
            highlightbackground=CARD_BORDER,
            highlightthickness=1,
        )
        metrics_card.pack(fill="x", pady=(0, 10))

        tk.Label(
            metrics_card,
            text="SYSTEM HEALTH",
            bg=PANEL_BG,
            fg=TEXT,
            font=("Helvetica", 10, "bold"),
        ).pack(anchor="w", padx=14, pady=(10, 4))

        self.metrics_label = tk.Label(
            metrics_card,
            text="",
            justify="left",
            anchor="w",
            bg=PANEL_BG,
            fg=TEXT_MUTED,
            font=("Menlo", 9),
        )
        self.metrics_label.pack(fill="x", padx=14, pady=(0, 10))

        log_card = tk.Frame(
            side,
            bg=PANEL_BG,
            highlightbackground=CARD_BORDER,
            highlightthickness=1,
        )
        log_card.pack(fill="both", expand=True)

        log_head = tk.Frame(log_card, bg=PANEL_BG, height=38)
        log_head.pack(fill="x")
        log_head.pack_propagate(False)

        tk.Label(
            log_head,
            text="EVENT STREAM",
            bg=PANEL_BG,
            fg=TEXT,
            font=("Helvetica", 10, "bold"),
        ).pack(side="left", padx=14, pady=10)

        self.log_text = tk.Text(
            log_card,
            height=8,
            wrap="word",
            state="disabled",
            bg="#111827",
            fg="#dbeafe",
            insertbackground="white",
            relief="flat",
            borderwidth=0,
            font=("Menlo", 8),
            padx=10,
            pady=8,
        )
        self.log_text.pack(side="left", fill="both", expand=True, padx=(8, 0), pady=(0, 8))

        scrollbar = ttk.Scrollbar(
            log_card,
            orient="vertical",
            command=self.log_text.yview,
        )
        scrollbar.pack(side="right", fill="y", padx=(0, 8), pady=(0, 8))
        self.log_text.configure(yscrollcommand=scrollbar.set)

        self._draw_static_warehouse()
        self._draw_warehouse()

    def _create_kpi(self, parent, title, key, column):
        card = tk.Frame(
            parent,
            bg=PANEL_BG,
            highlightbackground=CARD_BORDER,
            highlightthickness=1,
        )
        card.grid(row=0, column=column, sticky="nsew", padx=4)

        parent.grid_columnconfigure(column, weight=1)

        tk.Label(
            card,
            text=title,
            bg=PANEL_BG,
            fg=TEXT_MUTED,
            font=("Helvetica", 8, "bold"),
        ).pack(anchor="w", padx=12, pady=(10, 0))

        var = tk.StringVar(value="0")
        self.metric_vars[key] = var

        tk.Label(
            card,
            textvariable=var,
            bg=PANEL_BG,
            fg=TEXT,
            font=("Helvetica", 18, "bold"),
        ).pack(anchor="w", padx=12, pady=(1, 9))

    # ========================================================
    # UDP RECEIVER
    # ========================================================

    def _start_receiver(self):
        thread = threading.Thread(
            target=self._receiver_loop,
            daemon=True,
        )
        thread.start()

    def _receiver_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket = sock

        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((TELEMETRY_HOST, TELEMETRY_PORT))
            sock.settimeout(0.5)

            self.message_queue.put({
                "_visualizer_event": "CONNECTED",
            })

            while self.running:
                try:
                    data, _ = sock.recvfrom(65535)
                    message = json.loads(data.decode("utf-8"))
                    self.message_queue.put(message)
                except socket.timeout:
                    continue
                except json.JSONDecodeError:
                    continue
                except OSError:
                    break
                except Exception as exc:
                    self.message_queue.put({
                        "_visualizer_event": "ERROR",
                        "error": str(exc),
                    })
        except Exception as exc:
            self.message_queue.put({
                "_visualizer_event": "ERROR",
                "error": str(exc),
            })

    # ========================================================
    # TELEMETRY PROCESSING
    # ========================================================

    def _process_queue(self):
        """Drain telemetry while preserving transient completion events.

        We coalesce telemetry so the GUI renders only the newest packet per
        AMR. TASK_COMPLETE is special: the controller emits TASK_COMPLETE and
        then quickly emits IDLE, so the completion packet can otherwise be
        overwritten before the GUI sees it.
        """
        latest_by_amr = {}
        completion_messages = []
        special_events = []

        while True:
            try:
                message = self.message_queue.get_nowait()
            except queue.Empty:
                break

            if message.get("_visualizer_event"):
                special_events.append(message)
                continue

            if message.get("type") != "TELEMETRY":
                continue

            amr_id = message.get("amr_id")
            if amr_id not in self.amrs:
                continue

            self.packet_count += 1

            # Preserve transient task-completion telemetry before coalescing.
            if message.get("status") == "TASK_COMPLETE":
                completion_messages.append(message)

            latest_by_amr[amr_id] = message

        changed = False

        # Process every completion packet even when a later IDLE packet exists.
        for message in completion_messages:
            amr_id = message.get("amr_id")
            self._record_events(amr_id, message)
            changed = True

        for message in special_events:
            event_type = message.get("_visualizer_event")
            if event_type == "CONNECTED":
                self._add_log(
                    f"Telemetry connected on {TELEMETRY_HOST}:{TELEMETRY_PORT}"
                )
                self.connection_var.set("TELEMETRY • CONNECTED")
                self.header_pill.configure(bg="#16361f", fg="#86efac")
                changed = True
            elif event_type == "ERROR":
                self._add_log(
                    f"Telemetry socket error: {message.get('error', 'unknown error')}"
                )
                self.connection_var.set("TELEMETRY • ERROR")
                self.header_pill.configure(bg="#3b1616", fg="#fca5a5")
                changed = True

        for amr_id, message in latest_by_amr.items():
            old_signature = self._amr_signature(self.amrs[amr_id])
            self._update_amr(amr_id, message)

            # TASK_COMPLETE packets were already processed above so they are
            # not lost to the IDLE packet that normally follows them.
            if message.get("status") != "TASK_COMPLETE":
                self._record_events(amr_id, message)

            if self.amrs[amr_id]["edge_ai"]["risk_level"] != "UNKNOWN":
                self.ai_seen = True

            if old_signature != self._amr_signature(self.amrs[amr_id]):
                changed = True

        if self.ai_seen:
            self.ai_header_var.set("EDGE AI • ACTIVE")
            if hasattr(self, "ai_header_pill"):
                self.ai_header_pill.configure(
                    bg="#16361f",
                    fg="#86efac",
                )

        return changed

    def _update_amr(self, amr_id, message):
        robot = self.amrs[amr_id]

        old_ai = dict(robot.get("edge_ai", {}))

        robot["position"] = self._to_cell(message.get("position"))
        robot["next_position"] = self._to_cell(message.get("next_position"))
        robot["target"] = self._to_cell(message.get("target"))
        robot["status"] = message.get("status", "UNKNOWN")
        robot["fleet_status"] = message.get(
            "fleet_status",
            robot["fleet_status"],
        )
        robot["task_id"] = message.get("task_id")
        robot["assigned_task"] = message.get("assigned_task")
        robot["priority"] = message.get(
            "priority",
            robot["priority"],
        )
        robot["path"] = [
            cell
            for cell in (
                self._to_cell(item)
                for item in message.get("path", [])
            )
            if cell is not None
        ]
        robot["path_index"] = int(
            message.get("path_index", 0) or 0
        )

        metrics = message.get("metrics", {}) or {}
        robot["metrics"] = {
            "moves": int(metrics.get("moves", 0) or 0),
            "conflicts": int(metrics.get("conflicts", 0) or 0),
            "replans": int(metrics.get("replans", 0) or 0),
            "collisions": int(metrics.get("collisions", 0) or 0),
        }

        ai = message.get("edge_ai", {}) or {}
        robot["edge_ai"] = {
            "risk_level": str(
                ai.get("risk_level", "UNKNOWN")
            ).upper(),
            "confidence": float(
                ai.get("confidence", 0.0) or 0.0
            ),
            "action": str(
                ai.get("action", "MONITOR")
            ).upper(),
            "reason": str(
                ai.get("reason", "")
            ),
            "decisions": int(
                ai.get("decisions", 0) or 0
            ),
        }

        robot["dynamic_obstacles"] = {
            cell
            for cell in (
                self._to_cell(item)
                for item in message.get("dynamic_obstacles", [])
            )
            if cell is not None
        }

        robot["last_update"] = time.time()
        robot["event"] = message.get("event")
        robot["event_data"] = message.get("event_data")

        self.dynamic_obstacles.update(
            robot["dynamic_obstacles"]
        )

        new_ai = robot["edge_ai"]
        ai_changed = (
            old_ai.get("risk_level") != new_ai["risk_level"]
            or old_ai.get("action") != new_ai["action"]
        )
        if (
            ai_changed
            and new_ai["risk_level"] != "UNKNOWN"
            and (
                new_ai["risk_level"],
                new_ai["action"],
            ) != self._last_ai_log.get(amr_id)
        ):
            self._last_ai_log[amr_id] = (
                new_ai["risk_level"],
                new_ai["action"],
            )
            confidence = new_ai["confidence"]
            reason = (
                f" • {new_ai['reason']}"
                if new_ai["reason"]
                else ""
            )
            self._add_log(
                f"{amr_id} AI: "
                f"{new_ai['risk_level']} "
                f"({confidence:.1f}%) → "
                f"{new_ai['action']}{reason}"
            )

    # ========================================================
    # EVENT LOGGING
    # ========================================================

    def _record_events(self, amr_id, message):
        event = message.get("event")
        if event:
            key = (
                amr_id,
                event,
                json.dumps(
                    message.get("event_data"),
                    sort_keys=True,
                ),
            )
            if key not in self.last_event_keys:
                self.last_event_keys.add(key)
                self._add_log(f"{amr_id}: {event}")

        status = message.get("status")
        if status == "TASK_COMPLETE":
            task_id = message.get("task_id")
            if task_id:
                self.total_completed_tasks.add(task_id)

    def _add_log(self, text):
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {text}\n"
        self.event_log.append(line.rstrip())
        self.event_log = self.event_log[-80:]

        if hasattr(self, "log_text"):
            self.log_text.configure(state="normal")
            self.log_text.delete("1.0", "end")
            self.log_text.insert(
                "end",
                "\n".join(self.event_log) + "\n",
            )
            self.log_text.see("end")
            self.log_text.configure(state="disabled")

    # ========================================================
    # REFRESH
    # ========================================================

    def _schedule_refresh(self):
        if not self.running:
            return

        changed = self._process_queue()
        now = time.time()

        if now - self._last_panel_refresh >= 0.25:
            self._refresh_status_panel()
            self._last_panel_refresh = now

        if changed:
            self._draw_warehouse()

        self.root.after(100, self._schedule_refresh)

    # ========================================================
    # DRAWING — STATIC WAREHOUSE
    # ========================================================

    def _draw_static_warehouse(self):
        self.canvas.delete("all")

        self.root.update_idletasks()
        width = max(
            self.canvas.winfo_width(),
            CANVAS_GRID_WIDTH + 24,
        )
        height = max(
            self.canvas.winfo_height(),
            CANVAS_GRID_HEIGHT + 24,
        )

        self.canvas.create_rectangle(
            0, 0, width, height,
            fill=GRID_BG,
            outline="",
            tags=("static",),
        )

        origin_x = GRID_PADDING
        origin_y = GRID_PADDING

        # Main grid cells
        for y in range(HEIGHT):
            for x in range(WIDTH):
                cell = (x, y)
                x1, y1, x2, y2 = self._cell_rect(
                    cell,
                    origin_x,
                    origin_y,
                )
                obstacle = cell in STATIC_OBSTACLES

                if obstacle:
                    fill = STATIC_RACK
                    outline = "#1e293b"
                    text_fill = STATIC_RACK_TEXT
                else:
                    fill = "#ffffff"
                    outline = GRID_LINE
                    text_fill = "#a1adbb"

                self.canvas.create_rectangle(
                    x1,
                    y1,
                    x2,
                    y2,
                    fill=fill,
                    outline=outline,
                    width=1,
                    tags=("static",),
                )

                self.canvas.create_text(
                    x1 + 4,
                    y1 + 4,
                    text=f"{x},{y}",
                    anchor="nw",
                    fill=text_fill,
                    font=("Helvetica", 7),
                    tags=("static",),
                )

                if obstacle:
                    self.canvas.create_text(
                        (x1 + x2) / 2,
                        (y1 + y2) / 2,
                        text="RACK",
                        fill="#cbd5e1",
                        font=("Helvetica", 7, "bold"),
                        tags=("static",),
                    )

        # Axis labels
        grid_right = origin_x + WIDTH * CELL_SIZE
        grid_bottom = origin_y + HEIGHT * CELL_SIZE

        for x in range(WIDTH):
            x_center = (
                origin_x
                + x * CELL_SIZE
                + CELL_SIZE / 2
            )
            self.canvas.create_text(
                x_center,
                grid_bottom + 15,
                text=str(x),
                fill="#64748b",
                font=("Helvetica", 8, "bold"),
                tags=("static",),
            )

        for y in range(HEIGHT):
            y_center = (
                grid_bottom
                - (y + 0.5) * CELL_SIZE
            )
            self.canvas.create_text(
                origin_x - 18,
                y_center,
                text=str(y),
                fill="#64748b",
                font=("Helvetica", 8, "bold"),
                tags=("static",),
            )

        # Map labels
        self.canvas.create_text(
            origin_x,
            17,
            text="WAREHOUSE FLOOR GRID",
            anchor="w",
            fill=TEXT,
            font=("Helvetica", 10, "bold"),
            tags=("static",),
        )

        self.canvas.create_text(
            grid_right,
            17,
            text="AI RISK HALO: LOW / MEDIUM / HIGH",
            anchor="e",
            fill=TEXT_MUTED,
            font=("Helvetica", 8, "bold"),
            tags=("static",),
        )

        # Legend
        legend_x = grid_right - 195
        legend_y = grid_bottom + 42
        self._draw_legend(
            legend_x,
            legend_y,
        )

    def _draw_warehouse(self):
        self.canvas.delete("dynamic")

        origin_x = GRID_PADDING
        origin_y = GRID_PADDING

        # Dynamic obstacles
        for cell in self.dynamic_obstacles:
            x1, y1, x2, y2 = self._cell_rect(
                cell,
                origin_x,
                origin_y,
            )
            self.canvas.create_rectangle(
                x1 + 3,
                y1 + 3,
                x2 - 3,
                y2 - 3,
                fill="#fee2e2",
                outline=DYNAMIC_OBSTACLE,
                width=2,
                tags=("dynamic",),
            )
            self.canvas.create_line(
                x1 + 7,
                y1 + 7,
                x2 - 7,
                y2 - 7,
                fill=DYNAMIC_OBSTACLE,
                width=2,
                tags=("dynamic",),
            )
            self.canvas.create_line(
                x2 - 7,
                y1 + 7,
                x1 + 7,
                y2 - 7,
                fill=DYNAMIC_OBSTACLE,
                width=2,
                tags=("dynamic",),
            )

        # Paths
        for amr_id in AMR_IDS:
            robot = self.amrs[amr_id]
            path = robot.get("path", [])
            if len(path) >= 2:
                self._draw_path(
                    path,
                    robot.get("path_index", 0),
                    origin_x,
                    origin_y,
                    amr_id,
                )

        # Target / next-position markers
        for amr_id in AMR_IDS:
            robot = self.amrs[amr_id]
            target = robot.get("target")
            if target is not None:
                self._draw_marker(
                    target,
                    origin_x,
                    origin_y,
                    outline=AMR_COLORS[amr_id],
                    dash=(5, 3),
                    label="DEST",
                )

            next_position = robot.get("next_position")
            if next_position is not None:
                risk = robot["edge_ai"]["risk_level"]
                next_color = RISK_COLORS.get(
                    risk,
                    STATUS_COLORS.get(
                        robot.get("status"),
                        AMR_COLORS[amr_id],
                    ),
                )
                self._draw_marker(
                    next_position,
                    origin_x,
                    origin_y,
                    outline=next_color,
                    dash=(2, 2),
                    label="NEXT",
                )

        # AMR bodies
        for amr_id in AMR_IDS:
            robot = self.amrs[amr_id]
            if robot.get("position") is not None:
                self._draw_amr(
                    robot,
                    origin_x,
                    origin_y,
                )

    def _cell_rect(self, cell, origin_x, origin_y):
        x, y = cell
        display_row = HEIGHT - 1 - y
        x1 = origin_x + x * CELL_SIZE
        y1 = origin_y + display_row * CELL_SIZE
        x2 = x1 + CELL_SIZE
        y2 = y1 + CELL_SIZE
        return x1, y1, x2, y2

    def _cell_center(self, cell, origin_x, origin_y):
        x1, y1, x2, y2 = self._cell_rect(
            cell,
            origin_x,
            origin_y,
        )
        return (
            (x1 + x2) / 2,
            (y1 + y2) / 2,
        )

    def _draw_path(
        self,
        path,
        path_index,
        origin_x,
        origin_y,
        amr_id,
    ):
        if len(path) < 2:
            return

        coords = []
        for cell in path:
            cx, cy = self._cell_center(
                cell,
                origin_x,
                origin_y,
            )
            coords.extend([cx, cy])

        color = AMR_COLORS[amr_id]

        self.canvas.create_line(
            *coords,
            fill="#cbd5e1",
            width=6,
            capstyle="round",
            smooth=False,
            tags=("dynamic",),
        )
        self.canvas.create_line(
            *coords,
            fill=color,
            width=3,
            capstyle="round",
            smooth=False,
            tags=("dynamic",),
        )

        # Highlight the current planned segment.
        try:
            idx = max(
                0,
                min(
                    int(path_index),
                    len(path) - 2,
                ),
            )
            c1 = self._cell_center(
                path[idx],
                origin_x,
                origin_y,
            )
            c2 = self._cell_center(
                path[idx + 1],
                origin_x,
                origin_y,
            )
            self.canvas.create_line(
                c1[0],
                c1[1],
                c2[0],
                c2[1],
                fill=color,
                width=6,
                capstyle="round",
                tags=("dynamic",),
            )
        except (TypeError, ValueError):
            pass

    def _draw_marker(
        self,
        cell,
        origin_x,
        origin_y,
        outline,
        dash,
        label,
    ):
        x1, y1, x2, y2 = self._cell_rect(
            cell,
            origin_x,
            origin_y,
        )
        inset = 6

        self.canvas.create_rectangle(
            x1 + inset,
            y1 + inset,
            x2 - inset,
            y2 - inset,
            outline=outline,
            width=2,
            dash=dash,
            tags=("dynamic",),
        )
        self.canvas.create_text(
            (x1 + x2) / 2,
            (y1 + y2) / 2,
            text=label,
            fill=outline,
            font=("Helvetica", 6, "bold"),
            tags=("dynamic",),
        )

    def _draw_amr(self, robot, origin_x, origin_y):
        amr_id = robot["amr_id"]
        position = robot["position"]

        x1, y1, x2, y2 = self._cell_rect(
            position,
            origin_x,
            origin_y,
        )

        color = AMR_COLORS[amr_id]
        risk = robot["edge_ai"]["risk_level"]
        risk_color = RISK_COLORS.get(
            risk,
            RISK_COLORS["UNKNOWN"],
        )

        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        # Outer AI risk halo
        self.canvas.create_oval(
            x1 + 3,
            y1 + 3,
            x2 - 3,
            y2 - 3,
            outline=risk_color,
            width=4 if risk in {"MEDIUM", "HIGH"} else 3,
            tags=("dynamic",),
        )

        # AMR body
        self.canvas.create_rectangle(
            x1 + 10,
            y1 + 10,
            x2 - 10,
            y2 - 10,
            fill=color,
            outline="white",
            width=2,
            tags=("dynamic",),
        )

        self.canvas.create_text(
            cx,
            cy - 1,
            text=amr_id.replace("AMR-", "A"),
            fill="white",
            font=("Helvetica", 10, "bold"),
            tags=("dynamic",),
        )

        # Risk badge
        if risk != "UNKNOWN":
            self.canvas.create_text(
                cx,
                y1 - 5,
                text=f"{risk} {robot['edge_ai']['confidence']:.0f}%",
                fill=risk_color,
                font=("Helvetica", 7, "bold"),
                tags=("dynamic",),
            )
        else:
            self.canvas.create_text(
                cx,
                y1 - 5,
                text="AI WAITING",
                fill=TEXT_MUTED,
                font=("Helvetica", 7, "bold"),
                tags=("dynamic",),
            )

        # Robot status
        status = robot.get("status", "UNKNOWN")
        status_color = STATUS_COLORS.get(
            status,
            color,
        )
        self.canvas.create_text(
            cx,
            y2 + 5,
            text=status,
            fill=status_color,
            font=("Helvetica", 7, "bold"),
            tags=("dynamic",),
        )

    def _draw_legend(self, x, y):
        self.canvas.create_text(
            x,
            y,
            text="LEGEND",
            anchor="nw",
            fill=TEXT,
            font=("Helvetica", 9, "bold"),
            tags=("static",),
        )

        rows = [
            ("AMR-1", AMR_COLORS["AMR-1"]),
            ("AMR-2", AMR_COLORS["AMR-2"]),
            ("AMR-3", AMR_COLORS["AMR-3"]),
            ("Static rack", STATIC_RACK),
            ("Dynamic obstacle", DYNAMIC_OBSTACLE),
            ("AI LOW", RISK_COLORS["LOW"]),
            ("AI MEDIUM", RISK_COLORS["MEDIUM"]),
            ("AI HIGH", RISK_COLORS["HIGH"]),
        ]

        for idx, (label, color) in enumerate(rows):
            col = idx // 4
            row = idx % 4

            xx = x + col * 98
            yy = y + 18 + row * 17

            self.canvas.create_rectangle(
                xx,
                yy,
                xx + 10,
                yy + 10,
                fill=color,
                outline=color,
                tags=("static",),
            )
            self.canvas.create_text(
                xx + 16,
                yy + 5,
                text=label,
                anchor="w",
                fill=TEXT_MUTED,
                font=("Helvetica", 7),
                tags=("static",),
            )

    # ========================================================
    # RIGHT PANEL
    # ========================================================

    def _refresh_status_panel(self):
        for child in self.status_frame.winfo_children():
            child.destroy()

        now = time.time()

        for amr_id in AMR_IDS:
            robot = self.amrs[amr_id]
            age = now - robot.get("last_update", 0.0)
            online = age <= 3.0

            self._create_amr_card(
                self.status_frame,
                robot,
                online,
            )

        metrics = self._collect_metrics()
        uptime = int(time.time() - self.start_time)

        self.metric_vars["online"].set(
            str(
                sum(
                    1
                    for r in self.amrs.values()
                    if now - r.get("last_update", 0.0) <= 3.0
                )
            )
        )
        self.metric_vars["completed"].set(
            str(len(self.total_completed_tasks))
        )
        self.metric_vars["moves"].set(
            str(metrics["moves"])
        )
        self.metric_vars["conflicts"].set(
            str(metrics["conflicts"])
        )
        self.metric_vars["replans"].set(
            str(metrics["replans"])
        )
        self.metric_vars["collisions"].set(
            str(metrics["collisions"])
        )

        ai_active_count = sum(
            1
            for r in self.amrs.values()
            if r["edge_ai"]["risk_level"] != "UNKNOWN"
        )

        self.metrics_label.configure(
            text=(
                f"Uptime             {uptime:>5}s\n"
                f"Packets received   {self.packet_count:>5}\n"
                f"AI-enabled AMRs    {ai_active_count:>5}/3\n"
                f"Dynamic obstacles  {len(self.dynamic_obstacles):>5}\n"
                f"AI decisions       "
                f"{sum(r['edge_ai']['decisions'] for r in self.amrs.values()):>5}"
            )
        )

        if self.ai_seen:
            self.ai_header_var.set("EDGE AI • ACTIVE")
            if hasattr(self, "ai_header_pill"):
                self.ai_header_pill.configure(
                    bg="#16361f",
                    fg="#86efac",
                )

    def _create_amr_card(self, parent, robot, online):
        risk = robot["edge_ai"]["risk_level"]
        action = robot["edge_ai"]["action"]
        confidence = max(
            0.0,
            min(
                100.0,
                robot["edge_ai"]["confidence"],
            ),
        )

        risk_color = RISK_COLORS.get(
            risk,
            RISK_COLORS["UNKNOWN"],
        )
        action_color = ACTION_COLORS.get(
            action,
            ACTION_COLORS["MONITOR"],
        )
        card_accent = AMR_COLORS[robot["amr_id"]]

        card = tk.Frame(
            parent,
            bg=PANEL_BG,
            highlightbackground=CARD_BORDER,
            highlightthickness=1,
        )
        card.pack(fill="x", pady=4)

        top = tk.Frame(card, bg=PANEL_BG)
        top.pack(fill="x", padx=12, pady=(10, 5))

        name = tk.Label(
            top,
            text=robot["amr_id"],
            bg=PANEL_BG,
            fg=card_accent,
            font=("Helvetica", 11, "bold"),
        )
        name.pack(side="left")

        tk.Label(
            top,
            text=f"P{robot['priority']}",
            bg=card_accent,
            fg="white",
            font=("Helvetica", 7, "bold"),
            padx=6,
            pady=2,
        ).pack(side="left", padx=6)

        tk.Label(
            top,
            text="ONLINE" if online else "OFFLINE",
            bg="#e8f5ed" if online else "#f1f5f9",
            fg="#166534" if online else "#64748b",
            font=("Helvetica", 7, "bold"),
            padx=7,
            pady=3,
        ).pack(side="right")

        data_row = tk.Frame(card, bg=PANEL_BG)
        data_row.pack(fill="x", padx=12)

        position = robot["position"]
        next_position = robot["next_position"]
        task = robot["task_id"] or "—"
        status = robot["status"]
        fleet_status = robot["fleet_status"]

        left_text = (
            f"POS  {position if position else '—'}   "
            f"NEXT  {next_position if next_position else '—'}\n"
            f"TASK {task}   •   {status}/{fleet_status}"
        )

        tk.Label(
            data_row,
            text=left_text,
            bg=PANEL_BG,
            fg=TEXT_MUTED,
            justify="left",
            anchor="w",
            font=("Menlo", 8),
        ).pack(side="left", fill="x", expand=True)

        risk_badge = tk.Label(
            data_row,
            text=risk,
            bg=risk_color,
            fg="white",
            font=("Helvetica", 8, "bold"),
            padx=9,
            pady=4,
        )
        risk_badge.pack(side="right")

        # Confidence row
        conf_row = tk.Frame(card, bg=PANEL_BG)
        conf_row.pack(fill="x", padx=12, pady=(7, 0))

        tk.Label(
            conf_row,
            text=f"AI confidence  {confidence:.1f}%",
            bg=PANEL_BG,
            fg=TEXT_MUTED,
            font=("Helvetica", 8),
        ).pack(side="left")

        pb_style = {
            "LOW": "RiskLow.Horizontal.TProgressbar",
            "MEDIUM": "RiskMedium.Horizontal.TProgressbar",
            "HIGH": "RiskHigh.Horizontal.TProgressbar",
            "UNKNOWN": "RiskUnknown.Horizontal.TProgressbar",
        }.get(
            risk,
            "RiskUnknown.Horizontal.TProgressbar",
        )

        bar = ttk.Progressbar(
            conf_row,
            style=pb_style,
            maximum=100,
            value=confidence,
        )
        bar.pack(
            side="right",
            fill="x",
            expand=True,
            padx=(10, 0),
        )

        action_row = tk.Frame(card, bg=CARD_ALT)
        action_row.pack(fill="x", padx=10, pady=(7, 10))

        tk.Label(
            action_row,
            text=f"AI ACTION  {action}",
            bg=CARD_ALT,
            fg=action_color,
            font=("Helvetica", 8, "bold"),
        ).pack(side="left", padx=7, pady=6)

        reason = robot["edge_ai"]["reason"] or "No current risk reason"
        if len(reason) > 42:
            reason = reason[:39] + "..."

        tk.Label(
            action_row,
            text=reason,
            bg=CARD_ALT,
            fg=TEXT_MUTED,
            font=("Helvetica", 7),
        ).pack(
            side="right",
            padx=7,
            pady=6,
        )

    def _collect_metrics(self):
        total = {
            "moves": 0,
            "conflicts": 0,
            "replans": 0,
            "collisions": 0,
        }

        for robot in self.amrs.values():
            for key in total:
                total[key] += robot["metrics"].get(
                    key,
                    0,
                )

        return total

    # ========================================================
    # HELPERS / SHUTDOWN
    # ========================================================

    @staticmethod
    def _to_cell(value):
        if value is None:
            return None
        try:
            if len(value) != 2:
                return None
            return (
                int(value[0]),
                int(value[1]),
            )
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _amr_signature(robot):
        """Display-relevant state for cheap change detection."""
        edge_ai = robot.get("edge_ai", {})
        return (
            robot.get("position"),
            robot.get("next_position"),
            robot.get("target"),
            robot.get("status"),
            robot.get("fleet_status"),
            robot.get("task_id"),
            tuple(robot.get("path", [])),
            robot.get("path_index", 0),
            tuple(
                sorted(
                    robot.get(
                        "dynamic_obstacles",
                        set(),
                    )
                )
            ),
            tuple(
                sorted(
                    robot.get(
                        "metrics",
                        {},
                    ).items()
                )
            ),
            edge_ai.get("risk_level"),
            round(
                float(
                    edge_ai.get(
                        "confidence",
                        0.0,
                    )
                ),
                1,
            ),
            edge_ai.get("action"),
            edge_ai.get("reason"),
            robot.get("event"),
            repr(robot.get("event_data")),
        )

    def close(self):
        self.running = False
        if self.telemetry_socket is not None:
            try:
                self.telemetry_socket.close()
            except OSError:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    WarehouseVisualizer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
