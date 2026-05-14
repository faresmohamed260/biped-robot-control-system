from __future__ import annotations

import json
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
from tkinter import messagebox, ttk

import cv2
import requests
from PIL import Image, ImageTk

from biped_endpoint_discovery import discover_biped_endpoint
from biped_vision_tracking import BALL_COLOR_RANGES, ROBOT_COLOR_RANGES, RobotBallTracker, TrackingConfig
from biped_vision_camera import fetch_ip_webcam_frame, get_cached_ip_webcam_url


APP_ROOT = Path(__file__).resolve().parent
CONFIG_PATH = APP_ROOT / "biped_desktop_app_config.json"
VISION_RECORDINGS_DIR = APP_ROOT / "output" / "vision_recordings"
APP_BRANDING_DIR = APP_ROOT / "branding" / "app"
APP_ICON_PATH = APP_BRANDING_DIR / "app_icon.ico"
APP_BANNER_PATH = APP_BRANDING_DIR / "app_banner.png"
MAIN_LOGO_PATH = APP_BRANDING_DIR / "main_logo.png"

PALETTE_YELLOW = "#FFC107"
PALETTE_WHITE = "#FFFFFF"
PALETTE_BLACK = "#1E1E1E"
PALETTE_MID = "#555555"
PALETTE_LIGHT = "#A0A0A0"
PANEL_BG = "#242424"
INPUT_BG = "#2C2C2C"
SIDEBAR_BG = "#181818"
SURFACE_BG = "#202020"
CONTROL_MODE_OPTIONS = [
    ("none", 0),
    ("axis", 1),
    ("buttons", 2),
]
AXIS_SOURCE_OPTIONS = [
    ("none", 0),
    ("left_stick_x", 1),
    ("left_stick_y", 2),
    ("right_stick_x", 3),
    ("right_stick_y", 4),
    ("dpad_x", 5),
    ("dpad_y", 6),
    ("triggers", 7),
]
BUTTON_OPTIONS = [
    ("none", 0),
    ("up", 1),
    ("down", 2),
    ("left", 3),
    ("right", 4),
    ("square", 5),
    ("cross", 6),
    ("circle", 7),
    ("triangle", 8),
    ("l1", 9),
    ("r1", 10),
    ("l2", 11),
    ("r2", 12),
    ("share", 13),
    ("options", 14),
    ("l3", 15),
    ("r3", 16),
    ("ps", 17),
    ("touchpad", 18),
]
JOINT_ORDER = ["base", "wrist_rotate", "shoulder", "wrist_pitch", "elbow", "gripper"]
POSE_JOINTS = ["base", "shoulder", "elbow", "wrist_pitch", "wrist_rotate", "gripper"]
JOINT_LABELS = {
    "base": "Left Ankle",
    "shoulder": "Left Knee",
    "elbow": "Left Hip",
    "wrist_pitch": "Right Knee",
    "wrist_rotate": "Right Ankle",
    "gripper": "Right Hip",
}

DEFAULT_CONFIG = {
    "base_url": "http://192.168.1.132",
    "camera_url": get_cached_ip_webcam_url() or "http://192.168.1.141:8080/shot.jpg",
    "robot_marker_color": "Yellow",
    "ball_color": "Red",
    "alignment_deadband_px": 45,
    "collision_distance_cm": 18.0,
    "ball_diameter_cm": 22.0,
    "min_robot_area_px": 900,
    "min_ball_area_px": 120,
    "pose_duration_ms": 300,
    "interp_steps": 16,
    "hold_ms": 60,
    "pause_between_actions_ms": 250,
    "forward_sequence": "stand,left_forward,right_forward,stand",
    "turn_left_sequence": "shift_left,stand",
    "turn_right_sequence": "shift_right,stand",
    "collision_sequence": "",
}


def load_config() -> dict:
    if CONFIG_PATH.exists():
        try:
            payload = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                merged = dict(DEFAULT_CONFIG)
                merged.update(payload)
                return merged
        except json.JSONDecodeError:
            pass
    return dict(DEFAULT_CONFIG)


def save_config(config: dict) -> None:
    CONFIG_PATH.write_text(json.dumps(config, indent=2), encoding="utf-8")


class RobotClient:
    def __init__(self) -> None:
        self.session = requests.Session()
        self.session.trust_env = False
        self.session.headers.update({"Connection": "close", "User-Agent": "Biped-Desktop/1.0"})
        self.base_url = ""

    def set_base_url(self, base_url: str) -> None:
        normalized = base_url.strip().rstrip("/")
        if not normalized.startswith(("http://", "https://")):
            normalized = f"http://{normalized}"
        self.base_url = normalized

    def _get(self, path: str, params: dict | None = None, timeout: float = 4.0) -> dict:
        if not self.base_url:
            raise RuntimeError("Robot base URL is not set.")
        response = self.session.get(f"{self.base_url}{path}", params=params, timeout=timeout)
        response.raise_for_status()
        data = response.json()
        if isinstance(data, dict) and data.get("ok") is False:
            raise RuntimeError(str(data.get("error", "Device returned an error.")))
        return data

    def get_state(self) -> dict:
        return self._get("/api/state", timeout=5.0)

    def move_joint(self, joint: str, angle: int) -> None:
        self._get("/api/joint", params={"cmd": "move", "joint": joint, "value": int(angle)}, timeout=4.0)

    def move_pose(self, pose: dict[str, int], delay_ms: int = 40) -> None:
        for joint_name in JOINT_ORDER:
            if joint_name not in pose:
                continue
            self.move_joint(joint_name, int(pose[joint_name]))
            if delay_ms > 0:
                time.sleep(delay_ms / 1000.0)

    def save_pose(self, name: str, pose: dict[str, int]) -> None:
        params = {"cmd": "save_pose", "name": name}
        params.update({joint_name: int(pose[joint_name]) for joint_name in POSE_JOINTS})
        self._get("/api/biped", params=params, timeout=5.0)

    def run_pose(self, name: str, duration_ms: int, interp_steps: int, hold_ms: int) -> None:
        self._get(
            "/api/biped",
            params={
                "cmd": "run_pose",
                "name": name,
                "duration_ms": int(duration_ms),
                "interp_steps": int(interp_steps),
                "hold_ms": int(hold_ms),
            },
            timeout=max(5.0, duration_ms / 1000.0 + hold_ms / 1000.0 + 5.0),
        )

    def play_sequence(self, names: str, duration_ms: int, interp_steps: int, hold_ms: int, repeat_count: int = 1) -> None:
        self._get(
            "/api/biped",
            params={
                "cmd": "play_sequence",
                "names": names,
                "duration_ms": int(duration_ms),
                "interp_steps": int(interp_steps),
                "hold_ms": int(hold_ms),
                "repeat": int(repeat_count),
            },
            timeout=max(5.0, repeat_count * 10.0),
        )

    def stand(self, duration_ms: int, interp_steps: int, hold_ms: int) -> None:
        self._get(
            "/api/biped",
            params={
                "cmd": "stand",
                "duration_ms": int(duration_ms),
                "interp_steps": int(interp_steps),
                "hold_ms": int(hold_ms),
            },
            timeout=max(5.0, duration_ms / 1000.0 + hold_ms / 1000.0 + 5.0),
        )

    def system_action(self, cmd: str) -> dict:
        return self._get("/api/system", params={"cmd": cmd}, timeout=6.0)

    def wifi_set(self, *, hostname: str, ap_ssid: str, ap_password: str, sta_ssid: str, sta_password: str) -> dict:
        return self._get(
            "/api/wifi",
            params={
                "cmd": "set",
                "hostname": hostname,
                "ap_ssid": ap_ssid,
                "ap_password": ap_password,
                "sta_ssid": sta_ssid,
                "sta_password": sta_password,
            },
            timeout=8.0,
        )

    def wifi_reconnect(self) -> dict:
        return self._get("/api/wifi", params={"cmd": "reconnect"}, timeout=6.0)

    def ps4_action(self, cmd: str, **params: int | str) -> dict:
        payload = {"cmd": cmd}
        payload.update(params)
        return self._get("/api/ps4", params=payload, timeout=6.0)

    def joint_action(self, joint_name: str, cmd: str, **params: int | str) -> dict:
        payload = {"cmd": cmd, "joint": joint_name}
        payload.update(params)
        return self._get("/api/joint", params=payload, timeout=6.0)


@dataclass
class VisionFrame:
    rgb_image: object
    info: dict
    action_text: str


class DumeDesktopApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Biped Robot Desktop Control")
        self._configure_window_size()
        self.root.configure(bg=PALETTE_BLACK)
        self._banner_photo: ImageTk.PhotoImage | None = None
        self._logo_photo: ImageTk.PhotoImage | None = None
        self._current_video_label_image: ImageTk.PhotoImage | None = None
        self._last_video_rgb_image: object | None = None
        self._nav_buttons: dict[str, tk.Button] = {}
        self._panel_frames: dict[str, ttk.Frame] = {}
        self._active_panel = "setup"
        self._configure_theme()
        self._load_branding_assets()
        self._apply_window_icon()

        self.config = load_config()
        self.client = RobotClient()
        self.client.set_base_url(str(self.config["base_url"]))

        self.current_state: dict | None = None
        self.pose_map: dict[str, dict[str, int]] = {}
        self.joint_state_map: dict[str, dict] = {}
        self.image_queue: Queue[VisionFrame] = Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.feed_enabled = False
        self.autonomy_enabled = False
        self.worker_thread: threading.Thread | None = None
        self.active_vision_settings: dict = {}
        self.last_frame_info: dict | None = None
        self.current_photo: ImageTk.PhotoImage | None = None
        self.last_logged_action = ""
        self.recording_lock = threading.Lock()
        self.recorded_autonomy_frames: list[object] = []
        self.last_recorded_autonomy_frames: list[object] = []
        self.last_exported_video_path: Path | None = None

        self.base_url_var = tk.StringVar(value=str(self.config["base_url"]))
        self.camera_url_var = tk.StringVar(value=str(self.config["camera_url"]))
        self.robot_color_var = tk.StringVar(value=str(self.config["robot_marker_color"]))
        self.ball_color_var = tk.StringVar(value=str(self.config["ball_color"]))
        self.status_var = tk.StringVar(value="Disconnected")
        self.system_info_var = tk.StringVar(value="No robot connected.")
        self.feed_status_var = tk.StringVar(value="Feed stopped")
        self.info_var = tk.StringVar(value="No frame yet.")
        self.pose_var = tk.StringVar(value="stand")
        self.joint_var = tk.StringVar(value="base")
        self.sequence_forward_var = tk.StringVar(value=str(self.config["forward_sequence"]))
        self.sequence_left_var = tk.StringVar(value=str(self.config["turn_left_sequence"]))
        self.sequence_right_var = tk.StringVar(value=str(self.config["turn_right_sequence"]))
        self.sequence_collision_var = tk.StringVar(value=str(self.config["collision_sequence"]))
        self.alignment_deadband_var = tk.IntVar(value=int(self.config["alignment_deadband_px"]))
        self.collision_distance_var = tk.DoubleVar(value=float(self.config["collision_distance_cm"]))
        self.ball_diameter_var = tk.DoubleVar(value=float(self.config["ball_diameter_cm"]))
        self.min_robot_area_var = tk.IntVar(value=int(self.config["min_robot_area_px"]))
        self.min_ball_area_var = tk.IntVar(value=int(self.config["min_ball_area_px"]))
        self.pose_duration_var = tk.IntVar(value=int(self.config["pose_duration_ms"]))
        self.interp_steps_var = tk.IntVar(value=int(self.config["interp_steps"]))
        self.hold_ms_var = tk.IntVar(value=int(self.config["hold_ms"]))
        self.pause_between_actions_var = tk.IntVar(value=int(self.config["pause_between_actions_ms"]))
        self.hostname_var = tk.StringVar(value="")
        self.ap_ssid_var = tk.StringVar(value="Biped-Setup")
        self.ap_password_var = tk.StringVar(value="robotarm123")
        self.sta_ssid_var = tk.StringVar(value="")
        self.sta_password_var = tk.StringVar(value="")
        self.wifi_info_var = tk.StringVar(value="No Wi-Fi state yet.")
        self.controller_info_var = tk.StringVar(value="No controller state yet.")
        self.raw_state_var = tk.StringVar(value="{}")
        self.controller_enabled_var = tk.BooleanVar(value=False)
        self.controller_pair_mode_var = tk.BooleanVar(value=False)
        self.controller_home_button_var = tk.StringVar(value="none")
        self.controller_led_r_var = tk.IntVar(value=0)
        self.controller_led_g_var = tk.IntVar(value=0)
        self.controller_led_b_var = tk.IntVar(value=255)
        self.controller_rumble_force_var = tk.IntVar(value=0)
        self.controller_rumble_duration_var = tk.IntVar(value=0)
        self.controller_deadzone_var = tk.IntVar(value=48)

        self.joint_pin_var = tk.IntVar(value=0)
        self.joint_min_var = tk.IntVar(value=0)
        self.joint_max_var = tk.IntVar(value=180)
        self.joint_home_var = tk.IntVar(value=90)
        self.joint_step_var = tk.IntVar(value=2)
        self.joint_pulse_min_var = tk.IntVar(value=500)
        self.joint_pulse_max_var = tk.IntVar(value=2400)
        self.joint_physical_min_var = tk.IntVar(value=0)
        self.joint_physical_max_var = tk.IntVar(value=180)
        self.joint_neutral_output_var = tk.IntVar(value=90)
        self.joint_stop_deadband_var = tk.IntVar(value=3)
        self.joint_max_speed_scale_var = tk.IntVar(value=100)
        self.joint_invert_var = tk.BooleanVar(value=False)
        self.joint_input_invert_var = tk.BooleanVar(value=False)
        self.joint_move_var = tk.IntVar(value=90)
        self.joint_nudge_var = tk.IntVar(value=2)
        self.joint_control_mode_var = tk.StringVar(value="none")
        self.joint_axis_source_var = tk.StringVar(value="none")
        self.joint_positive_button_var = tk.StringVar(value="none")
        self.joint_negative_button_var = tk.StringVar(value="none")
        self.joint_info_var = tk.StringVar(value="No joint selected.")

        self.pose_values: dict[str, tk.IntVar] = {
            joint_name: tk.IntVar(value=90) for joint_name in POSE_JOINTS
        }

        self._build_ui()
        self.root.after(100, self._poll_image_queue)
        self.root.after(300, self.auto_connect_robot)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _configure_window_size(self) -> None:
        screen_width = max(1024, self.root.winfo_screenwidth())
        screen_height = max(700, self.root.winfo_screenheight())
        window_width = min(1480, max(1024, screen_width - 80))
        window_height = min(920, max(700, screen_height - 120))
        minimum_width = min(1120, max(980, screen_width - 160))
        minimum_height = min(760, max(660, screen_height - 180))
        self.root.geometry(f"{window_width}x{window_height}")
        self.root.minsize(minimum_width, minimum_height)

    def _configure_theme(self) -> None:
        style = ttk.Style(self.root)
        if "clam" in style.theme_names():
            style.theme_use("clam")

        style.configure(".", background=PALETTE_BLACK, foreground=PALETTE_WHITE, fieldbackground=INPUT_BG)
        style.configure("TFrame", background=PALETTE_BLACK)
        style.configure("Header.TFrame", background=PALETTE_BLACK)
        style.configure("Card.TFrame", background=SURFACE_BG)
        style.configure("TLabel", background=PALETTE_BLACK, foreground=PALETTE_WHITE)
        style.configure("Muted.TLabel", background=PALETTE_BLACK, foreground=PALETTE_LIGHT)
        style.configure("HeaderTitle.TLabel", background=PALETTE_BLACK, foreground=PALETTE_WHITE, font=("Segoe UI", 22, "bold"))
        style.configure("HeaderSub.TLabel", background=PALETTE_BLACK, foreground=PALETTE_LIGHT, font=("Segoe UI", 10))
        style.configure("TLabelframe", background=SURFACE_BG, bordercolor=PALETTE_MID, relief="solid", padding=10)
        style.configure("TLabelframe.Label", background=SURFACE_BG, foreground=PALETTE_YELLOW, font=("Segoe UI", 10, "bold"))
        style.configure("TButton", background=PALETTE_YELLOW, foreground=PALETTE_BLACK, borderwidth=0, focusthickness=0, padding=(10, 6), font=("Segoe UI", 9, "bold"))
        style.map("TButton", background=[("active", "#FFD54F"), ("pressed", "#E0A800"), ("disabled", PALETTE_MID)], foreground=[("disabled", PALETTE_LIGHT)])
        style.configure("TEntry", fieldbackground=INPUT_BG, foreground=PALETTE_WHITE, insertcolor=PALETTE_WHITE, bordercolor=PALETTE_MID)
        style.configure("TCombobox", fieldbackground=INPUT_BG, foreground=PALETTE_WHITE, arrowsize=16, bordercolor=PALETTE_MID)
        style.map("TCombobox", fieldbackground=[("readonly", INPUT_BG)], selectbackground=[("readonly", PALETTE_YELLOW)], selectforeground=[("readonly", PALETTE_BLACK)])
        style.configure("TSpinbox", fieldbackground=INPUT_BG, foreground=PALETTE_WHITE, arrowsize=14, bordercolor=PALETTE_MID)
        style.configure("TCheckbutton", background=SURFACE_BG, foreground=PALETTE_WHITE)
        style.map("TCheckbutton", background=[("active", SURFACE_BG)], foreground=[("active", PALETTE_WHITE)])
        style.configure("Vertical.TScrollbar", background=PANEL_BG, troughcolor=PALETTE_BLACK, bordercolor=PALETTE_BLACK, arrowcolor=PALETTE_YELLOW)
        style.configure("Horizontal.TScrollbar", background=PANEL_BG, troughcolor=PALETTE_BLACK, bordercolor=PALETTE_BLACK, arrowcolor=PALETTE_YELLOW)

    def _load_branding_assets(self) -> None:
        try:
            banner_path = APP_BANNER_PATH if APP_BANNER_PATH.exists() else MAIN_LOGO_PATH
            if banner_path.exists():
                banner = Image.open(banner_path)
                banner.thumbnail((1100, 96))
                self._banner_photo = ImageTk.PhotoImage(banner)
        except Exception:
            self._banner_photo = None
        try:
            if MAIN_LOGO_PATH.exists():
                logo = Image.open(MAIN_LOGO_PATH)
                logo.thumbnail((150, 80))
                self._logo_photo = ImageTk.PhotoImage(logo)
        except Exception:
            self._logo_photo = None

    def _apply_window_icon(self) -> None:
        try:
            if APP_ICON_PATH.exists():
                self.root.iconbitmap(default=str(APP_ICON_PATH))
        except Exception:
            pass

    def _make_themed_scale(self, parent: tk.Misc, variable: tk.Variable, *, from_: int, to: int, length: int) -> tk.Scale:
        scale = tk.Scale(
            parent,
            from_=from_,
            to=to,
            orient="horizontal",
            resolution=1,
            showvalue=False,
            variable=variable,
            length=length,
            bg=PANEL_BG,
            fg=PALETTE_WHITE,
            troughcolor=PALETTE_MID,
            activebackground=PALETTE_YELLOW,
            highlightthickness=0,
            bd=0,
            sliderlength=22,
        )
        return scale

    def _make_nav_button(self, parent: tk.Misc, key: str, label: str) -> tk.Button:
        button = tk.Button(
            parent,
            text=label,
            command=lambda: self._show_panel(key),
            bg=SIDEBAR_BG,
            fg=PALETTE_WHITE,
            activebackground=PALETTE_YELLOW,
            activeforeground=PALETTE_BLACK,
            relief="flat",
            bd=0,
            highlightthickness=0,
            anchor="w",
            padx=18,
            pady=12,
            font=("Segoe UI", 10, "bold"),
            cursor="hand2",
        )
        return button

    def _show_panel(self, panel_key: str) -> None:
        for key, frame in self._panel_frames.items():
            if key == panel_key:
                frame.tkraise()
            button = self._nav_buttons.get(key)
            if button is not None:
                if key == panel_key:
                    button.configure(bg=PALETTE_YELLOW, fg=PALETTE_BLACK)
                else:
                    button.configure(bg=SIDEBAR_BG, fg=PALETTE_WHITE)
        self._active_panel = panel_key

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, style="Header.TFrame")
        container.pack(fill="both", expand=True, padx=12, pady=12)

        header = ttk.Frame(container, style="Header.TFrame")
        header.pack(fill="x", pady=(0, 10))
        header.columnconfigure(1, weight=1)

        if self._logo_photo is not None:
            logo_label = tk.Label(header, image=self._logo_photo, bg=PALETTE_BLACK, bd=0)
            logo_label.grid(row=0, column=0, rowspan=2, sticky="w", padx=(0, 12))

        ttk.Label(header, text="Biped Robot Control Center", style="HeaderTitle.TLabel").grid(row=0, column=1, sticky="sw")
        ttk.Label(
            header,
            text="Desktop setup, calibration, and vision-guided motion using the new yellow-black palette.",
            style="HeaderSub.TLabel",
        ).grid(row=1, column=1, sticky="nw", pady=(2, 0))

        if self._banner_photo is not None:
            banner_label = tk.Label(container, image=self._banner_photo, bg=PALETTE_BLACK, bd=0)
            banner_label.pack(fill="x", pady=(0, 10))

        body = ttk.Frame(container, style="Header.TFrame")
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        sidebar = tk.Frame(body, bg=SIDEBAR_BG, width=230, bd=0, highlightthickness=0)
        sidebar.grid(row=0, column=0, sticky="nsw", padx=(0, 12))
        sidebar.grid_propagate(False)

        sidebar_top = tk.Frame(sidebar, bg=SIDEBAR_BG)
        sidebar_top.pack(fill="x", padx=12, pady=(16, 10))
        tk.Label(sidebar_top, text="Navigation", bg=SIDEBAR_BG, fg=PALETTE_YELLOW, font=("Segoe UI", 10, "bold")).pack(anchor="w")
        tk.Label(sidebar_top, text="Choose a control surface", bg=SIDEBAR_BG, fg=PALETTE_LIGHT, font=("Segoe UI", 9)).pack(anchor="w", pady=(4, 0))

        nav_items = [
            ("setup", "Setup"),
            ("joints", "Joints"),
            ("wifi", "Wi-Fi"),
            ("controller", "Controller"),
            ("vision", "Vision"),
            ("state", "State"),
        ]
        nav_holder = tk.Frame(sidebar, bg=SIDEBAR_BG)
        nav_holder.pack(fill="x", padx=10, pady=(4, 12))
        for key, label in nav_items:
            button = self._make_nav_button(nav_holder, key, label)
            button.pack(fill="x", pady=4)
            self._nav_buttons[key] = button

        status_card = tk.Frame(sidebar, bg=PANEL_BG, bd=0, highlightthickness=0)
        status_card.pack(fill="x", padx=12, pady=(8, 12))
        tk.Label(status_card, text="Connection", bg=PANEL_BG, fg=PALETTE_YELLOW, font=("Segoe UI", 10, "bold")).pack(anchor="w", padx=12, pady=(12, 2))
        tk.Label(status_card, textvariable=self.status_var, bg=PANEL_BG, fg=PALETTE_WHITE, wraplength=190, justify="left", font=("Segoe UI", 9)).pack(anchor="w", padx=12, pady=(0, 8))
        tk.Label(status_card, textvariable=self.system_info_var, bg=PANEL_BG, fg=PALETTE_LIGHT, wraplength=190, justify="left", font=("Segoe UI", 8)).pack(anchor="w", padx=12, pady=(0, 12))

        content_shell = ttk.Frame(body, style="Card.TFrame")
        content_shell.grid(row=0, column=1, sticky="nsew")
        content_shell.columnconfigure(0, weight=1)
        content_shell.rowconfigure(0, weight=1)

        self.setup_tab = ttk.Frame(content_shell, style="TFrame")
        self.joints_tab = ttk.Frame(content_shell, style="TFrame")
        self.wifi_tab = ttk.Frame(content_shell, style="TFrame")
        self.controller_tab = ttk.Frame(content_shell, style="TFrame")
        self.vision_tab = ttk.Frame(content_shell, style="TFrame")
        self.state_tab = ttk.Frame(content_shell, style="TFrame")
        self._panel_frames = {
            "setup": self.setup_tab,
            "joints": self.joints_tab,
            "wifi": self.wifi_tab,
            "controller": self.controller_tab,
            "vision": self.vision_tab,
            "state": self.state_tab,
        }
        for frame in self._panel_frames.values():
            frame.grid(row=0, column=0, sticky="nsew")

        self._build_setup_tab()
        self._build_joints_tab()
        self._build_wifi_tab()
        self._build_controller_tab()
        self._build_vision_tab()
        self._build_state_tab()
        self._show_panel("setup")

    def _build_setup_tab(self) -> None:
        self.setup_tab.columnconfigure(0, weight=1)
        self.setup_tab.columnconfigure(1, weight=1)
        self.setup_tab.rowconfigure(2, weight=1)

        connection = ttk.LabelFrame(self.setup_tab, text="Connection")
        connection.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=8)
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="Robot base URL").grid(row=0, column=0, padx=8, pady=8, sticky="w")
        ttk.Entry(connection, textvariable=self.base_url_var).grid(row=0, column=1, padx=8, pady=8, sticky="ew")
        ttk.Button(connection, text="Connect", command=self.connect_robot).grid(row=0, column=2, padx=6, pady=8)
        ttk.Button(connection, text="Auto Discover", command=self.auto_connect_robot).grid(row=0, column=3, padx=6, pady=8)
        ttk.Button(connection, text="Refresh", command=self.refresh_state).grid(row=0, column=4, padx=6, pady=8)
        ttk.Button(connection, text="Stand", command=self.run_stand).grid(row=0, column=5, padx=6, pady=8)
        ttk.Button(connection, text="Save Flash", command=self.save_flash).grid(row=0, column=6, padx=6, pady=8)
        ttk.Button(connection, text="Reload Flash", command=self.reload_flash).grid(row=0, column=7, padx=6, pady=8)
        ttk.Label(connection, textvariable=self.status_var).grid(row=1, column=0, columnspan=5, padx=8, pady=(0, 8), sticky="w")
        ttk.Label(connection, textvariable=self.system_info_var).grid(row=2, column=0, columnspan=8, padx=8, pady=(0, 8), sticky="w")

        pose_frame = ttk.LabelFrame(self.setup_tab, text="Pose Editor")
        pose_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=8)
        pose_frame.columnconfigure(1, weight=1)
        ttk.Label(pose_frame, text="Saved pose").grid(row=0, column=0, padx=8, pady=8, sticky="w")
        self.pose_combo = ttk.Combobox(pose_frame, textvariable=self.pose_var, state="readonly", values=[])
        self.pose_combo.grid(row=0, column=1, padx=8, pady=8, sticky="ew")
        self.pose_combo.bind("<<ComboboxSelected>>", lambda _event: self.load_selected_pose())
        pose_buttons = ttk.Frame(pose_frame)
        pose_buttons.grid(row=0, column=2, padx=8, pady=8, sticky="e")
        ttk.Button(pose_buttons, text="Load", command=self.load_selected_pose).pack(side="left", padx=4)
        ttk.Button(pose_buttons, text="Run Saved", command=self.run_selected_pose).pack(side="left", padx=4)
        ttk.Button(pose_buttons, text="Save to Flash", command=self.save_selected_pose).pack(side="left", padx=4)

        ttk.Button(pose_frame, text="Capture Robot -> Editor", command=self.capture_current_pose).grid(row=1, column=0, padx=8, pady=6, sticky="w")
        ttk.Button(pose_frame, text="Apply Editor Pose", command=self.apply_editor_pose).grid(row=1, column=1, padx=8, pady=6, sticky="w")

        self.pose_scale_container = ttk.Frame(pose_frame)
        self.pose_scale_container.grid(row=2, column=0, columnspan=3, sticky="nsew", padx=8, pady=8)
        self.pose_scale_container.columnconfigure(1, weight=1)
        for row_index, joint_name in enumerate(POSE_JOINTS):
            ttk.Label(self.pose_scale_container, text=JOINT_LABELS[joint_name]).grid(row=row_index, column=0, padx=6, pady=4, sticky="w")
            scale = self._make_themed_scale(
                self.pose_scale_container,
                self.pose_values[joint_name],
                from_=0,
                to=180,
                length=460,
            )
            scale.grid(row=row_index, column=1, padx=6, pady=4, sticky="ew")
            ttk.Spinbox(self.pose_scale_container, from_=0, to=180, textvariable=self.pose_values[joint_name], width=6).grid(row=row_index, column=2, padx=6, pady=4)

        sequence_frame = ttk.LabelFrame(self.setup_tab, text="Motion Sequences")
        sequence_frame.grid(row=1, column=1, rowspan=2, sticky="nsew", padx=8, pady=8)
        sequence_frame.columnconfigure(1, weight=1)
        sequence_rows = [
            ("Forward sequence", self.sequence_forward_var),
            ("Turn left sequence", self.sequence_left_var),
            ("Turn right sequence", self.sequence_right_var),
            ("Collision sequence", self.sequence_collision_var),
        ]
        for row_index, (label, variable) in enumerate(sequence_rows):
            ttk.Label(sequence_frame, text=label).grid(row=row_index, column=0, padx=8, pady=8, sticky="w")
            ttk.Entry(sequence_frame, textvariable=variable).grid(row=row_index, column=1, padx=8, pady=8, sticky="ew")

        timing_row = len(sequence_rows)
        ttk.Label(sequence_frame, text="Pose duration (ms)").grid(row=timing_row, column=0, padx=8, pady=8, sticky="w")
        ttk.Spinbox(sequence_frame, from_=0, to=5000, textvariable=self.pose_duration_var, width=8).grid(row=timing_row, column=1, padx=8, pady=8, sticky="w")
        ttk.Label(sequence_frame, text="Interpolation steps").grid(row=timing_row + 1, column=0, padx=8, pady=8, sticky="w")
        ttk.Spinbox(sequence_frame, from_=1, to=100, textvariable=self.interp_steps_var, width=8).grid(row=timing_row + 1, column=1, padx=8, pady=8, sticky="w")
        ttk.Label(sequence_frame, text="Hold per pose (ms)").grid(row=timing_row + 2, column=0, padx=8, pady=8, sticky="w")
        ttk.Spinbox(sequence_frame, from_=0, to=2000, textvariable=self.hold_ms_var, width=8).grid(row=timing_row + 2, column=1, padx=8, pady=8, sticky="w")
        ttk.Button(sequence_frame, text="Save Settings", command=self.save_settings).grid(row=timing_row + 3, column=0, padx=8, pady=12, sticky="w")
        ttk.Button(sequence_frame, text="Run Forward Once", command=self.run_forward_sequence_once).grid(row=timing_row + 3, column=1, padx=8, pady=12, sticky="w")

    def _build_joints_tab(self) -> None:
        self.joints_tab.columnconfigure(0, weight=1)
        self.joints_tab.columnconfigure(1, weight=1)
        joint_select = ttk.LabelFrame(self.joints_tab, text="Joint Selection")
        joint_select.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=8)
        joint_select.columnconfigure(1, weight=1)
        ttk.Label(joint_select, text="Joint").grid(row=0, column=0, padx=8, pady=8, sticky="w")
        self.joint_combo = ttk.Combobox(joint_select, textvariable=self.joint_var, state="readonly", values=[])
        self.joint_combo.grid(row=0, column=1, padx=8, pady=8, sticky="ew")
        self.joint_combo.bind("<<ComboboxSelected>>", lambda _event: self.load_selected_joint())
        ttk.Button(joint_select, text="Refresh joint", command=self.load_selected_joint).grid(row=0, column=2, padx=8, pady=8)
        ttk.Label(joint_select, textvariable=self.joint_info_var).grid(row=1, column=0, columnspan=3, padx=8, pady=(0, 8), sticky="w")

        move_frame = ttk.LabelFrame(self.joints_tab, text="Motion")
        move_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=8)
        move_frame.columnconfigure(1, weight=1)
        ttk.Label(move_frame, text="Move angle").grid(row=0, column=0, padx=8, pady=8, sticky="w")
        self._make_themed_scale(move_frame, self.joint_move_var, from_=-100, to=180, length=360).grid(row=0, column=1, padx=8, pady=8, sticky="ew")
        ttk.Spinbox(move_frame, from_=-100, to=180, textvariable=self.joint_move_var, width=8).grid(row=0, column=2, padx=8, pady=8)
        ttk.Label(move_frame, text="Nudge").grid(row=1, column=0, padx=8, pady=8, sticky="w")
        ttk.Spinbox(move_frame, from_=-180, to=180, textvariable=self.joint_nudge_var, width=8).grid(row=1, column=1, padx=8, pady=8, sticky="w")
        move_buttons = ttk.Frame(move_frame)
        move_buttons.grid(row=2, column=0, columnspan=3, padx=8, pady=8, sticky="w")
        for label, command in [
            ("Move", self.move_selected_joint),
            ("+", self.nudge_selected_joint_positive),
            ("-", self.nudge_selected_joint_negative),
            ("Home", self.home_selected_joint),
            ("Attach", self.attach_selected_joint),
            ("Detach", self.detach_selected_joint),
        ]:
            ttk.Button(move_buttons, text=label, command=command).pack(side="left", padx=4)

        settings = ttk.LabelFrame(self.joints_tab, text="Joint Settings")
        settings.grid(row=1, column=1, sticky="nsew", padx=8, pady=8)
        settings.columnconfigure(1, weight=1)
        fields = [
            ("Pin", self.joint_pin_var),
            ("Min", self.joint_min_var),
            ("Max", self.joint_max_var),
            ("Home", self.joint_home_var),
            ("Step", self.joint_step_var),
            ("Pulse min", self.joint_pulse_min_var),
            ("Pulse max", self.joint_pulse_max_var),
            ("Physical min", self.joint_physical_min_var),
            ("Physical max", self.joint_physical_max_var),
            ("Neutral output", self.joint_neutral_output_var),
            ("Stop deadband", self.joint_stop_deadband_var),
            ("Max speed scale", self.joint_max_speed_scale_var),
        ]
        for row_index, (label, variable) in enumerate(fields):
            ttk.Label(settings, text=label).grid(row=row_index, column=0, padx=8, pady=4, sticky="w")
            ttk.Spinbox(settings, from_=-360, to=4000, textvariable=variable, width=10).grid(row=row_index, column=1, padx=8, pady=4, sticky="w")

        bool_frame = ttk.Frame(settings)
        bool_frame.grid(row=len(fields), column=0, columnspan=2, padx=8, pady=8, sticky="w")
        ttk.Checkbutton(bool_frame, text="Invert servo", variable=self.joint_invert_var).pack(side="left", padx=4)
        ttk.Checkbutton(bool_frame, text="Invert controller input", variable=self.joint_input_invert_var).pack(side="left", padx=12)

        row = len(fields) + 1
        ttk.Label(settings, text="Control mode").grid(row=row, column=0, padx=8, pady=4, sticky="w")
        self.joint_control_mode_combo = ttk.Combobox(settings, textvariable=self.joint_control_mode_var, state="readonly", values=[name for name, _ in CONTROL_MODE_OPTIONS])
        self.joint_control_mode_combo.grid(row=row, column=1, padx=8, pady=4, sticky="ew")
        row += 1
        ttk.Label(settings, text="Axis source").grid(row=row, column=0, padx=8, pady=4, sticky="w")
        self.joint_axis_source_combo = ttk.Combobox(settings, textvariable=self.joint_axis_source_var, state="readonly", values=[name for name, _ in AXIS_SOURCE_OPTIONS])
        self.joint_axis_source_combo.grid(row=row, column=1, padx=8, pady=4, sticky="ew")
        row += 1
        ttk.Label(settings, text="Positive button").grid(row=row, column=0, padx=8, pady=4, sticky="w")
        self.joint_positive_button_combo = ttk.Combobox(settings, textvariable=self.joint_positive_button_var, state="readonly", values=[name for name, _ in BUTTON_OPTIONS])
        self.joint_positive_button_combo.grid(row=row, column=1, padx=8, pady=4, sticky="ew")
        row += 1
        ttk.Label(settings, text="Negative button").grid(row=row, column=0, padx=8, pady=4, sticky="w")
        self.joint_negative_button_combo = ttk.Combobox(settings, textvariable=self.joint_negative_button_var, state="readonly", values=[name for name, _ in BUTTON_OPTIONS])
        self.joint_negative_button_combo.grid(row=row, column=1, padx=8, pady=4, sticky="ew")
        row += 1
        ttk.Button(settings, text="Apply Joint Settings", command=self.apply_selected_joint_settings).grid(row=row, column=0, columnspan=2, padx=8, pady=12, sticky="w")

    def _build_wifi_tab(self) -> None:
        self.wifi_tab.columnconfigure(0, weight=1)
        frame = ttk.LabelFrame(self.wifi_tab, text="Wi-Fi")
        frame.pack(fill="both", expand=True, padx=8, pady=8)
        frame.columnconfigure(1, weight=1)
        ttk.Label(frame, textvariable=self.wifi_info_var, wraplength=1000, justify="left").grid(row=0, column=0, columnspan=3, padx=8, pady=8, sticky="w")
        rows = [
            ("Hostname", self.hostname_var),
            ("AP SSID", self.ap_ssid_var),
            ("AP password", self.ap_password_var),
            ("Station SSID", self.sta_ssid_var),
            ("Station password", self.sta_password_var),
        ]
        for row_index, (label, variable) in enumerate(rows, start=1):
            ttk.Label(frame, text=label).grid(row=row_index, column=0, padx=8, pady=8, sticky="w")
            show = "*" if "password" in label.lower() else None
            ttk.Entry(frame, textvariable=variable, show=show).grid(row=row_index, column=1, padx=8, pady=8, sticky="ew")
        buttons = ttk.Frame(frame)
        buttons.grid(row=len(rows) + 1, column=0, columnspan=3, padx=8, pady=12, sticky="w")
        ttk.Button(buttons, text="Save Wi-Fi Settings", command=self.save_wifi_settings).pack(side="left", padx=4)
        ttk.Button(buttons, text="Reconnect Wi-Fi", command=self.reconnect_wifi).pack(side="left", padx=4)
        ttk.Button(buttons, text="Reboot ESP32", command=self.reboot_robot).pack(side="left", padx=4)

    def _build_controller_tab(self) -> None:
        self.controller_tab.columnconfigure(0, weight=1)
        frame = ttk.LabelFrame(self.controller_tab, text="Wireless Controller")
        frame.pack(fill="both", expand=True, padx=8, pady=8)
        frame.columnconfigure(1, weight=1)
        ttk.Label(frame, textvariable=self.controller_info_var, wraplength=1000, justify="left").grid(row=0, column=0, columnspan=4, padx=8, pady=8, sticky="w")
        ttk.Checkbutton(frame, text="Controller enabled", variable=self.controller_enabled_var, command=self.apply_controller_enabled).grid(row=1, column=0, padx=8, pady=8, sticky="w")
        ttk.Checkbutton(frame, text="Pairing mode", variable=self.controller_pair_mode_var, command=self.apply_controller_pair_mode).grid(row=1, column=1, padx=8, pady=8, sticky="w")

        ttk.Label(frame, text="Home-all button").grid(row=2, column=0, padx=8, pady=8, sticky="w")
        self.controller_home_button_combo = ttk.Combobox(frame, textvariable=self.controller_home_button_var, state="readonly", values=[name for name, _ in BUTTON_OPTIONS])
        self.controller_home_button_combo.grid(row=2, column=1, padx=8, pady=8, sticky="ew")
        ttk.Button(frame, text="Apply home-all button", command=self.apply_controller_home_button).grid(row=2, column=2, padx=8, pady=8, sticky="w")

        ttk.Label(frame, text="LED RGB").grid(row=3, column=0, padx=8, pady=8, sticky="w")
        led_frame = ttk.Frame(frame)
        led_frame.grid(row=3, column=1, padx=8, pady=8, sticky="w")
        for variable in (self.controller_led_r_var, self.controller_led_g_var, self.controller_led_b_var):
            ttk.Spinbox(led_frame, from_=0, to=255, textvariable=variable, width=6).pack(side="left", padx=3)
        ttk.Button(frame, text="Apply LED", command=self.apply_controller_led).grid(row=3, column=2, padx=8, pady=8, sticky="w")

        ttk.Label(frame, text="Rumble").grid(row=4, column=0, padx=8, pady=8, sticky="w")
        rumble_frame = ttk.Frame(frame)
        rumble_frame.grid(row=4, column=1, padx=8, pady=8, sticky="w")
        ttk.Label(rumble_frame, text="Force").pack(side="left")
        ttk.Spinbox(rumble_frame, from_=0, to=255, textvariable=self.controller_rumble_force_var, width=6).pack(side="left", padx=4)
        ttk.Label(rumble_frame, text="Duration").pack(side="left")
        ttk.Spinbox(rumble_frame, from_=0, to=255, textvariable=self.controller_rumble_duration_var, width=6).pack(side="left", padx=4)
        ttk.Button(frame, text="Apply rumble", command=self.apply_controller_rumble).grid(row=4, column=2, padx=8, pady=8, sticky="w")

        ttk.Label(frame, text="Stick deadzone").grid(row=5, column=0, padx=8, pady=8, sticky="w")
        ttk.Spinbox(frame, from_=0, to=200, textvariable=self.controller_deadzone_var, width=8).grid(row=5, column=1, padx=8, pady=8, sticky="w")
        ttk.Button(frame, text="Apply deadzone", command=self.apply_controller_deadzone).grid(row=5, column=2, padx=8, pady=8, sticky="w")
        ttk.Button(frame, text="Capture stick centers", command=self.calibrate_controller_centers).grid(row=5, column=3, padx=8, pady=8, sticky="w")

        recovery = ttk.Frame(frame)
        recovery.grid(row=6, column=0, columnspan=4, padx=8, pady=12, sticky="w")
        for label, command in [
            ("Remember current", self.controller_remember_current),
            ("Disconnect", self.controller_disconnect),
            ("Forget remembered", self.controller_forget_target),
            ("Forget all", self.controller_forget_all),
        ]:
            ttk.Button(recovery, text=label, command=command).pack(side="left", padx=4)

    def _build_state_tab(self) -> None:
        self.state_tab.columnconfigure(0, weight=1)
        self.state_tab.rowconfigure(0, weight=1)
        frame = ttk.LabelFrame(self.state_tab, text="Raw State")
        frame.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        self.state_text = tk.Text(frame, wrap="none", bg=INPUT_BG, fg=PALETTE_WHITE, insertbackground=PALETTE_WHITE, selectbackground=PALETTE_YELLOW, selectforeground=PALETTE_BLACK, relief="flat", bd=0)
        self.state_text.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        ttk.Button(frame, text="Refresh State", command=self.refresh_state).grid(row=1, column=0, padx=8, pady=(0, 8), sticky="w")

    def _build_vision_tab(self) -> None:
        self.vision_tab.columnconfigure(0, weight=2)
        self.vision_tab.columnconfigure(1, weight=1)
        self.vision_tab.rowconfigure(1, weight=1)

        controls = ttk.LabelFrame(self.vision_tab, text="Vision Controls")
        controls.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=8)
        controls.columnconfigure(1, weight=1)
        controls.columnconfigure(3, weight=1)
        controls.columnconfigure(5, weight=1)

        ttk.Label(controls, text="Camera URL").grid(row=0, column=0, padx=8, pady=8, sticky="w")
        ttk.Entry(controls, textvariable=self.camera_url_var).grid(row=0, column=1, columnspan=3, padx=8, pady=8, sticky="ew")
        ttk.Button(controls, text="Start Feed", command=self.start_feed).grid(row=0, column=4, padx=6, pady=8)
        ttk.Button(controls, text="Stop Feed", command=self.stop_feed).grid(row=0, column=5, padx=6, pady=8, sticky="w")

        ttk.Label(controls, text="Robot marker").grid(row=1, column=0, padx=8, pady=8, sticky="w")
        ttk.Combobox(controls, textvariable=self.robot_color_var, state="readonly", values=sorted(ROBOT_COLOR_RANGES.keys())).grid(row=1, column=1, padx=8, pady=8, sticky="ew")
        ttk.Label(controls, text="Ball color").grid(row=1, column=2, padx=8, pady=8, sticky="w")
        ttk.Combobox(controls, textvariable=self.ball_color_var, state="readonly", values=sorted(BALL_COLOR_RANGES.keys())).grid(row=1, column=3, padx=8, pady=8, sticky="ew")
        ttk.Label(controls, text="Deadband").grid(row=1, column=4, padx=8, pady=8, sticky="w")
        ttk.Spinbox(controls, from_=5, to=300, textvariable=self.alignment_deadband_var, width=8).grid(row=1, column=5, padx=8, pady=8, sticky="w")

        ttk.Label(controls, text="Collision cm").grid(row=2, column=0, padx=8, pady=8, sticky="w")
        ttk.Spinbox(controls, from_=1.0, to=100.0, increment=1.0, textvariable=self.collision_distance_var, width=8).grid(row=2, column=1, padx=8, pady=8, sticky="w")

        ttk.Label(controls, text="Ball cm").grid(row=2, column=2, padx=8, pady=8, sticky="w")
        ttk.Spinbox(controls, from_=1.0, to=50.0, increment=0.5, textvariable=self.ball_diameter_var, width=8).grid(row=2, column=3, padx=8, pady=8, sticky="w")
        ttk.Label(controls, text="Min robot").grid(row=2, column=4, padx=8, pady=8, sticky="w")
        ttk.Spinbox(controls, from_=100, to=20000, increment=100, textvariable=self.min_robot_area_var, width=8).grid(row=2, column=5, padx=8, pady=8, sticky="w")

        ttk.Label(controls, text="Min ball").grid(row=3, column=0, padx=8, pady=8, sticky="w")
        ttk.Spinbox(controls, from_=20, to=10000, increment=10, textvariable=self.min_ball_area_var, width=8).grid(row=3, column=1, padx=8, pady=8, sticky="w")
        ttk.Label(controls, text="Pause ms").grid(row=3, column=2, padx=8, pady=8, sticky="w")
        ttk.Spinbox(controls, from_=0, to=5000, increment=50, textvariable=self.pause_between_actions_var, width=8).grid(row=3, column=3, padx=8, pady=8, sticky="w")

        action_frame = ttk.Frame(controls)
        action_frame.grid(row=3, column=4, columnspan=2, padx=8, pady=(4, 8), sticky="ew")
        ttk.Button(action_frame, text="Move Forward Once", command=self.run_forward_sequence_once).pack(side="left", padx=4)
        ttk.Button(action_frame, text="Start Autonomous", command=self.start_autonomy).pack(side="left", padx=4)
        ttk.Button(action_frame, text="Stop Autonomous", command=self.stop_autonomy).pack(side="left", padx=4)
        ttk.Button(action_frame, text="Export Last Video", command=self.export_last_autonomy_video).pack(side="left", padx=4)
        ttk.Button(action_frame, text="Save Settings", command=self.save_settings).pack(side="left", padx=4)
        ttk.Label(action_frame, textvariable=self.feed_status_var).pack(side="left", padx=12)

        video_frame = ttk.LabelFrame(self.vision_tab, text="Live Camera Feed")
        video_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=8)
        video_frame.columnconfigure(0, weight=1)
        video_frame.rowconfigure(0, weight=1)
        self.video_label = ttk.Label(video_frame, anchor="center")
        self.video_label.grid(row=0, column=0, sticky="nsew")
        self.video_label.bind("<Configure>", self._on_video_label_resize)

        side_frame = ttk.LabelFrame(self.vision_tab, text="Detection & Motion")
        side_frame.grid(row=1, column=1, sticky="nsew", padx=8, pady=8)
        side_frame.columnconfigure(0, weight=1)
        self.info_label = ttk.Label(side_frame, textvariable=self.info_var, wraplength=360, justify="left")
        self.info_label.grid(row=0, column=0, padx=8, pady=8, sticky="ew")
        side_frame.bind("<Configure>", self._on_detection_panel_resize)

        ttk.Label(side_frame, text="Forward sequence").grid(row=1, column=0, padx=8, pady=(12, 2), sticky="w")
        ttk.Entry(side_frame, textvariable=self.sequence_forward_var).grid(row=2, column=0, padx=8, pady=4, sticky="ew")
        ttk.Label(side_frame, text="Turn left sequence").grid(row=3, column=0, padx=8, pady=(12, 2), sticky="w")
        ttk.Entry(side_frame, textvariable=self.sequence_left_var).grid(row=4, column=0, padx=8, pady=4, sticky="ew")
        ttk.Label(side_frame, text="Turn right sequence").grid(row=5, column=0, padx=8, pady=(12, 2), sticky="w")
        ttk.Entry(side_frame, textvariable=self.sequence_right_var).grid(row=6, column=0, padx=8, pady=4, sticky="ew")
        ttk.Label(side_frame, text="Collision sequence (optional)").grid(row=7, column=0, padx=8, pady=(12, 2), sticky="w")
        ttk.Entry(side_frame, textvariable=self.sequence_collision_var).grid(row=8, column=0, padx=8, pady=4, sticky="ew")

        ttk.Label(side_frame, text="Action log").grid(row=9, column=0, padx=8, pady=(12, 2), sticky="w")
        self.log_text = tk.Text(side_frame, height=16, wrap="word", bg=INPUT_BG, fg=PALETTE_WHITE, insertbackground=PALETTE_WHITE, selectbackground=PALETTE_YELLOW, selectforeground=PALETTE_BLACK, relief="flat", bd=0)
        self.log_text.grid(row=10, column=0, padx=8, pady=8, sticky="nsew")
        side_frame.rowconfigure(10, weight=1)

    def log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {message}\n"
        self.log_text.insert("end", line)
        self.log_text.see("end")

    def save_settings(self) -> None:
        self.config.update(
            {
                "base_url": self.base_url_var.get().strip(),
                "camera_url": self.camera_url_var.get().strip(),
                "robot_marker_color": self.robot_color_var.get(),
                "ball_color": self.ball_color_var.get(),
                "alignment_deadband_px": int(self.alignment_deadband_var.get()),
                "collision_distance_cm": float(self.collision_distance_var.get()),
                "ball_diameter_cm": float(self.ball_diameter_var.get()),
                "min_robot_area_px": int(self.min_robot_area_var.get()),
                "min_ball_area_px": int(self.min_ball_area_var.get()),
                "pose_duration_ms": int(self.pose_duration_var.get()),
                "interp_steps": int(self.interp_steps_var.get()),
                "hold_ms": int(self.hold_ms_var.get()),
                "pause_between_actions_ms": int(self.pause_between_actions_var.get()),
                "forward_sequence": self.sequence_forward_var.get().strip(),
                "turn_left_sequence": self.sequence_left_var.get().strip(),
                "turn_right_sequence": self.sequence_right_var.get().strip(),
                "collision_sequence": self.sequence_collision_var.get().strip(),
            }
        )
        save_config(self.config)
        self.log("Settings saved.")

    def _settings_snapshot(self) -> dict:
        return {
            "camera_url": self.camera_url_var.get().strip(),
            "robot_marker_color": self.robot_color_var.get(),
            "ball_color": self.ball_color_var.get(),
            "alignment_deadband_px": int(self.alignment_deadband_var.get()),
            "collision_distance_cm": float(self.collision_distance_var.get()),
            "ball_diameter_cm": float(self.ball_diameter_var.get()),
            "min_robot_area_px": int(self.min_robot_area_var.get()),
            "min_ball_area_px": int(self.min_ball_area_var.get()),
            "pose_duration_ms": int(self.pose_duration_var.get()),
            "interp_steps": int(self.interp_steps_var.get()),
            "hold_ms": int(self.hold_ms_var.get()),
            "pause_between_actions_ms": int(self.pause_between_actions_var.get()),
            "forward_sequence": self.sequence_forward_var.get().strip(),
            "turn_left_sequence": self.sequence_left_var.get().strip(),
            "turn_right_sequence": self.sequence_right_var.get().strip(),
            "collision_sequence": self.sequence_collision_var.get().strip(),
        }

    def connect_robot(self) -> None:
        try:
            self.client.set_base_url(self.base_url_var.get().strip())
            self.current_state = self.client.get_state()
            self._refresh_ui_from_state(self.current_state)
            self.status_var.set(f"Connected: {self.client.base_url}")
            self.save_settings()
            self.log(f"Connected to {self.client.base_url}")
        except Exception as exc:
            self.status_var.set(f"Connection failed: {exc}")
            messagebox.showerror("Connection failed", str(exc))

    def auto_connect_robot(self) -> None:
        try:
            self.status_var.set("Searching for biped robot...")
            resolved = discover_biped_endpoint(self.base_url_var.get().strip(), APP_ROOT, allow_subnet_scan=True)
            if resolved is None:
                self.status_var.set("Auto discovery failed. Enter URL manually.")
                self.log("Auto discovery failed.")
                return
            self.base_url_var.set(resolved.base_url)
            self.client.set_base_url(resolved.base_url)
            self.current_state = self.client.get_state()
            self._refresh_ui_from_state(self.current_state)
            self.status_var.set(f"Connected: {resolved.base_url} (via {resolved.source})")
            self.save_settings()
            self.log(f"Auto-connected to {resolved.base_url} via {resolved.source}.")
        except Exception as exc:
            self.status_var.set(f"Auto discovery failed: {exc}")
            self.log(f"Auto discovery error: {exc}")

    def refresh_state(self) -> None:
        try:
            self.current_state = self.client.get_state()
            self._refresh_ui_from_state(self.current_state)
            self.status_var.set(f"Connected: {self.client.base_url}")
            self.log("Robot state refreshed.")
        except Exception as exc:
            self.status_var.set(f"Refresh failed: {exc}")
            messagebox.showerror("Refresh failed", str(exc))

    def run_stand(self) -> None:
        try:
            self.client.stand(self.pose_duration_var.get(), self.interp_steps_var.get(), self.hold_ms_var.get())
            self.log("Stand command sent.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Stand failed", str(exc))

    def save_flash(self) -> None:
        try:
            self.client.system_action("save")
            self.log("Saved current settings to flash.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))

    def reload_flash(self) -> None:
        try:
            self.client.system_action("load")
            self.log("Reloaded settings from flash.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Reload failed", str(exc))

    def _refresh_ui_from_state(self, state: dict) -> None:
        self._load_pose_map_from_state(state)
        self._load_joint_map_from_state(state)
        self._load_wifi_from_state(state)
        self._load_controller_from_state(state)
        self._load_raw_state(state)
        wifi = state.get("wifi", {}) or {}
        device_type = state.get("device_type", "unknown")
        firmware = state.get("firmware_version", "unknown")
        self.system_info_var.set(
            f"Device type: {device_type} | Firmware: {firmware} | "
            f"STA IP: {wifi.get('sta_ip') or 'none'} | AP SSID: {wifi.get('ap_ssid') or 'n/a'}"
        )

    def _load_pose_map_from_state(self, state: dict) -> None:
        biped = state.get("biped", {}) or {}
        poses = biped.get("poses", [])
        self.pose_map = {
            str(entry.get("name", "")).strip(): {str(k): int(v) for k, v in (entry.get("angles", {}) or {}).items()}
            for entry in poses
            if str(entry.get("name", "")).strip()
        }
        pose_names = sorted(self.pose_map.keys())
        self.pose_combo["values"] = pose_names
        if pose_names:
            current = self.pose_var.get()
            if current not in pose_names:
                self.pose_var.set(pose_names[0])
            self.load_selected_pose()

    def _load_joint_map_from_state(self, state: dict) -> None:
        joints = state.get("joints", []) or []
        self.joint_state_map = {str(entry.get("name")): entry for entry in joints if entry.get("name")}
        joint_names = list(self.joint_state_map.keys())
        self.joint_combo["values"] = joint_names
        if joint_names:
            current = self.joint_var.get()
            if current not in joint_names:
                self.joint_var.set(joint_names[0])
            self.load_selected_joint()

    def _load_wifi_from_state(self, state: dict) -> None:
        wifi = state.get("wifi", {}) or {}
        self.hostname_var.set(str(wifi.get("hostname", "")))
        self.ap_ssid_var.set(str(wifi.get("ap_ssid", "")))
        self.sta_ssid_var.set(str(wifi.get("sta_ssid", "")))
        self.wifi_info_var.set(
            f"AP: {wifi.get('ap_ssid')} @ {wifi.get('ap_ip')} | "
            f"STA: {wifi.get('sta_ssid') or 'not set'} | Connected: {wifi.get('sta_connected')} | "
            f"STA IP: {wifi.get('sta_ip') or 'none'} | mDNS: {wifi.get('mdns_hostname') or 'inactive'} | "
            f"Last result: {wifi.get('last_result') or 'unknown'} | Last failure: {wifi.get('last_failure') or 'none'}"
        )

    def _load_controller_from_state(self, state: dict) -> None:
        controller = state.get("ps4", {}) or {}
        self.controller_enabled_var.set(bool(controller.get("enabled", False)))
        self.controller_pair_mode_var.set(bool(controller.get("allow_new_connections", False)))
        self.controller_home_button_var.set(str(controller.get("home_all_button", "none")))
        self.controller_led_r_var.set(int(controller.get("led_r", 0)))
        self.controller_led_g_var.set(int(controller.get("led_g", 0)))
        self.controller_led_b_var.set(int(controller.get("led_b", 255)))
        self.controller_rumble_force_var.set(int(controller.get("rumble_force", 0)))
        self.controller_rumble_duration_var.set(int(controller.get("rumble_duration", 0)))
        self.controller_deadzone_var.set(int(controller.get("axis_deadzone", 48)))
        self.controller_info_var.set(
            f"State: {controller.get('state')} | Connected: {controller.get('connected')} | "
            f"Battery: {controller.get('battery')}% | Pair mode: {controller.get('allow_new_connections')} | "
            f"Current: {controller.get('controller_name') or 'none'} | "
            f"Remembered: {controller.get('remembered_name') or 'none'} | "
            f"ESP32 BT MAC: {controller.get('esp32_bt_mac') or 'n/a'} | "
            f"Last error: {controller.get('last_error') or 'none'}"
        )

    def _load_raw_state(self, state: dict) -> None:
        payload = json.dumps(state, indent=2)
        self.state_text.delete("1.0", "end")
        self.state_text.insert("1.0", payload)

    def load_selected_pose(self) -> None:
        pose_name = self.pose_var.get()
        pose = self.pose_map.get(pose_name)
        if not pose:
            return
        for joint_name in POSE_JOINTS:
            self.pose_values[joint_name].set(int(pose.get(joint_name, self.pose_values[joint_name].get())))
        self.log(f"Loaded pose `{pose_name}` into editor.")

    def capture_current_pose(self) -> None:
        try:
            state = self.client.get_state()
            joints = {str(entry.get("name")): int(entry.get("position", 90)) for entry in state.get("joints", [])}
            for joint_name in POSE_JOINTS:
                if joint_name in joints:
                    self.pose_values[joint_name].set(joints[joint_name])
            self.log("Captured current robot angles into editor.")
        except Exception as exc:
            messagebox.showerror("Capture failed", str(exc))

    def current_editor_pose(self) -> dict[str, int]:
        return {joint_name: int(var.get()) for joint_name, var in self.pose_values.items()}

    def apply_editor_pose(self) -> None:
        try:
            self.client.move_pose(self.current_editor_pose())
            self.log("Applied editor pose to robot.")
        except Exception as exc:
            messagebox.showerror("Apply failed", str(exc))

    def save_selected_pose(self) -> None:
        pose_name = self.pose_var.get().strip()
        if not pose_name:
            messagebox.showerror("Missing pose", "Select a saved firmware pose name first.")
            return
        try:
            pose = self.current_editor_pose()
            self.client.save_pose(pose_name, pose)
            self.pose_map[pose_name] = dict(pose)
            self.log(f"Saved `{pose_name}` to robot flash.")
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))

    def run_selected_pose(self) -> None:
        pose_name = self.pose_var.get().strip()
        if not pose_name:
            return
        try:
            self.client.run_pose(
                pose_name,
                self.pose_duration_var.get(),
                self.interp_steps_var.get(),
                self.hold_ms_var.get(),
            )
            self.log(f"Ran saved pose `{pose_name}`.")
        except Exception as exc:
            messagebox.showerror("Run pose failed", str(exc))

    def run_forward_sequence_once(self) -> None:
        sequence = self.sequence_forward_var.get().strip()
        if not sequence:
            messagebox.showerror("Missing sequence", "Forward sequence is empty.")
            return
        try:
            self.client.play_sequence(
                sequence,
                self.pose_duration_var.get(),
                self.interp_steps_var.get(),
                self.hold_ms_var.get(),
                repeat_count=1,
            )
            self.log(f"Ran forward sequence: {sequence}")
        except Exception as exc:
            messagebox.showerror("Sequence failed", str(exc))

    def load_selected_joint(self) -> None:
        joint_name = self.joint_var.get().strip()
        joint = self.joint_state_map.get(joint_name)
        if not joint:
            return
        self.joint_pin_var.set(int(joint.get("pin", 0)))
        self.joint_min_var.set(int(joint.get("min_angle", 0)))
        self.joint_max_var.set(int(joint.get("max_angle", 180)))
        self.joint_home_var.set(int(joint.get("home_angle", 90)))
        self.joint_step_var.set(int(joint.get("step", 2)))
        self.joint_pulse_min_var.set(int(joint.get("pulse_min", 500)))
        self.joint_pulse_max_var.set(int(joint.get("pulse_max", 2400)))
        self.joint_physical_min_var.set(int(joint.get("physical_min_angle", joint.get("min_angle", 0))))
        self.joint_physical_max_var.set(int(joint.get("physical_max_angle", joint.get("max_angle", 180))))
        self.joint_neutral_output_var.set(int(joint.get("neutral_output", 90)))
        self.joint_stop_deadband_var.set(int(joint.get("stop_deadband", 3)))
        self.joint_max_speed_scale_var.set(int(joint.get("max_speed_scale", 100)))
        self.joint_invert_var.set(bool(joint.get("invert", False)))
        self.joint_input_invert_var.set(bool(joint.get("input_invert", False)))
        self.joint_move_var.set(int(joint.get("position", 90)))
        self.joint_nudge_var.set(int(joint.get("step", 2)))
        self.joint_control_mode_var.set(str(joint.get("control_mode", "none")))
        self.joint_axis_source_var.set(str(joint.get("axis_source", "none")))
        self.joint_positive_button_var.set(str(joint.get("positive_button", "none")))
        self.joint_negative_button_var.set(str(joint.get("negative_button", "none")))
        label = str(joint.get("label", joint_name))
        self.joint_info_var.set(
            f"{label} | position={joint.get('position')} | physical={joint.get('physical_angle')} | "
            f"attached={joint.get('attached')} | stored min/max={joint.get('stored_min_angle')} / {joint.get('stored_max_angle')}"
        )

    def _selected_joint_name(self) -> str:
        joint_name = self.joint_var.get().strip()
        if not joint_name:
            raise RuntimeError("No joint selected.")
        return joint_name

    def move_selected_joint(self) -> None:
        try:
            self.client.joint_action(self._selected_joint_name(), "move", value=int(self.joint_move_var.get()))
            self.log(f"Moved {self.joint_var.get()} to {self.joint_move_var.get()}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Move failed", str(exc))

    def nudge_selected_joint_positive(self) -> None:
        try:
            self.client.joint_action(self._selected_joint_name(), "nudge", value=int(abs(self.joint_nudge_var.get())))
            self.log(f"Nudged {self.joint_var.get()} +{abs(self.joint_nudge_var.get())}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Nudge failed", str(exc))

    def nudge_selected_joint_negative(self) -> None:
        try:
            self.client.joint_action(self._selected_joint_name(), "nudge", value=-int(abs(self.joint_nudge_var.get())))
            self.log(f"Nudged {self.joint_var.get()} -{abs(self.joint_nudge_var.get())}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Nudge failed", str(exc))

    def home_selected_joint(self) -> None:
        try:
            self.client.joint_action(self._selected_joint_name(), "home")
            self.log(f"Homed {self.joint_var.get()}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Home failed", str(exc))

    def attach_selected_joint(self) -> None:
        try:
            self.client.joint_action(self._selected_joint_name(), "attach")
            self.log(f"Attached {self.joint_var.get()}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Attach failed", str(exc))

    def detach_selected_joint(self) -> None:
        try:
            self.client.joint_action(self._selected_joint_name(), "detach")
            self.log(f"Detached {self.joint_var.get()}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Detach failed", str(exc))

    def apply_selected_joint_settings(self) -> None:
        try:
            joint_name = self._selected_joint_name()
            self.client.joint_action(
                joint_name,
                "apply",
                pin=int(self.joint_pin_var.get()),
                motor_type=0,
                min=int(self.joint_min_var.get()),
                max=int(self.joint_max_var.get()),
                home=int(self.joint_home_var.get()),
                step=int(self.joint_step_var.get()),
                pulse_min=int(self.joint_pulse_min_var.get()),
                pulse_max=int(self.joint_pulse_max_var.get()),
                physical_min_angle=int(self.joint_physical_min_var.get()),
                physical_max_angle=int(self.joint_physical_max_var.get()),
                neutral_output=int(self.joint_neutral_output_var.get()),
                stop_deadband=int(self.joint_stop_deadband_var.get()),
                max_speed_scale=int(self.joint_max_speed_scale_var.get()),
                invert=1 if self.joint_invert_var.get() else 0,
                control_mode=next(value for name, value in CONTROL_MODE_OPTIONS if name == self.joint_control_mode_var.get()),
                axis_source=next(value for name, value in AXIS_SOURCE_OPTIONS if name == self.joint_axis_source_var.get()),
                positive_button=next(value for name, value in BUTTON_OPTIONS if name == self.joint_positive_button_var.get()),
                negative_button=next(value for name, value in BUTTON_OPTIONS if name == self.joint_negative_button_var.get()),
                input_invert=1 if self.joint_input_invert_var.get() else 0,
            )
            self.log(f"Applied settings for {joint_name}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Apply failed", str(exc))

    def save_wifi_settings(self) -> None:
        try:
            self.client.wifi_set(
                hostname=self.hostname_var.get().strip(),
                ap_ssid=self.ap_ssid_var.get().strip(),
                ap_password=self.ap_password_var.get(),
                sta_ssid=self.sta_ssid_var.get().strip(),
                sta_password=self.sta_password_var.get(),
            )
            self.log("Saved Wi-Fi settings.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Wi-Fi save failed", str(exc))

    def reconnect_wifi(self) -> None:
        try:
            self.client.wifi_reconnect()
            self.log("Requested Wi-Fi reconnect.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Reconnect failed", str(exc))

    def reboot_robot(self) -> None:
        try:
            self.client.system_action("reboot")
            self.log("Reboot requested.")
            self.status_var.set("Reboot requested. Wait a few seconds, then reconnect.")
        except Exception as exc:
            messagebox.showerror("Reboot failed", str(exc))

    def apply_controller_enabled(self) -> None:
        try:
            self.client.ps4_action("enable", value=1 if self.controller_enabled_var.get() else 0)
            self.log(f"Controller enabled set to {self.controller_enabled_var.get()}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Controller update failed", str(exc))

    def apply_controller_pair_mode(self) -> None:
        try:
            self.client.ps4_action("pair_mode", value=1 if self.controller_pair_mode_var.get() else 0)
            self.log(f"Pair mode set to {self.controller_pair_mode_var.get()}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Pair mode update failed", str(exc))

    def apply_controller_home_button(self) -> None:
        try:
            value = next(v for name, v in BUTTON_OPTIONS if name == self.controller_home_button_var.get())
            self.client.ps4_action("home_button", value=value)
            self.log(f"Applied home-all button: {self.controller_home_button_var.get()}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Home button update failed", str(exc))

    def apply_controller_led(self) -> None:
        try:
            self.client.ps4_action(
                "led",
                r=int(self.controller_led_r_var.get()),
                g=int(self.controller_led_g_var.get()),
                b=int(self.controller_led_b_var.get()),
            )
            self.log("Applied controller LED color.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("LED update failed", str(exc))

    def apply_controller_rumble(self) -> None:
        try:
            self.client.ps4_action(
                "rumble",
                force=int(self.controller_rumble_force_var.get()),
                duration=int(self.controller_rumble_duration_var.get()),
            )
            self.log("Applied controller rumble.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Rumble failed", str(exc))

    def apply_controller_deadzone(self) -> None:
        try:
            self.client.ps4_action("deadzone", value=int(self.controller_deadzone_var.get()))
            self.log(f"Applied deadzone: {self.controller_deadzone_var.get()}.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Deadzone update failed", str(exc))

    def calibrate_controller_centers(self) -> None:
        try:
            self.client.ps4_action("calibrate_center")
            self.log("Captured stick centers.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Calibration failed", str(exc))

    def controller_remember_current(self) -> None:
        try:
            self.client.ps4_action("remember_current")
            self.log("Remembered current controller.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Remember failed", str(exc))

    def controller_disconnect(self) -> None:
        try:
            self.client.ps4_action("disconnect")
            self.log("Disconnected current controller.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Disconnect failed", str(exc))

    def controller_forget_target(self) -> None:
        try:
            self.client.ps4_action("forget_target")
            self.log("Forgot remembered controller.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Forget failed", str(exc))

    def controller_forget_all(self) -> None:
        try:
            self.client.ps4_action("forget")
            self.log("Cleared all controller bond data.")
            self.refresh_state()
        except Exception as exc:
            messagebox.showerror("Forget-all failed", str(exc))

    def start_feed(self) -> None:
        self.save_settings()
        self.active_vision_settings = self._settings_snapshot()
        self.feed_enabled = True
        self.feed_status_var.set("Feed running")
        if not self.worker_thread or not self.worker_thread.is_alive():
            self.stop_event.clear()
            self.worker_thread = threading.Thread(target=self._vision_loop, daemon=True)
            self.worker_thread.start()
        self.log("Vision feed started.")

    def stop_feed(self) -> None:
        self.feed_enabled = False
        self.autonomy_enabled = False
        self.feed_status_var.set("Feed stopped")
        self.log("Vision feed stopped.")

    def start_autonomy(self) -> None:
        self.save_settings()
        self.active_vision_settings = self._settings_snapshot()
        self.feed_enabled = True
        self.autonomy_enabled = True
        with self.recording_lock:
            self.recorded_autonomy_frames = []
        self.feed_status_var.set("Autonomous pursuit running")
        if not self.worker_thread or not self.worker_thread.is_alive():
            self.stop_event.clear()
            self.worker_thread = threading.Thread(target=self._vision_loop, daemon=True)
            self.worker_thread.start()
        self.log("Autonomous pursuit started. Recording is active for this run.")

    def stop_autonomy(self) -> None:
        self.autonomy_enabled = False
        with self.recording_lock:
            if self.recorded_autonomy_frames:
                self.last_recorded_autonomy_frames = [frame.copy() for frame in self.recorded_autonomy_frames]
        if self.feed_enabled:
            self.feed_status_var.set("Feed running")
        else:
            self.feed_status_var.set("Feed stopped")
        self.log("Autonomous pursuit stopped.")

    def export_last_autonomy_video(self) -> None:
        with self.recording_lock:
            if self.autonomy_enabled and self.recorded_autonomy_frames:
                frames = [frame.copy() for frame in self.recorded_autonomy_frames]
            else:
                frames = [frame.copy() for frame in self.last_recorded_autonomy_frames]
        if not frames:
            messagebox.showinfo("No recording", "No autonomous recording is available to export yet.")
            return
        try:
            VISION_RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = VISION_RECORDINGS_DIR / f"autonomy_{timestamp}.mp4"
            height, width = frames[0].shape[:2]
            writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), 8.0, (width, height))
            if not writer.isOpened():
                raise RuntimeError("Failed to open video writer.")
            try:
                for frame in frames:
                    writer.write(frame)
            finally:
                writer.release()
            self.last_exported_video_path = output_path
            self.log(f"Exported autonomous recording to {output_path}")
            messagebox.showinfo("Video exported", f"Saved to:\n{output_path}")
        except Exception as exc:
            messagebox.showerror("Export failed", str(exc))

    def _tracker_from_settings(self, settings: dict) -> RobotBallTracker:
        return RobotBallTracker(
            TrackingConfig(
                robot_marker_color=settings["robot_marker_color"],
                ball_color=settings["ball_color"],
                ball_diameter_cm=settings["ball_diameter_cm"],
                kick_distance_cm=settings["collision_distance_cm"],
                approach_distance_cm=9999.0,
                alignment_deadband_px=settings["alignment_deadband_px"],
                min_robot_area_px=settings["min_robot_area_px"],
                min_ball_area_px=settings["min_ball_area_px"],
            )
        )

    def _decide_action(self, info: dict, settings: dict) -> tuple[str | None, str]:
        if not info.get("robot_detected") and not info.get("ball_detected"):
            return None, "Searching for robot and ball."
        if not info.get("robot_detected"):
            return None, "Robot not detected."
        if not info.get("ball_detected"):
            return None, "Ball not detected."
        distance_cm = info.get("distance_cm")
        if isinstance(distance_cm, (int, float)) and distance_cm <= settings["collision_distance_cm"]:
            sequence = settings["collision_sequence"].strip()
            return ("collision", sequence) if sequence else ("collision", "")
        return "forward", settings["forward_sequence"].strip()

    def _execute_sequence_action(self, action_name: str, sequence: str, settings: dict) -> str:
        if action_name == "collision":
            if sequence:
                self.client.play_sequence(
                    sequence,
                    settings["pose_duration_ms"],
                    settings["interp_steps"],
                    settings["hold_ms"],
                    repeat_count=1,
                )
                return f"Collision detected. Ran collision sequence: {sequence}"
            return "Collision detected. Robot stopped."
        if not sequence:
            return f"No sequence configured for {action_name}."
        self.client.play_sequence(
            sequence,
            settings["pose_duration_ms"],
            settings["interp_steps"],
            settings["hold_ms"],
            repeat_count=1,
        )
        return f"Executed {action_name}: {sequence}"

    def _vision_loop(self) -> None:
        next_action_time = 0.0
        while not self.stop_event.is_set():
            if not self.feed_enabled and not self.autonomy_enabled:
                time.sleep(0.1)
                continue

            settings = dict(self.active_vision_settings or self._settings_snapshot())
            try:
                frame = fetch_ip_webcam_frame(settings["camera_url"])
                tracker = self._tracker_from_settings(settings)
                annotated, info = tracker.process_frame(frame)
                if self.autonomy_enabled:
                    with self.recording_lock:
                        self.recorded_autonomy_frames.append(annotated.copy())
                self.last_frame_info = info
                action_text = "Feed only"

                now = time.time()
                if self.autonomy_enabled and now >= next_action_time:
                    action_name, sequence = self._decide_action(info, settings)
                    if action_name:
                        action_text = self._execute_sequence_action(action_name, sequence, settings)
                        if action_name == "collision":
                            with self.recording_lock:
                                if self.recorded_autonomy_frames:
                                    self.last_recorded_autonomy_frames = [frame.copy() for frame in self.recorded_autonomy_frames]
                            self.autonomy_enabled = False
                        next_action_time = time.time() + max(0, settings["pause_between_actions_ms"]) / 1000.0
                    else:
                        action_text = sequence

                rgb = annotated[:, :, ::-1]
                while not self.image_queue.empty():
                    try:
                        self.image_queue.get_nowait()
                    except Empty:
                        break
                self.image_queue.put_nowait(VisionFrame(rgb_image=rgb, info=info, action_text=action_text))
            except Exception as exc:
                while not self.image_queue.empty():
                    try:
                        self.image_queue.get_nowait()
                    except Empty:
                        break
                fallback_info = {
                    "robot_detected": False,
                    "ball_detected": False,
                    "state": "ERROR",
                    "distance_cm": None,
                    "dx_px": None,
                }
                self.image_queue.put_nowait(VisionFrame(rgb_image=None, info=fallback_info, action_text=f"Vision error: {exc}"))
                time.sleep(0.5)

    def _poll_image_queue(self) -> None:
        try:
            while True:
                frame = self.image_queue.get_nowait()
                self._render_frame(frame)
        except Empty:
            pass
        self.root.after(100, self._poll_image_queue)

    def _on_detection_panel_resize(self, event: tk.Event) -> None:
        wraplength = max(240, event.width - 32)
        self.info_label.configure(wraplength=wraplength)

    def _on_video_label_resize(self, _event: tk.Event) -> None:
        self._update_video_preview()

    def _update_video_preview(self) -> None:
        if self._last_video_rgb_image is None:
            return
        width = max(1, self.video_label.winfo_width() - 12)
        height = max(1, self.video_label.winfo_height() - 12)
        if width < 32 or height < 32:
            return
        image = Image.fromarray(self._last_video_rgb_image)
        image.thumbnail((width, height), Image.Resampling.LANCZOS)
        photo = ImageTk.PhotoImage(image)
        self.current_photo = photo
        self.video_label.configure(image=photo)

    def _render_frame(self, frame: VisionFrame) -> None:
        info = frame.info or {}
        lines = [
            f"State: {info.get('state')}",
            f"Robot detected: {info.get('robot_detected')}",
            f"Ball detected: {info.get('ball_detected')}",
            f"Distance (cm): {info.get('distance_cm')}",
            f"dx (px): {info.get('dx_px')}",
            f"Action: {frame.action_text}",
        ]
        self.info_var.set("\n".join(lines))
        if frame.action_text != self.last_logged_action and frame.action_text != "Feed only":
            self.log(frame.action_text)
            self.last_logged_action = frame.action_text
        if frame.action_text.startswith("Collision detected"):
            self.feed_status_var.set("Collision reached - stopped")
        elif self.autonomy_enabled:
            self.feed_status_var.set("Autonomous pursuit running")
        elif self.feed_enabled:
            self.feed_status_var.set("Feed running")
        else:
            self.feed_status_var.set("Feed stopped")

        if frame.rgb_image is None:
            return
        self._last_video_rgb_image = frame.rgb_image
        self._update_video_preview()

    def _on_close(self) -> None:
        self.stop_event.set()
        self.feed_enabled = False
        self.autonomy_enabled = False
        self.save_settings()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    style = ttk.Style(root)
    if "vista" in style.theme_names():
        style.theme_use("vista")
    app = DumeDesktopApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
