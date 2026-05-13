from __future__ import annotations

import ipaddress
import json
import socket
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List
from urllib.parse import urlparse

import requests
import streamlit as st
from biped_gait_phase import walk_forward as phase_walk_forward
from ik_path import GRIPPER_ACTIONS, IK_JOINTS, CapturedIkPose, IkPathModule, IkPathPlan
from sequence_live import LiveSequenceRecorder
from sequence_module import DumeSequenceModule

DEFAULT_DEVICE_URL = "http://192.168.4.1"
DEFAULT_MDNS_HOSTNAME = "dume-biped.local"
IDENTIFY_PATH = "/api/identify"
CONTROL_MODE_OPTIONS = [("none", 0), ("axis", 1), ("buttons", 2)]
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
MOTOR_TYPE_OPTIONS = [("positional_180", 0), ("continuous_360", 1)]
APP_ROOT = Path(__file__).resolve().parent
BRANDING_DIR = APP_ROOT / "branding"
DEVICE_CACHE_PATH = APP_ROOT / "device_cache.json"
VIRTUAL_BIPED_POSES_PATH = APP_ROOT / "config" / "biped_virtual_poses.json"
DISCOVERY_TIMEOUT = 1.0
DISCOVERY_WORKERS = 32
BIPED_JOINT_ORDER = ["base", "shoulder", "elbow", "wrist_pitch", "wrist_rotate", "gripper"]
BIPED_SEQUENCE_PRESETS = {
    "Tiptoe forward cycle": "stand,left_forward,right_forward,stand",
    "Tiptoe backward cycle": "stand,right_forward,left_forward,stand",
    "Tutorial forward cycle": "stand,shift_right,right_forward,shift_left,left_forward,stand",
    "Tutorial backward cycle": "stand,shift_left,left_forward,shift_right,right_forward,stand",
    "Simple forward cycle": "stand,left_forward,stand,right_forward,stand",
    "Simple backward cycle": "stand,right_forward,stand,left_forward,stand",
    "Left balance test": "stand,left_forward,stand",
    "Right balance test": "stand,right_forward,stand",
    "Custom": "",
}

VISION_STATE_FALLBACK_TURNS = {
    "APPROACH": ("TURN_LEFT", "TURN_RIGHT"),
    "ALIGN_AND_STEP": ("TURN_LEFT", "TURN_RIGHT"),
}

DEFAULT_VIRTUAL_BIPED_POSES = {
    "crouch_stand": {"base": 70, "shoulder": 115, "elbow": 115, "wrist_pitch": 0, "wrist_rotate": 60, "gripper": 140},
}


def create_http_session() -> requests.Session:
    session = requests.Session()
    session.trust_env = False
    session.headers.update(
        {
            "Connection": "close",
            "User-Agent": "Biped-Dashboard/1.0",
        }
    )
    return session


HTTP_SESSION = create_http_session()

@dataclass
class JointState:
    name: str
    label: str
    coordinate_space: str
    pin: int
    motor_type: str
    min_angle: int
    max_angle: int
    home_angle: int
    step: int
    pulse_min: int
    pulse_max: int
    physical_min_angle: int
    physical_max_angle: int
    physical_angle: float
    physical_home_angle: float
    neutral_output: int
    stop_deadband: int
    max_speed_scale: int
    invert: bool
    position: int
    startup_target: int
    raw_output: int
    stored_min_angle: int
    stored_max_angle: int
    stored_home_angle: int
    stored_physical_min_angle: int
    stored_physical_max_angle: int
    stored_position: int
    attached: bool
    velocity: int
    control_mode: str
    axis_source: str
    positive_button: str
    negative_button: str
    input_invert: bool


@dataclass
class ControllerState:
    enabled: bool
    allow_new_connections: bool
    state: str
    status_text: str
    last_error: str
    scanning_in_progress: bool
    reconnect_in_progress: bool
    connected: bool
    esp32_bt_mac: str
    controller_name: str
    controller_type: str
    controller_bt_addr: str
    remembered_name: str
    remembered_type: str
    remembered_bt_addr: str
    led_r: int
    led_g: int
    led_b: int
    rumble_force: int
    rumble_duration: int
    axis_deadzone: int
    axis_center_lx: int
    axis_center_ly: int
    axis_center_rx: int
    axis_center_ry: int
    home_all_button: str
    battery: int
    battery_raw: int


@dataclass
class WifiState:
    hostname: str
    mdns_hostname: str
    mdns_active: bool
    ap_active: bool
    ap_ssid: str
    ap_ip: str
    sta_ssid: str
    sta_connected: bool
    sta_ip: str
    sta_status: str
    last_result: str
    last_failure: str


@dataclass
class DeviceInfo:
    base_url: str
    hostname: str
    mdns_hostname: str
    ip_address: str
    ap_ip: str
    mac: str
    firmware_version: str
    device_model: str


def option_index(options: List[tuple[str, int]], value: str) -> int:
    for index, (name, _) in enumerate(options):
        if name == value:
            return index
    return 0


def api_get(base_url: str, path: str, params: dict | None = None) -> dict:
    response = HTTP_SESSION.get(f"{base_url.rstrip('/')}{path}", params=params, timeout=8)
    response.raise_for_status()
    payload = response.json()
    if not payload.get("ok", True):
        raise RuntimeError(payload.get("error", "Request failed"))
    return payload


def asset_text(name: str) -> str:
    return (BRANDING_DIR / name).read_text(encoding="utf-8")


def render_brand_header() -> None:
    st.image(
        str(BRANDING_DIR / "logo-horizontal.svg"),
        use_container_width=True,
    )


def render_sidebar_brand() -> None:
    st.sidebar.image(
        str(BRANDING_DIR / "logo-mark.svg"),
        use_container_width=True,
    )


def parse_joint_state(payload: dict) -> JointState:
    return JointState(
        name=payload["name"],
        label=str(payload.get("label", payload["name"].replace("_", " ").title())),
        coordinate_space=str(payload.get("coordinate_space", "")),
        pin=int(payload["pin"]),
        motor_type=str(payload["motor_type"]),
        min_angle=int(payload["min_angle"]),
        max_angle=int(payload["max_angle"]),
        home_angle=int(payload["home_angle"]),
        step=int(payload["step"]),
        pulse_min=int(payload["pulse_min"]),
        pulse_max=int(payload["pulse_max"]),
        physical_min_angle=int(payload.get("physical_min_angle", payload["min_angle"])),
        physical_max_angle=int(payload.get("physical_max_angle", payload["max_angle"])),
        physical_angle=float(payload.get("physical_angle", payload["position"])),
        physical_home_angle=float(payload.get("physical_home_angle", payload["home_angle"])),
        neutral_output=int(payload.get("neutral_output", 90)),
        stop_deadband=int(payload.get("stop_deadband", 3)),
        max_speed_scale=int(payload.get("max_speed_scale", 100)),
        invert=bool(payload["invert"]),
        position=int(payload["position"]),
        startup_target=int(payload.get("startup_target", payload["home_angle"])),
        raw_output=int(payload.get("raw_output", payload["position"])),
        stored_min_angle=int(payload.get("stored_min_angle", payload["min_angle"])),
        stored_max_angle=int(payload.get("stored_max_angle", payload["max_angle"])),
        stored_home_angle=int(payload.get("stored_home_angle", payload["home_angle"])),
        stored_physical_min_angle=int(payload.get("stored_physical_min_angle", payload.get("physical_min_angle", payload["min_angle"]))),
        stored_physical_max_angle=int(payload.get("stored_physical_max_angle", payload.get("physical_max_angle", payload["max_angle"]))),
        stored_position=int(payload.get("stored_position", payload["position"])),
        attached=bool(payload["attached"]),
        velocity=int(payload["velocity"]),
        control_mode=str(payload["control_mode"]),
        axis_source=str(payload["axis_source"]),
        positive_button=str(payload["positive_button"]),
        negative_button=str(payload["negative_button"]),
        input_invert=bool(payload["input_invert"]),
    )


def parse_controller_state(payload: dict) -> ControllerState:
    return ControllerState(
        enabled=bool(payload["enabled"]),
        allow_new_connections=bool(payload["allow_new_connections"]),
        state=str(payload.get("state", "idle")),
        status_text=str(payload.get("status_text", "")),
        last_error=str(payload.get("last_error", "")),
        scanning_in_progress=bool(payload.get("scanning_in_progress", False)),
        reconnect_in_progress=bool(payload.get("reconnect_in_progress", False)),
        connected=bool(payload["connected"]),
        esp32_bt_mac=str(payload["esp32_bt_mac"]),
        controller_name=str(payload["controller_name"]),
        controller_type=str(payload["controller_type"]),
        controller_bt_addr=str(payload["controller_bt_addr"]),
        remembered_name=str(payload.get("remembered_name", "")),
        remembered_type=str(payload.get("remembered_type", "")),
        remembered_bt_addr=str(payload.get("remembered_bt_addr", "")),
        led_r=int(payload["led_r"]),
        led_g=int(payload["led_g"]),
        led_b=int(payload["led_b"]),
        rumble_force=int(payload["rumble_force"]),
        rumble_duration=int(payload["rumble_duration"]),
        axis_deadzone=int(payload["axis_deadzone"]),
        axis_center_lx=int(payload["axis_center_lx"]),
        axis_center_ly=int(payload["axis_center_ly"]),
        axis_center_rx=int(payload["axis_center_rx"]),
        axis_center_ry=int(payload["axis_center_ry"]),
        home_all_button=str(payload["home_all_button"]),
        battery=int(payload["battery"]),
        battery_raw=int(payload["battery_raw"]),
    )


def parse_wifi_state(payload: dict) -> WifiState:
    return WifiState(
        hostname=str(payload.get("hostname", "")),
        mdns_hostname=str(payload.get("mdns_hostname", "")),
        mdns_active=bool(payload.get("mdns_active", False)),
        ap_active=bool(payload.get("ap_active", True)),
        ap_ssid=str(payload.get("ap_ssid", "")),
        ap_ip=str(payload.get("ap_ip", "")),
        sta_ssid=str(payload.get("sta_ssid", "")),
        sta_connected=bool(payload.get("sta_connected", False)),
        sta_ip=str(payload.get("sta_ip", "")),
        sta_status=str(payload.get("sta_status", "unknown")),
        last_result=str(payload.get("last_result", "")),
        last_failure=str(payload.get("last_failure", "")),
    )


def fetch_state(base_url: str) -> dict:
    payload = api_get(base_url, "/api/state")
    joints = {joint["name"]: parse_joint_state(joint) for joint in payload["joints"]}
    return {
        "controller": parse_controller_state(payload["ps4"]),
        "wifi": parse_wifi_state(payload["wifi"]),
        "joints": joints,
        "biped": payload.get("biped", {}),
        "device_type": str(payload.get("device_type", "")),
        "device_model": str(payload.get("device_model", "")),
        "firmware_version": str(payload.get("firmware_version", "")),
    }


def sync_state(base_url: str) -> None:
    st.session_state.robot_state = fetch_state(base_url)


def load_sequence_config(base_url: str) -> dict:
    return {
        "bridge": {
            "device_base_url": normalize_base_url(base_url),
            "dry_run": True,
        },
        "controller": {
            "axis_deadzone": 48,
        },
    }


def get_sequence_recorder(base_url: str) -> LiveSequenceRecorder:
    normalized = normalize_base_url(base_url)
    recorder = st.session_state.get("main_sequence_live_recorder")
    recorder_url = st.session_state.get("main_sequence_live_recorder_url")
    if recorder is None or recorder_url != normalized:
        recorder = LiveSequenceRecorder(root=APP_ROOT, config=load_sequence_config(normalized))
        st.session_state.main_sequence_live_recorder = recorder
        st.session_state.main_sequence_live_recorder_url = normalized
    return recorder


def get_sequence_module(base_url: str) -> DumeSequenceModule:
    normalized = normalize_base_url(base_url)
    module = st.session_state.get("main_sequence_module")
    module_url = st.session_state.get("main_sequence_module_url")
    if module is None or module_url != normalized:
        module = DumeSequenceModule(root=APP_ROOT, config=load_sequence_config(normalized))
        st.session_state.main_sequence_module = module
        st.session_state.main_sequence_module_url = normalized
    return module


def get_ik_path_module(base_url: str) -> IkPathModule:
    normalized = normalize_base_url(base_url)
    module = st.session_state.get("main_ik_path_module")
    module_url = st.session_state.get("main_ik_path_module_url")
    if module is None or module_url != normalized:
        module = IkPathModule(root=APP_ROOT, base_url=normalized)
        st.session_state.main_ik_path_module = module
        st.session_state.main_ik_path_module_url = normalized
    return module


def normalize_base_url(value: str) -> str:
    candidate = value.strip()
    if not candidate:
        return ""
    if "://" not in candidate:
        candidate = f"http://{candidate}"
    return candidate.rstrip("/")


def canonical_device_base_url(base_url: str, payload: dict | None = None) -> str:
    normalized = normalize_base_url(base_url)
    if not payload:
        return normalized
    ip_address = str(payload.get("ip_address", "")).strip()
    if ip_address:
        return normalize_base_url(ip_address)
    return normalized


def local_cache() -> dict:
    if DEVICE_CACHE_PATH.exists():
        try:
            return json.loads(DEVICE_CACHE_PATH.read_text(encoding="utf-8"))
        except Exception:
            return {}
    return {}


def save_local_cache(data: dict) -> None:
    DEVICE_CACHE_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")


def load_virtual_biped_poses() -> dict[str, dict[str, int]]:
    poses = {name: dict(values) for name, values in DEFAULT_VIRTUAL_BIPED_POSES.items()}
    if VIRTUAL_BIPED_POSES_PATH.exists():
        try:
            payload = json.loads(VIRTUAL_BIPED_POSES_PATH.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                for name, values in payload.items():
                    if isinstance(name, str) and isinstance(values, dict):
                        poses[name] = {str(joint): int(angle) for joint, angle in values.items()}
        except Exception:
            pass
    return poses


def save_virtual_biped_pose(name: str, angles: dict[str, int]) -> None:
    poses = load_virtual_biped_poses()
    poses[name] = {str(joint): int(angle) for joint, angle in angles.items()}
    VIRTUAL_BIPED_POSES_PATH.parent.mkdir(parents=True, exist_ok=True)
    VIRTUAL_BIPED_POSES_PATH.write_text(json.dumps(poses, indent=2), encoding="utf-8")


def is_captured_ik_pose(value: object) -> bool:
    return hasattr(value, "servo_pose") and hasattr(value, "cartesian_pose_mm")


def is_ik_path_plan(value: object) -> bool:
    return hasattr(value, "name") and hasattr(value, "commands") and hasattr(value, "start_pose") and hasattr(value, "end_pose")


def parse_device_info(base_url: str, payload: dict) -> DeviceInfo:
    canonical_base_url = canonical_device_base_url(base_url, payload)
    return DeviceInfo(
        base_url=canonical_base_url,
        hostname=str(payload.get("hostname", "")),
        mdns_hostname=str(payload.get("mdns_hostname", "")),
        ip_address=str(payload.get("ip_address", "")),
        ap_ip=str(payload.get("ap_ip", "")),
        mac=str(payload.get("mac", "")),
        firmware_version=str(payload.get("firmware_version", "")),
        device_model=str(payload.get("device_model", "")),
    )


def probe_device(base_url: str, timeout: float = DISCOVERY_TIMEOUT) -> DeviceInfo | None:
    candidate = normalize_base_url(base_url)
    if not candidate:
        return None
    try:
        response = HTTP_SESSION.get(f"{candidate}{IDENTIFY_PATH}", timeout=timeout)
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", True):
            return None
        if payload.get("device_type") not in {"robot_arm", "robot_biped"}:
            return None
        return parse_device_info(candidate, payload)
    except Exception:
        return None


def cache_successful_device(base_url: str, device: DeviceInfo | None = None) -> None:
    cache = local_cache()
    cache["last_success_url"] = normalize_base_url(base_url)
    if device is not None:
        cache["last_hostname"] = device.hostname
        cache["last_mdns_hostname"] = device.mdns_hostname
        cache["last_ip"] = device.ip_address
    save_local_cache(cache)


def local_ipv4_addresses() -> list[str]:
    addresses: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                addresses.add(ip)
    except Exception:
        pass
    return sorted(addresses)


def probable_subnets() -> list[ipaddress.IPv4Network]:
    subnets: list[ipaddress.IPv4Network] = []
    cache = local_cache()
    candidates = local_ipv4_addresses()
    if cache.get("last_ip"):
        candidates.insert(0, str(cache["last_ip"]))
    seen: set[str] = set()
    for ip in candidates:
        try:
            network = ipaddress.ip_network(f"{ip}/24", strict=False)
        except ValueError:
            continue
        if network.network_address.is_loopback:
            continue
        key = str(network)
        if key not in seen:
            seen.add(key)
            subnets.append(network)
    return subnets


def dedupe_devices(devices: list[DeviceInfo]) -> list[DeviceInfo]:
    deduped: dict[str, DeviceInfo] = {}
    for device in devices:
        deduped[device.base_url] = device
    return list(deduped.values())


def discovery_candidates() -> list[str]:
    cache = local_cache()
    candidates: list[str] = []
    if cache.get("last_success_url"):
        candidates.append(str(cache["last_success_url"]))
    for hostname in [cache.get("last_mdns_hostname"), cache.get("last_hostname"), DEFAULT_MDNS_HOSTNAME]:
        if hostname:
            host = str(hostname)
            if not host.endswith(".local") and "." not in host:
                host = f"{host}.local"
            candidates.append(f"http://{host}")
    seen: list[str] = []
    for item in candidates:
        normalized = normalize_base_url(item)
        if normalized and normalized not in seen:
            seen.append(normalized)
    return seen


def scan_subnet(network: ipaddress.IPv4Network) -> list[DeviceInfo]:
    devices: list[DeviceInfo] = []
    hosts = [str(host) for host in network.hosts()]
    with ThreadPoolExecutor(max_workers=DISCOVERY_WORKERS) as executor:
        futures = {
            executor.submit(probe_device, f"http://{host}", DISCOVERY_TIMEOUT): host
            for host in hosts
        }
        for future in as_completed(futures):
            device = future.result()
            if device is not None:
                devices.append(device)
    return devices


def discover_devices() -> tuple[list[DeviceInfo], list[str]]:
    results: list[DeviceInfo] = []
    log: list[str] = []
    for candidate in discovery_candidates():
        device = probe_device(candidate, timeout=1.0)
        if device is not None:
            results.append(device)
            log.append(f"Resolved {candidate}")
    if results:
        return dedupe_devices(results), log
    for network in probable_subnets():
        log.append(f"Scanning {network}")
        devices = scan_subnet(network)
        if devices:
            results.extend(devices)
            break
    return dedupe_devices(results), log


def format_request_exception(exc: Exception) -> str:
    if isinstance(exc, requests.exceptions.ConnectTimeout):
        return "connection timed out"
    if isinstance(exc, requests.exceptions.ReadTimeout):
        return "device responded too slowly"
    if isinstance(exc, requests.exceptions.ConnectionError):
        return "connection failed"
    if isinstance(exc, requests.exceptions.HTTPError):
        response = exc.response
        if response is not None:
            return f"HTTP {response.status_code}"
        return "HTTP error"
    if isinstance(exc, ValueError):
        return "response was not valid JSON"
    return str(exc)


def diagnose_device_connection(base_url: str, original_error: Exception) -> str:
    target = normalize_base_url(base_url)
    details: list[str] = [f"Failed to connect to `{target}`: {format_request_exception(original_error)}."]
    for path, timeout in [(IDENTIFY_PATH, 3), ("/api/state", 5)]:
        try:
            response = HTTP_SESSION.get(f"{target}{path}", timeout=timeout)
            response.raise_for_status()
            payload = response.json()
            details.append(
                f"`{path}` responded with HTTP {response.status_code}, ok={payload.get('ok', True)}, "
                f"device_type={payload.get('device_type', 'n/a')}."
            )
        except Exception as exc:
            details.append(f"`{path}` check failed: {format_request_exception(exc)}.")
    local_ips = local_ipv4_addresses()
    if local_ips:
        details.append(f"Local IPv4 addresses: {', '.join(local_ips)}.")
    details.append("Confirm this PC is on the same SSID/VLAN as the ESP and not on an isolated guest network.")
    return " ".join(details)


def connect_device(base_url: str, device: DeviceInfo | None = None) -> bool:
    target = normalize_base_url(base_url)
    try:
        if device is None:
            device = probe_device(target, timeout=1.0)
        resolved_target = device.base_url if device is not None else target
        sync_state(resolved_target)
        st.session_state.connected_base_url = resolved_target
        st.session_state.device_url = resolved_target
        st.session_state.pop("device_error", None)
        if device is not None:
            cache_successful_device(resolved_target, device)
        else:
            cache_successful_device(resolved_target)
        return True
    except Exception as exc:
        st.session_state.device_error = diagnose_device_connection(target, exc)
        return False


def auto_discovery_flow() -> None:
    signature = "|".join(local_ipv4_addresses())
    if st.session_state.get("discovery_signature") == signature and st.session_state.get("discovery_ran"):
        return
    st.session_state.discovery_signature = signature
    st.session_state.discovery_ran = True
    with st.spinner("Searching for the biped robot on your network..."):
        devices, log = discover_devices()
    st.session_state.discovery_log = log
    st.session_state.discovered_devices = devices
    if len(devices) == 1:
        connect_device(devices[0].base_url, devices[0])


def run_device_action(base_url: str, path: str, params: dict | None = None, refresh: bool = False, rerun: bool = False) -> bool:
    try:
        api_get(base_url, path, params=params)
        if refresh:
            sync_state(base_url)
        st.session_state.pop("device_error", None)
        if rerun:
            st.rerun()
        return True
    except Exception as exc:
        st.session_state.device_error = str(exc)
        return False


def determine_biped_vision_sequence(state_name: str, mapping: dict[str, str]) -> str:
    return mapping.get(state_name, "").strip()


def build_soft_shift_poses(
    poses_by_name: dict[str, dict[str, int]],
    joint_limits: dict[str, tuple[int, int]],
) -> dict[str, dict[str, int]]:
    required = {"left_forward", "right_forward"}
    if not required.issubset(poses_by_name):
        missing = ", ".join(sorted(required - set(poses_by_name)))
        raise ValueError(f"Missing required poses: {missing}")

    base_pose_name = "crouch_stand" if "crouch_stand" in poses_by_name else "stand"
    stand = poses_by_name[base_pose_name]
    left_forward = poses_by_name["left_forward"]
    right_forward = poses_by_name["right_forward"]
    factor = 0.4

    generated = {
        "shift_left": dict(stand),
        "shift_right": dict(stand),
    }
    left_side_joints = ["base", "shoulder", "elbow"]
    right_side_joints = ["wrist_pitch", "wrist_rotate", "gripper"]

    for joint_name in left_side_joints:
        low, high = joint_limits[joint_name]
        value = int(round(int(stand[joint_name]) + factor * (int(left_forward[joint_name]) - int(stand[joint_name]))))
        generated["shift_left"][joint_name] = max(low, min(high, value))

    for joint_name in right_side_joints:
        low, high = joint_limits[joint_name]
        value = int(round(int(stand[joint_name]) + factor * (int(right_forward[joint_name]) - int(stand[joint_name]))))
        generated["shift_right"][joint_name] = max(low, min(high, value))

    return generated


def execute_biped_pose_values(
    base_url: str,
    pose_values: dict[str, int],
    duration_ms: int,
    hold_ms: int,
    refresh: bool = True,
) -> bool:
    for joint_name in BIPED_JOINT_ORDER:
        if joint_name not in pose_values:
            continue
        if not run_device_action(
            base_url,
            "/api/joint",
            params={"cmd": "move", "joint": joint_name, "value": int(pose_values[joint_name])},
            refresh=False,
            rerun=False,
        ):
            return False
    if duration_ms > 0:
        time.sleep(duration_ms / 1000.0)
    if hold_ms > 0:
        time.sleep(hold_ms / 1000.0)
    if refresh:
        sync_state(base_url)
    return True


def execute_mixed_biped_sequence(
    base_url: str,
    sequence_names: str,
    poses_by_name: dict[str, dict[str, int]],
    firmware_pose_names: set[str],
    duration_ms: int,
    interp_steps: int,
    hold_ms: int,
    repeat_count: int = 1,
    refresh: bool = True,
) -> bool:
    names = [token.strip() for token in sequence_names.split(",") if token.strip()]
    if not names:
        return False
    for _ in range(max(1, int(repeat_count))):
        for pose_name in names:
            if pose_name in firmware_pose_names:
                if not run_device_action(
                    base_url,
                    "/api/biped",
                    params={
                        "cmd": "run_pose",
                        "name": pose_name,
                        "duration_ms": int(duration_ms),
                        "interp_steps": int(interp_steps),
                        "hold_ms": int(hold_ms),
                    },
                    refresh=False,
                    rerun=False,
                ):
                    return False
            elif pose_name in poses_by_name:
                if not execute_biped_pose_values(base_url, poses_by_name[pose_name], int(duration_ms), int(hold_ms), refresh=False):
                    return False
            else:
                st.session_state.device_error = f"Unknown pose in sequence: {pose_name}"
                return False
    if refresh:
        sync_state(base_url)
    return True


def execute_biped_sequence(
    base_url: str,
    sequence_names: str,
    duration_ms: int,
    interp_steps: int,
    hold_ms: int,
    repeat_count: int = 1,
    refresh: bool = True,
) -> bool:
    if not sequence_names.strip():
        return False
    return run_device_action(
        base_url,
        "/api/biped",
        params={
            "cmd": "play_sequence",
            "names": sequence_names.strip(),
            "duration_ms": int(duration_ms),
            "interp_steps": int(interp_steps),
            "hold_ms": int(hold_ms),
            "repeat": int(repeat_count),
        },
        refresh=refresh,
    )


def choose_vision_action_state(
    info: dict,
    previous_distance_cm: float | None,
    distance_increase_tolerance_cm: float,
) -> str:
    state_name = str(info.get("state", ""))
    current_distance_cm = info.get("distance_cm")
    dx_cm = info.get("dx_cm")
    if (
        state_name in VISION_STATE_FALLBACK_TURNS
        and previous_distance_cm is not None
        and isinstance(current_distance_cm, (int, float))
        and current_distance_cm > previous_distance_cm + distance_increase_tolerance_cm
    ):
        left_state, right_state = VISION_STATE_FALLBACK_TURNS[state_name]
        if isinstance(dx_cm, (int, float)):
            return right_state if dx_cm > 0 else left_state
        return left_state
    return state_name


def device_label(device: DeviceInfo) -> str:
    mdns = device.mdns_hostname or "no-mdns"
    ip = device.ip_address or device.base_url
    return f"{device.device_model} | {mdns} | {ip}"


def render_connection_sidebar() -> str:
    cache = local_cache()
    st.sidebar.subheader("Connection")
    last_known = cache.get("last_success_url") or "none"
    st.sidebar.caption(f"Last successful device: `{last_known}`")

    if st.sidebar.button("Find biped devices", use_container_width=True):
        with st.spinner("Searching for biped devices..."):
            devices, log = discover_devices()
        st.session_state.discovered_devices = devices
        st.session_state.discovery_log = log
        if len(devices) == 1:
            connect_device(devices[0].base_url, devices[0])

    discovered_devices: list[DeviceInfo] = st.session_state.get("discovered_devices", [])
    if discovered_devices:
        if len(discovered_devices) == 1:
            device = discovered_devices[0]
            st.sidebar.success(f"Found `{device_label(device)}`")
        else:
            labels = [device_label(device) for device in discovered_devices]
            selected_label = st.sidebar.selectbox("Discovered devices", labels, key="device_picker")
            selected_device = discovered_devices[labels.index(selected_label)]
            if st.sidebar.button("Connect to selected device", use_container_width=True):
                connect_device(selected_device.base_url, selected_device)
    else:
        st.sidebar.info("Automatic discovery uses last known address, mDNS, then a local subnet scan.")

    if st.session_state.get("discovery_log"):
        with st.sidebar.expander("Discovery details"):
            for item in st.session_state["discovery_log"]:
                st.write(item)

    manual_default = st.session_state.get("device_url") or cache.get("last_success_url") or DEFAULT_DEVICE_URL
    with st.sidebar.expander("Manual URL override"):
        manual_url = st.text_input("Device URL", value=manual_default, key="manual_device_url").strip()
        st.caption("Use this only if automatic discovery fails or for advanced debugging.")
        if st.button("Connect manually", use_container_width=True):
            connect_device(manual_url)
    return normalize_base_url(manual_default)


def render_sequence_recorder_section(base_url: str) -> None:
    recorder = get_sequence_recorder(base_url)
    module = get_sequence_module(base_url)
    snapshot = recorder.snapshot()

    with st.expander("Sequence Recorder", expanded=False):
        st.caption(
            "Record controller-driven moves into a replayable sequence. "
            "This uses live `/api/state` updates and `ps4.inputs` for more reliable 360-joint capture."
        )

        controls_left, controls_mid, controls_right = st.columns(3)
        with controls_left:
            session_name = st.text_input(
                "Session name",
                value=st.session_state.get("main_sequence_session_name", "test_sequence"),
                key="main_sequence_session_name_input",
            )
            st.session_state.main_sequence_session_name = session_name
            overwrite_existing = st.checkbox("Overwrite existing session file", value=False, key="main_sequence_overwrite")
        with controls_mid:
            start_clicked = st.button("Start Recording", use_container_width=True, disabled=snapshot.running, key="main_sequence_start")
        with controls_right:
            stop_clicked = st.button("Stop Recording", use_container_width=True, disabled=not snapshot.running, key="main_sequence_stop")

        if start_clicked:
            try:
                session = recorder.start(session_name, overwrite_existing=overwrite_existing)
                st.success(f"Recording started: {session.session_name}")
                st.rerun()
            except Exception as exc:
                st.error(str(exc))

        if stop_clicked:
            session = recorder.stop()
            if session is not None:
                st.success(f"Recording stopped: {session.session_name}")
                st.rerun()

        status_left, status_mid, status_right = st.columns(3)
        with status_left:
            st.metric("Recorder", "Running" if snapshot.running else "Idle")
            st.write(f"Biped base URL: `{snapshot.dume_base_url or normalize_base_url(base_url)}`")
        with status_mid:
            st.metric("Steps Recorded", snapshot.steps_recorded)
            st.write(f"Controller connected: `{snapshot.controller_connected}`")
        with status_right:
            st.write(f"Included 360 joint: `{snapshot.included_continuous_joint or 'none'}`")
            st.write(f"Excluded 360 joints: `{', '.join(snapshot.excluded_continuous_joints) or 'none'}`")

        if snapshot.running:
            st.info("Recording is active. Move the biped robot with the controller now.")
        if snapshot.last_error:
            st.error(f"Recorder issue: {snapshot.last_error}")

        session_to_show = snapshot.session_name or st.session_state.get("main_sequence_session_name")
        if session_to_show:
            session_path = module.session_path(session_to_show)
            if session_path.exists():
                session_payload = json.loads(session_path.read_text(encoding="utf-8"))
                steps = session_payload.get("steps", [])
                if steps:
                    st.dataframe(
                        [
                            {
                                "index": index + 1,
                                "kind": step.get("kind"),
                                "joint": step.get("joint_name"),
                                "target": step.get("target_value"),
                                "speed": step.get("speed_percent"),
                                "direction": step.get("direction"),
                                "duration_ms": step.get("duration_ms"),
                                "delay_after_ms": step.get("delay_after_ms"),
                                "note": step.get("note"),
                            }
                            for index, step in enumerate(steps)
                        ],
                        use_container_width=True,
                        hide_index=True,
                    )
                else:
                    st.caption("No steps recorded yet for this session.")

                action_left, action_mid = st.columns(2)
                with action_left:
                    if st.button("Preview Replay", use_container_width=True, disabled=snapshot.running, key="main_sequence_preview_replay"):
                        st.session_state.main_sequence_replay_result = module.replay(session_to_show, dry_run=True).to_dict()
                    if st.button("Execute Replay", use_container_width=True, disabled=snapshot.running, key="main_sequence_execute_replay"):
                        st.session_state.main_sequence_replay_result = module.replay(session_to_show, dry_run=False).to_dict()
                with action_mid:
                    if st.button("Preview Return Home", use_container_width=True, disabled=snapshot.running, key="main_sequence_preview_return"):
                        st.session_state.main_sequence_return_result = module.return_home(session_to_show, dry_run=True).to_dict()
                    if st.button("Execute Return Home", use_container_width=True, disabled=snapshot.running, key="main_sequence_execute_return"):
                        st.session_state.main_sequence_return_result = module.return_home(session_to_show, dry_run=False).to_dict()

                with st.expander("Session JSON", expanded=False):
                    st.json(session_payload)

        if "main_sequence_replay_result" in st.session_state:
            with st.expander("Replay Result", expanded=False):
                st.json(st.session_state.main_sequence_replay_result)

        if "main_sequence_return_result" in st.session_state:
            with st.expander("Return Home Result", expanded=False):
                st.json(st.session_state.main_sequence_return_result)

    if snapshot.running:
        time.sleep(0.5)
        st.rerun()


def render_ik_path_section(base_url: str) -> None:
    state = st.session_state.get("robot_state", {})
    with st.expander("Full IK Path", expanded=False):
        if str(state.get("device_type", "")) == "robot_biped" or "Biped" in str(state.get("device_model", "")):
            st.info("Full IK Path is disabled for the biped configuration. The current IK workflow is specific to the original arm geometry. Use `Home all joints` to return to the stand pose and `Sequence Recorder` to build reusable posture or gait sequences.")
            return
        module = get_ik_path_module(base_url)
        st.caption(
            "Capture a manual `start` and `end` gripper position, then generate a Cartesian IK path using only `base`, `shoulder`, and `elbow`. "
            "Optional safe-height waypoints help avoid platform collisions, and path building now rejects waypoints that dip into the platform footprint."
        )

        name_col, steps_col, delay_col, safe_col = st.columns(4)
        with name_col:
            plan_name = st.text_input(
                "IK path name",
                value=st.session_state.get("main_ik_path_name", "ik_path_1"),
                key="main_ik_path_name_input",
            )
            st.session_state.main_ik_path_name = plan_name
        with steps_col:
            interpolation_steps = st.number_input("Steps per segment", min_value=2, max_value=80, value=10, step=1)
        with delay_col:
            delay_after_ms = st.number_input("Delay after each waypoint (ms)", min_value=0, max_value=3000, value=140, step=10)
        with safe_col:
            safe_height_mm = st.number_input("Safe travel height Z (mm)", min_value=0.0, max_value=500.0, value=120.0, step=5.0)

        use_safe_height = st.checkbox("Use safe-height waypoint path", value=True, key="main_ik_use_safe_height")
        grip_left, grip_mid, grip_right = st.columns(3)
        with grip_left:
            gripper_prepare_action = st.selectbox(
                "Gripper before start move",
                options=list(GRIPPER_ACTIONS),
                index=list(GRIPPER_ACTIONS).index("open"),
                key="main_ik_gripper_prepare_action",
            )
        with grip_mid:
            gripper_action_after_start_delay = st.selectbox(
                "Gripper after reaching start",
                options=list(GRIPPER_ACTIONS),
                index=list(GRIPPER_ACTIONS).index("close"),
                key="main_ik_gripper_action_after_start_delay",
            )
        with grip_right:
            gripper_action_at_end = st.selectbox(
                "Gripper after reaching end",
                options=list(GRIPPER_ACTIONS),
                index=list(GRIPPER_ACTIONS).index("open"),
                key="main_ik_gripper_action_end",
            )
        grip_cfg_left, grip_cfg_mid, grip_cfg_right = st.columns(3)
        with grip_cfg_left:
            gripper_start_delay_ms = st.number_input(
                "Start grip delay (ms)",
                min_value=0,
                max_value=5000,
                value=300,
                step=50,
                key="main_ik_gripper_start_delay_ms",
            )
        with grip_cfg_mid:
            gripper_end_delay_ms = st.number_input(
                "End release delay (ms)",
                min_value=0,
                max_value=5000,
                value=0,
                step=50,
                key="main_ik_gripper_end_delay_ms",
            )
        with grip_cfg_right:
            gripper_close_value = st.number_input(
                "Gripper close angle",
                min_value=0,
                max_value=180,
                value=20,
                step=1,
                key="main_ik_gripper_close_value",
            )

        capture_left, capture_mid, capture_right = st.columns(3)
        with capture_left:
            if st.button("Capture Start Pose", use_container_width=True, key="main_ik_capture_start"):
                try:
                    st.session_state.main_ik_start = module.capture_pose()
                    st.success("Captured start pose.")
                except Exception as exc:
                    st.error(str(exc))
        with capture_mid:
            if st.button("Capture End Pose", use_container_width=True, key="main_ik_capture_end"):
                try:
                    st.session_state.main_ik_end = module.capture_pose()
                    st.success("Captured end pose.")
                except Exception as exc:
                    st.error(str(exc))
        with capture_right:
            if st.button("Clear IK Poses", use_container_width=True, key="main_ik_clear"):
                st.session_state.pop("main_ik_start", None)
                st.session_state.pop("main_ik_end", None)
                st.session_state.pop("main_ik_plan", None)
                st.session_state.pop("main_ik_result", None)
                st.rerun()

        start_pose = st.session_state.get("main_ik_start")
        end_pose = st.session_state.get("main_ik_end")

        status_left, status_right = st.columns(2)
        with status_left:
            st.markdown("**Start pose**")
            if is_captured_ik_pose(start_pose):
                st.json({"servo": start_pose.servo_pose, "cartesian_mm": start_pose.cartesian_pose_mm})
            else:
                st.caption("Not captured yet.")
        with status_right:
            st.markdown("**End pose**")
            if is_captured_ik_pose(end_pose):
                st.json({"servo": end_pose.servo_pose, "cartesian_mm": end_pose.cartesian_pose_mm})
            else:
                st.caption("Not captured yet.")

        if st.button("Build IK Path", use_container_width=True, key="main_ik_build"):
            if not is_captured_ik_pose(start_pose) or not is_captured_ik_pose(end_pose):
                st.error("Capture both start and end poses first.")
            else:
                try:
                    plan = module.build_plan(
                        plan_name,
                        start_pose,
                        end_pose,
                        interpolation_steps=int(interpolation_steps),
                        delay_after_ms=int(delay_after_ms),
                        use_safe_height=bool(use_safe_height),
                        safe_height_mm=float(safe_height_mm),
                        gripper_prepare_action=gripper_prepare_action,
                        gripper_action_after_start_delay=gripper_action_after_start_delay,
                        gripper_action_at_end=gripper_action_at_end,
                        gripper_start_delay_ms=int(gripper_start_delay_ms),
                        gripper_end_delay_ms=int(gripper_end_delay_ms),
                        gripper_close_value=int(gripper_close_value),
                    )
                    st.session_state.main_ik_plan = plan
                    st.success(f"Built IK path: {plan.name}")
                except Exception as exc:
                    st.error(str(exc))

        plan = st.session_state.get("main_ik_plan")
        if is_ik_path_plan(plan):
            st.write(f"Path `{plan.name}` with `{len(plan.commands)}` solved IK waypoints across `{', '.join(IK_JOINTS)}`.")
            st.caption(
                f"Gripper: before start `{plan.gripper_prepare_action}`"
                + (f" -> {plan.gripper_open_value}" if plan.gripper_prepare_action == "open" and plan.gripper_open_value is not None else "")
                + (f" -> {plan.gripper_close_value}" if plan.gripper_prepare_action == "close" and plan.gripper_close_value is not None else "")
                + f", after start `{plan.gripper_action_after_start_delay}` after {plan.gripper_start_delay_ms} ms"
                + (f" -> {plan.gripper_open_value}" if plan.gripper_action_after_start_delay == "open" and plan.gripper_open_value is not None else "")
                + (f" -> {plan.gripper_close_value}" if plan.gripper_action_after_start_delay == "close" and plan.gripper_close_value is not None else "")
                + f", end `{plan.gripper_action_at_end}`"
                + (f" -> {plan.gripper_open_value}" if plan.gripper_action_at_end == "open" and plan.gripper_open_value is not None else "")
                + (f" -> {plan.gripper_close_value}" if plan.gripper_action_at_end == "close" and plan.gripper_close_value is not None else "")
                + "."
            )
            st.dataframe(
                [
                    {
                        "index": index + 1,
                        "base": command["base"],
                        "shoulder": command["shoulder"],
                        "elbow": command["elbow"],
                    }
                    for index, command in enumerate(plan.commands)
                ],
                use_container_width=True,
                hide_index=True,
            )

            execute_left, execute_mid = st.columns(2)
            with execute_left:
                if st.button("Preview IK Execution", use_container_width=True, key="main_ik_preview"):
                    st.session_state.main_ik_result = module.execute_plan(plan, dry_run=True)
            with execute_mid:
                if st.button("Execute IK Path", use_container_width=True, key="main_ik_execute"):
                    st.session_state.main_ik_result = module.execute_plan(plan, dry_run=False)

        if "main_ik_result" in st.session_state:
            with st.expander("IK Result", expanded=False):
                st.json(st.session_state.main_ik_result)


def render_biped_gait_section(base_url: str, state: dict) -> None:
    is_biped = str(state.get("device_type", "")) == "robot_biped" or "Biped" in str(state.get("device_model", ""))
    if not is_biped:
        return

    with st.expander("Biped Gait", expanded=False):
        st.caption(
            "Pose-driven calibration for the six-servo biped. The ESP32 stores named poses in flash and only executes interpolation and sequence playback; you tune poses here without reflashing again."
        )

        biped_data = state.get("biped", {}) or {}
        raw_poses = biped_data.get("poses", [])
        firmware_poses_by_name = {
            str(entry.get("name", "")).strip(): {str(k): int(v) for k, v in (entry.get("angles", {}) or {}).items()}
            for entry in raw_poses
            if str(entry.get("name", "")).strip()
        }
        if not firmware_poses_by_name:
            firmware_poses_by_name = {
                "stand": {"base": 80, "shoulder": 90, "elbow": 110, "wrist_pitch": 25, "wrist_rotate": 40, "gripper": 135},
                "left_forward": {"base": 110, "shoulder": 100, "elbow": 180, "wrist_pitch": 60, "wrist_rotate": 40, "gripper": 135},
                "right_forward": {"base": 80, "shoulder": 55, "elbow": 110, "wrist_pitch": 15, "wrist_rotate": 10, "gripper": 66},
                "shift_left": {"base": 80, "shoulder": 90, "elbow": 110, "wrist_pitch": 25, "wrist_rotate": 40, "gripper": 135},
                "shift_right": {"base": 80, "shoulder": 90, "elbow": 110, "wrist_pitch": 25, "wrist_rotate": 40, "gripper": 135},
            }
        virtual_poses_by_name = load_virtual_biped_poses()
        firmware_pose_names = set(firmware_poses_by_name.keys())
        poses_by_name = dict(firmware_poses_by_name)
        poses_by_name.update(virtual_poses_by_name)

        pose_names = list(poses_by_name.keys())
        timing_left, timing_mid, timing_right, timing_far = st.columns(4)
        with timing_left:
            pose_duration_ms = st.number_input(
                "Pose duration (ms)", min_value=0, max_value=5000, value=400, step=25, key="biped_pose_duration_ms"
            )
        with timing_mid:
            interp_steps = st.number_input(
                "Interpolation steps", min_value=1, max_value=100, value=20, step=1, key="biped_interp_steps"
            )
        with timing_right:
            hold_ms = st.number_input("Hold per pose (ms)", min_value=0, max_value=2000, value=80, step=10, key="biped_hold_ms")
        with timing_far:
            repeat_count = st.number_input("Sequence repeats", min_value=1, max_value=20, value=1, step=1, key="biped_repeat_count")

        top_actions = st.columns(3)
        with top_actions[0]:
            if st.button("Stand now", use_container_width=True, key="biped_stand_now"):
                run_device_action(
                    base_url,
                    "/api/biped",
                    params={"cmd": "stand", "duration_ms": int(pose_duration_ms), "interp_steps": int(interp_steps), "hold_ms": int(hold_ms)},
                    refresh=True,
                    rerun=True,
                )
        with top_actions[1]:
            if st.button("Play forward cycle", use_container_width=True, key="biped_walk_forward"):
                execute_mixed_biped_sequence(
                    base_url,
                    BIPED_SEQUENCE_PRESETS["Tiptoe forward cycle"],
                    poses_by_name,
                    firmware_pose_names,
                    int(pose_duration_ms),
                    int(interp_steps),
                    int(hold_ms),
                    int(repeat_count),
                    refresh=True,
                )
                st.rerun()
        with top_actions[2]:
            if st.button("Play backward cycle", use_container_width=True, key="biped_walk_backward"):
                execute_mixed_biped_sequence(
                    base_url,
                    BIPED_SEQUENCE_PRESETS["Tiptoe backward cycle"],
                    poses_by_name,
                    firmware_pose_names,
                    int(pose_duration_ms),
                    int(interp_steps),
                    int(hold_ms),
                    int(repeat_count),
                    refresh=True,
                )
                st.rerun()

        synth_cols = st.columns([2, 3])
        with synth_cols[0]:
            if st.button("Generate softer shift poses", use_container_width=True, key="biped_generate_soft_shifts"):
                try:
                    limits = {
                        joint_name: (int(state["joints"][joint_name].min_angle), int(state["joints"][joint_name].max_angle))
                        for joint_name in BIPED_JOINT_ORDER
                    }
                    generated = build_soft_shift_poses(poses_by_name, limits)
                    for pose_name, pose_values in generated.items():
                        params = {"cmd": "save_pose", "name": pose_name}
                        params.update({joint_name: int(pose_values[joint_name]) for joint_name in BIPED_JOINT_ORDER})
                        if not run_device_action(base_url, "/api/biped", params=params, refresh=False, rerun=False):
                            raise RuntimeError(st.session_state.get("device_error", f"Failed saving {pose_name}"))
                    sync_state(base_url)
                    st.success("Updated `shift_left` and `shift_right` as softer support-transfer poses derived from the stable forward poses.")
                    st.rerun()
                except Exception as exc:
                    st.error(str(exc))
        with synth_cols[1]:
            st.caption(
                "This keeps both shift poses close to the gait base pose and moves each support side only partway toward its stable forward pose. If `crouch_stand` exists, it becomes the base for softer tip-toe walking."
            )

        st.divider()
        pose_select_left, pose_select_mid = st.columns([2, 3])
        with pose_select_left:
            selected_pose_name = st.selectbox("Pose to edit", pose_names, key="biped_selected_pose")
        current_pose = poses_by_name[selected_pose_name]

        with pose_select_mid:
            selected_scope = "virtual (dashboard-only)" if selected_pose_name not in firmware_pose_names else "firmware-backed"
            st.caption(f"Save a manually balanced pose once, then reuse it in sequences. `Capture current` is the fastest way to calibrate from live servo positions. Selected pose is `{selected_scope}`.")

        editor_cols = st.columns(3)
        angle_values: dict[str, int] = {}
        for index, joint_name in enumerate(BIPED_JOINT_ORDER):
            joint = state["joints"][joint_name]
            with editor_cols[index % 3]:
                angle_values[joint_name] = int(
                    st.number_input(
                        joint.label,
                        min_value=int(joint.min_angle),
                        max_value=int(joint.max_angle),
                        value=int(current_pose.get(joint_name, joint.position)),
                        step=1,
                        key=f"biped_pose_{selected_pose_name}_{joint_name}",
                    )
                )

        pose_actions = st.columns(3)
        with pose_actions[0]:
            if st.button("Capture current into pose", use_container_width=True, key="biped_capture_current"):
                if selected_pose_name in firmware_pose_names:
                    run_device_action(
                        base_url,
                        "/api/biped",
                        params={"cmd": "save_pose", "name": selected_pose_name, "current": 1},
                        refresh=True,
                        rerun=True,
                    )
                else:
                    save_virtual_biped_pose(
                        selected_pose_name,
                        {joint_name: int(state["joints"][joint_name].position) for joint_name in BIPED_JOINT_ORDER},
                    )
                    st.success(f"Captured current joint positions into virtual pose `{selected_pose_name}`.")
                    st.rerun()
        with pose_actions[1]:
            if st.button("Save typed pose", use_container_width=True, key="biped_save_pose"):
                if selected_pose_name in firmware_pose_names:
                    params = {"cmd": "save_pose", "name": selected_pose_name}
                    params.update({joint_name: int(angle_values[joint_name]) for joint_name in BIPED_JOINT_ORDER})
                    run_device_action(base_url, "/api/biped", params=params, refresh=True, rerun=True)
                else:
                    save_virtual_biped_pose(selected_pose_name, {joint_name: int(angle_values[joint_name]) for joint_name in BIPED_JOINT_ORDER})
                    st.success(f"Saved virtual pose `{selected_pose_name}` locally.")
                    st.rerun()
        with pose_actions[2]:
            if st.button("Run selected pose", use_container_width=True, key="biped_run_pose"):
                if selected_pose_name in firmware_pose_names:
                    run_device_action(
                        base_url,
                        "/api/biped",
                        params={
                            "cmd": "run_pose",
                            "name": selected_pose_name,
                            "duration_ms": int(pose_duration_ms),
                            "interp_steps": int(interp_steps),
                            "hold_ms": int(hold_ms),
                        },
                        refresh=True,
                        rerun=True,
                    )
                else:
                    execute_biped_pose_values(base_url, poses_by_name[selected_pose_name], int(pose_duration_ms), int(hold_ms), refresh=True)
                    st.rerun()

        st.divider()
        preset_name = st.selectbox("Sequence preset", list(BIPED_SEQUENCE_PRESETS.keys()), key="biped_sequence_preset")
        default_sequence = BIPED_SEQUENCE_PRESETS[preset_name]
        sequence_names = st.text_input(
            "Sequence pose names (comma-separated)",
            value=default_sequence,
            key=f"biped_sequence_names_{preset_name}",
            help="Example: crouch_stand,shift_right,right_forward,crouch_stand,shift_left,left_forward,crouch_stand",
        )
        sequence_actions = st.columns(2)
        with sequence_actions[0]:
            if st.button("Play sequence", use_container_width=True, key="biped_play_sequence"):
                execute_mixed_biped_sequence(
                    base_url,
                    sequence_names,
                    poses_by_name,
                    firmware_pose_names,
                    int(pose_duration_ms),
                    int(interp_steps),
                    int(hold_ms),
                    int(repeat_count),
                    refresh=True,
                )
                st.rerun()
        with sequence_actions[1]:
            st.caption("Suggested workflow: tune `crouch_stand`, `left_forward`, and `right_forward`, then generate softer `shift_left` and `shift_right`, and test the tiptoe forward cycle before changing anything else.")

        table_rows = []
        for pose_name in pose_names:
            row = {"pose": pose_name}
            row.update({state["joints"][joint_name].label: poses_by_name[pose_name].get(joint_name, state["joints"][joint_name].position) for joint_name in BIPED_JOINT_ORDER})
            table_rows.append(row)
        st.dataframe(table_rows, use_container_width=True, hide_index=True)

        st.divider()
        st.markdown("**Phase Walk**")
        st.caption("Host-side phase-based walk model. This does not touch firmware pose slots and executes interpolated frames directly over the existing joint API.")
        phase_cols = st.columns(3)
        with phase_cols[0]:
            phase_steps = st.number_input("Steps", min_value=1, max_value=20, value=3, step=1, key="biped_phase_walk_steps")
        with phase_cols[1]:
            phase_substeps = st.number_input("Substeps per phase", min_value=1, max_value=20, value=6, step=1, key="biped_phase_walk_substeps")
        with phase_cols[2]:
            phase_delay_ms = st.number_input("Joint delay (ms)", min_value=0, max_value=1000, value=80, step=10, key="biped_phase_walk_delay")
        if st.button("Walk Forward", use_container_width=True, key="biped_phase_walk_forward"):
            try:
                phase_walk_forward(
                    steps=int(phase_steps),
                    esp_ip=base_url,
                    substeps=int(phase_substeps),
                    inter_delay_ms=int(phase_delay_ms),
                )
                sync_state(base_url)
                st.success("Phase walk execution finished.")
                st.rerun()
            except Exception as exc:
                st.error(str(exc))


def render_biped_vision_section(base_url: str, state: dict) -> None:
    is_biped = str(state.get("device_type", "")) == "robot_biped" or "Biped" in str(state.get("device_model", ""))
    if not is_biped:
        return

    with st.expander("Vision Ball Tracking", expanded=False):
        try:
            from biped_vision_tracking import RobotBallTracker, TrackingConfig
            from biped_vision_camera import discover_ip_webcams, fetch_ip_webcam_frame, get_cached_ip_webcam_url
        except Exception as exc:
            st.error(f"Vision dependencies are not available: {exc}")
            st.caption("Install `opencv-python` and `numpy` in the dashboard environment to enable this section.")
            return

        st.caption(
            "Uses an Android IP Webcam feed to detect the robot and the ball, estimate their relative position, and trigger your saved biped sequences until the robot reaches the ball."
        )

        source_mode = st.radio("Camera source", ["Manual URL", "Discover IP Webcam"], horizontal=True, key="biped_vision_source_mode")
        camera_url_default = st.session_state.get("biped_vision_camera_url") or get_cached_ip_webcam_url() or "http://192.168.1.141:8080/shot.jpg"
        camera_url = camera_url_default

        if source_mode == "Discover IP Webcam":
            discovery_cols = st.columns([1, 2])
            with discovery_cols[0]:
                discovery_mode = st.selectbox("Discovery mode", ["quick", "full"], key="biped_vision_discovery_mode")
                if st.button("Refresh IP cameras", use_container_width=True, key="biped_vision_refresh_cameras"):
                    with st.spinner("Scanning for Android IP Webcam..."):
                        st.session_state.biped_vision_sources = discover_ip_webcams(discovery_mode)
            sources = st.session_state.get("biped_vision_sources", [])
            if sources:
                labels = [f"{source.name} - {source.value}" for source in sources]
                selected_label = st.selectbox("Discovered cameras", labels, key="biped_vision_selected_source")
                selected_source = sources[labels.index(selected_label)]
                camera_url = str(selected_source.value)
            else:
                st.info("No discovered IP webcams yet. You can still use a manual URL below.")
                camera_url = st.text_input("Fallback camera URL", value=camera_url_default, key="biped_vision_manual_fallback")
        else:
            camera_url = st.text_input("IP Webcam shot URL", value=camera_url_default, key="biped_vision_manual_url")

        st.session_state.biped_vision_camera_url = camera_url

        config_cols = st.columns(4)
        with config_cols[0]:
            marker_color = st.selectbox("Robot marker color", ["Blue", "Red", "Yellow", "Orange", "Purple"], key="biped_vision_marker_color")
        with config_cols[1]:
            ball_color = st.selectbox("Ball color", ["Red", "Orange", "Yellow", "Green", "Blue"], index=0, key="biped_vision_ball_color")
        with config_cols[2]:
            alignment_deadband_px = st.number_input("Alignment deadband (px)", min_value=5, max_value=300, value=45, step=5, key="biped_vision_deadband")
        with config_cols[3]:
            approach_distance_cm = st.number_input("Approach distance (cm)", min_value=5.0, max_value=200.0, value=55.0, step=1.0, key="biped_vision_approach")
        config_cols_b = st.columns(4)
        with config_cols_b[0]:
            kick_distance_cm = st.number_input("Collision distance (cm)", min_value=1.0, max_value=100.0, value=18.0, step=1.0, key="biped_vision_kick")
        with config_cols_b[1]:
            ball_diameter_cm = st.number_input("Ball diameter (cm)", min_value=1.0, max_value=50.0, value=22.0, step=0.5, key="biped_vision_ball_size")
        with config_cols_b[2]:
            min_robot_area_px = st.number_input("Min robot area (px)", min_value=100, max_value=20000, value=900, step=100, key="biped_vision_robot_area")
        with config_cols_b[3]:
            min_ball_area_px = st.number_input("Min ball area (px)", min_value=20, max_value=10000, value=120, step=10, key="biped_vision_ball_area")

        control_cols = st.columns(3)
        with control_cols[0]:
            vision_pose_duration_ms = st.number_input("Action pose duration (ms)", min_value=0, max_value=5000, value=300, step=25, key="biped_vision_pose_duration")
            vision_interp_steps = st.number_input("Action interpolation steps", min_value=1, max_value=100, value=16, step=1, key="biped_vision_interp")
        with control_cols[1]:
            vision_hold_ms = st.number_input("Action hold (ms)", min_value=0, max_value=2000, value=60, step=10, key="biped_vision_hold")
            autonomous_cycles = st.number_input("Autonomous pursuit cycles", min_value=1, max_value=20, value=5, step=1, key="biped_vision_cycles")
        with control_cols[2]:
            post_action_pause_ms = st.number_input("Pause between cycles (ms)", min_value=0, max_value=5000, value=250, step=50, key="biped_vision_pause")
            distance_increase_tolerance_cm = st.number_input(
                "Distance increase tolerance (cm)",
                min_value=0.0,
                max_value=50.0,
                value=2.0,
                step=0.5,
                key="biped_vision_distance_tolerance",
            )

        mapping = {
            "TURN_LEFT": st.text_input("TURN_LEFT sequence", value="shift_left,stand", key="biped_vision_turn_left"),
            "TURN_RIGHT": st.text_input("TURN_RIGHT sequence", value="shift_right,stand", key="biped_vision_turn_right"),
            "APPROACH": st.text_input("APPROACH sequence", value="stand,left_forward,right_forward,stand", key="biped_vision_approach_seq"),
            "ALIGN_AND_STEP": st.text_input("ALIGN_AND_STEP sequence", value="stand,left_forward,right_forward,stand", key="biped_vision_align_seq"),
            "KICK_READY": st.text_input("KICK_READY sequence", value="stand,left_forward,right_forward,stand", key="biped_vision_kick_seq"),
        }

        tracker = RobotBallTracker(
            TrackingConfig(
                robot_marker_color=marker_color,
                ball_color=ball_color,
                ball_diameter_cm=float(ball_diameter_cm),
                kick_distance_cm=float(kick_distance_cm),
                approach_distance_cm=float(approach_distance_cm),
                alignment_deadband_px=int(alignment_deadband_px),
                min_robot_area_px=int(min_robot_area_px),
                min_ball_area_px=int(min_ball_area_px),
            )
        )

        def process_one_frame() -> dict | None:
            try:
                frame = fetch_ip_webcam_frame(camera_url)
                annotated, info = tracker.process_frame(frame)
                st.session_state.biped_vision_last_result = {
                    "image": annotated[:, :, ::-1],
                    "info": info,
                    "camera_url": camera_url,
                }
                return info
            except Exception as exc:
                st.session_state.biped_vision_error = str(exc)
                return None

        button_cols = st.columns(3)
        with button_cols[0]:
            if st.button("Process frame", use_container_width=True, key="biped_vision_process_frame"):
                st.session_state.pop("biped_vision_error", None)
                process_one_frame()
        with button_cols[1]:
            if st.button("Execute next action from state", use_container_width=True, key="biped_vision_step_action"):
                st.session_state.pop("biped_vision_error", None)
                info = process_one_frame()
                if info is not None:
                    previous_distance_cm = st.session_state.get("biped_vision_previous_distance_cm")
                    action_state = choose_vision_action_state(info, previous_distance_cm, float(distance_increase_tolerance_cm))
                    sequence = determine_biped_vision_sequence(action_state, mapping)
                    if sequence:
                        execute_biped_sequence(
                            base_url,
                            sequence,
                            int(vision_pose_duration_ms),
                            int(vision_interp_steps),
                            int(vision_hold_ms),
                            repeat_count=1,
                            refresh=True,
                        )
                    if isinstance(info.get("distance_cm"), (int, float)):
                        st.session_state.biped_vision_previous_distance_cm = float(info["distance_cm"])
                    else:
                        st.info(f"No movement mapped for state `{action_state}`.")
        with button_cols[2]:
            if st.button("Run autonomous pursuit", use_container_width=True, key="biped_vision_autonomous"):
                st.session_state.pop("biped_vision_error", None)
                action_log: list[str] = []
                previous_distance_cm = st.session_state.get("biped_vision_previous_distance_cm")
                for cycle_index in range(int(autonomous_cycles)):
                    info = process_one_frame()
                    if info is None:
                        action_log.append(f"Cycle {cycle_index + 1}: frame error")
                        break
                    state_name = str(info.get("state", ""))
                    action_state = choose_vision_action_state(info, previous_distance_cm, float(distance_increase_tolerance_cm))
                    action_log.append(
                        f"Cycle {cycle_index + 1}: state={state_name} action={action_state} distance_cm={info.get('distance_cm')} dx_cm={info.get('dx_cm')}"
                    )
                    if action_state == "KICK_READY":
                        sequence = determine_biped_vision_sequence("KICK_READY", mapping)
                        if sequence:
                            execute_biped_sequence(
                                base_url,
                                sequence,
                                int(vision_pose_duration_ms),
                                int(vision_interp_steps),
                                int(vision_hold_ms),
                                repeat_count=1,
                                refresh=True,
                            )
                            action_log.append(f"Cycle {cycle_index + 1}: executed kick sequence")
                        break
                    sequence = determine_biped_vision_sequence(action_state, mapping)
                    if not sequence:
                        action_log.append(f"Cycle {cycle_index + 1}: no action for state {action_state}")
                        break
                    execute_biped_sequence(
                        base_url,
                        sequence,
                        int(vision_pose_duration_ms),
                        int(vision_interp_steps),
                        int(vision_hold_ms),
                        repeat_count=1,
                        refresh=True,
                    )
                    if isinstance(info.get("distance_cm"), (int, float)):
                        previous_distance_cm = float(info["distance_cm"])
                        st.session_state.biped_vision_previous_distance_cm = previous_distance_cm
                    if int(post_action_pause_ms) > 0:
                        time.sleep(int(post_action_pause_ms) / 1000.0)
                st.session_state.biped_vision_action_log = action_log

        if "biped_vision_error" in st.session_state:
            st.error(st.session_state.biped_vision_error)

        last_result = st.session_state.get("biped_vision_last_result")
        if last_result:
            st.image(last_result["image"], caption=f"Tracked frame from {last_result.get('camera_url', camera_url)}", use_container_width=True)
            st.json(last_result["info"])

        action_log = st.session_state.get("biped_vision_action_log")
        if action_log:
            st.code("\n".join(action_log), language="text")


def wifi_controls(base_url: str, wifi: WifiState) -> None:
    info_left, info_mid, info_right = st.columns(3)
    with info_left:
        st.write(f"AP SSID: `{wifi.ap_ssid}`")
        st.write(f"AP IP: `{wifi.ap_ip}`")
        st.write(f"AP active: `{wifi.ap_active}`")
    with info_mid:
        st.write(f"Station SSID: `{wifi.sta_ssid or 'not set'}`")
        st.write(f"Station connected: `{wifi.sta_connected}`")
        st.write(f"Station status: `{wifi.sta_status}`")
    with info_right:
        st.write(f"Station IP: `{wifi.sta_ip or 'not connected'}`")
        st.write(f"Hostname: `{wifi.hostname}`")
        st.write(f"mDNS: `{wifi.mdns_hostname or 'inactive'}`")

    status_left, status_mid = st.columns(2)
    with status_left:
        st.caption(f"Last Wi-Fi result: `{wifi.last_result or 'unknown'}`")
    with status_mid:
        st.caption(f"Last Wi-Fi failure: `{wifi.last_failure or 'none'}`")

    form_left, form_mid = st.columns(2)
    with form_left:
        hostname = st.text_input("Hostname", value=wifi.hostname, key="wifi_hostname")
        ap_ssid = st.text_input("AP SSID", value=wifi.ap_ssid, key="wifi_ap_ssid")
        ap_password = st.text_input("AP password", value="robotarm123", type="password", key="wifi_ap_password")
    with form_mid:
        sta_ssid = st.text_input("Station SSID", value=wifi.sta_ssid, key="wifi_sta_ssid")
        sta_password = st.text_input("Station password", value="", type="password", key="wifi_sta_password")

    button_left, button_mid, button_right = st.columns(3)
    with button_left:
        if st.button("Save Wi-Fi settings", use_container_width=True):
            run_device_action(
                base_url,
                "/api/wifi",
                params={
                    "cmd": "set",
                    "hostname": hostname,
                    "ap_ssid": ap_ssid,
                    "ap_password": ap_password,
                    "sta_ssid": sta_ssid,
                    "sta_password": sta_password,
                },
                refresh=True,
                rerun=True,
            )
    with button_mid:
        if st.button("Reconnect Wi-Fi", use_container_width=True):
            run_device_action(base_url, "/api/wifi", params={"cmd": "reconnect"}, refresh=True, rerun=True)
    with button_right:
        if st.button("Reboot ESP32", use_container_width=True):
            run_device_action(base_url, "/api/system", params={"cmd": "reboot"})
            st.info("Reboot requested. Wait a few seconds, then refresh.")


def controller_controls(base_url: str, controller: ControllerState) -> None:
    status_left, status_mid, status_right = st.columns(3)
    with status_left:
        st.write(f"Subsystem: `{controller.state}`")
        st.write(f"ESP32 Bluetooth MAC: `{controller.esp32_bt_mac}`")
    with status_mid:
        st.write(f"Connected: `{controller.connected}`")
        st.write(f"Battery: `{controller.battery}%`")
    with status_right:
        st.write(f"Pairing mode: `{controller.allow_new_connections}`")
        st.write(f"Reconnect active: `{controller.reconnect_in_progress}`")

    if controller.status_text:
        st.info(controller.status_text)
    if controller.last_error:
        st.warning(f"Last controller issue: `{controller.last_error}`")

    subsystem_left, subsystem_mid = st.columns(2)
    with subsystem_left:
        enabled = st.toggle(
            "Controller input enabled",
            value=controller.enabled,
            help="Disable this while tuning joints from the UI to avoid conflicting commands.",
        )
        if enabled != controller.enabled:
            run_device_action(base_url, "/api/ps4", params={"cmd": "enable", "value": 1 if enabled else 0}, refresh=True, rerun=True)
    with subsystem_mid:
        pair_mode = st.toggle(
            "Pairing / discovery mode",
            value=controller.allow_new_connections,
            help="Turn this on for first-time pairing, then hold SHARE + PS on the controller until it flashes quickly.",
        )
        if pair_mode != controller.allow_new_connections:
            run_device_action(base_url, "/api/ps4", params={"cmd": "pair_mode", "value": 1 if pair_mode else 0}, refresh=True, rerun=True)

    current_left, remembered_right = st.columns(2)
    with current_left:
        st.markdown("**Current controller**")
        st.write(f"Name: `{controller.controller_name or 'none'}`")
        st.write(f"Type: `{controller.controller_type or 'unknown'}`")
        st.write(f"Address: `{controller.controller_bt_addr or 'n/a'}`")
    with remembered_right:
        st.markdown("**Remembered controller**")
        st.write(f"Name: `{controller.remembered_name or 'none'}`")
        st.write(f"Type: `{controller.remembered_type or 'unknown'}`")
        st.write(f"Address: `{controller.remembered_bt_addr or 'n/a'}`")

    pair_left, pair_mid = st.columns(2)
    with pair_left:
        if st.button("Remember current controller", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "remember_current"}, refresh=True, rerun=True)
    with pair_mid:
        if controller.allow_new_connections:
            st.caption("Pairing mode is active. Put the controller in pairing mode now and the first successful connection will become the remembered target.")
        elif controller.remembered_bt_addr:
            st.caption("Pairing mode is off. Turning the remembered controller on should reconnect it automatically.")
        else:
            st.caption("No remembered controller yet. Enable pairing mode to bond one.")

    home_left, home_mid = st.columns(2)
    with home_left:
        home_all_button = st.selectbox(
            "Home all motors button",
            BUTTON_OPTIONS,
            index=option_index(BUTTON_OPTIONS, controller.home_all_button),
            key="controller_home_all_button",
            format_func=lambda item: item[0],
        )
        if st.button("Apply home-all button", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "home_button", "value": home_all_button[1]}, refresh=True, rerun=True)
    with home_mid:
        st.write(f"Configured home-all button: `{controller.home_all_button}`")
        st.caption("Pressing this button on the controller will send every joint to its configured home angle once per button press.")

    feedback_left, feedback_mid = st.columns(2)
    with feedback_left:
        st.write("Controller feedback")
        led_r = st.slider("LED red", 0, 255, controller.led_r)
        led_g = st.slider("LED green", 0, 255, controller.led_g)
        led_b = st.slider("LED blue", 0, 255, controller.led_b)
        if st.button("Apply LED color", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "led", "r": led_r, "g": led_g, "b": led_b}, refresh=True, rerun=True)
    with feedback_mid:
        rumble_force = st.slider("Rumble force", 0, 255, controller.rumble_force)
        rumble_duration = st.slider("Rumble duration", 0, 255, controller.rumble_duration)
        if st.button("Apply rumble", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "rumble", "force": rumble_force, "duration": rumble_duration}, refresh=True, rerun=True)

    calibration_left, calibration_mid = st.columns(2)
    with calibration_left:
        axis_deadzone = st.slider("Stick deadzone", 0, 200, controller.axis_deadzone)
        if st.button("Apply deadzone", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "deadzone", "value": axis_deadzone}, refresh=True, rerun=True)
    with calibration_mid:
        st.write(
            f"Centers: LX `{controller.axis_center_lx}` | LY `{controller.axis_center_ly}` | "
            f"RX `{controller.axis_center_rx}` | RY `{controller.axis_center_ry}`"
        )
        if st.button("Capture stick centers", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "calibrate_center"}, refresh=True, rerun=True)

    with st.expander("Advanced recovery"):
        recovery_left, recovery_mid, recovery_right = st.columns(3)
        with recovery_left:
            if st.button("Disconnect current controller", use_container_width=True):
                run_device_action(base_url, "/api/ps4", params={"cmd": "disconnect"}, refresh=True, rerun=True)
        with recovery_mid:
            if st.button("Forget remembered controller", use_container_width=True):
                run_device_action(base_url, "/api/ps4", params={"cmd": "forget_target"}, refresh=True, rerun=True)
        with recovery_right:
            if st.button("Forget all controller bond data", use_container_width=True):
                run_device_action(base_url, "/api/ps4", params={"cmd": "forget"}, refresh=True, rerun=True)
        st.caption("Use recovery actions only if normal reconnect fails. Bluepad32 handles reconnection from stored Bluetooth keys when pairing mode is off.")


def joint_controls(base_url: str, joint: JointState) -> None:
    with st.expander(joint.label, expanded=False):
        st.caption(f"API joint id: `{joint.name}`")
        is_continuous = joint.motor_type == "continuous_360"

        def clamp(value: int, low: int, high: int) -> int:
            return max(low, min(high, int(value)))

        command_label = "Current speed command" if is_continuous else "Current logical angle"
        move_label = "Speed command (%)" if is_continuous else "Logical angle target"
        min_label = "Min logical angle"
        max_label = "Max logical angle"
        home_label = "Home logical angle"
        space_label = "speed percent" if is_continuous else "logical servo-angle degrees"
        home_button_label = "Stop" if is_continuous else "Home"

        left, right, actions = st.columns([2, 2, 1])

        with right:
            pin = st.number_input("Pin", min_value=0, max_value=255, value=joint.pin, key=f"{joint.name}_pin")
            motor_type = st.toggle("Continuous 360 motor", value=is_continuous, key=f"{joint.name}_motor_type")

        range_min = -100 if motor_type else 0
        range_max = 100 if motor_type else 180

        safe_min = clamp(joint.min_angle, range_min, range_max)
        safe_max = clamp(joint.max_angle, range_min, range_max)

        if safe_min > safe_max:
            safe_min, safe_max = range_min, range_max

        safe_home = clamp(joint.home_angle, safe_min, safe_max)
        safe_position = clamp(joint.position, safe_min, safe_max)
        safe_step = max(1, int(joint.step))
        physical_min_angle = int(joint.physical_min_angle)
        physical_max_angle = int(joint.physical_max_angle)

        with left:
            if is_continuous:
                st.write(
                    f"{command_label}: `{joint.position}` | Startup/home target: `{joint.startup_target}` | "
                    f"Raw servo output: `{joint.raw_output}` | Attached: `{joint.attached}`"
                )
            else:
                st.write(
                    f"{command_label}: `{joint.position}` | Current physical angle: `{joint.physical_angle:.1f}` | "
                    f"Physical home angle: `{joint.physical_home_angle:.1f}` | Raw servo output: `{joint.raw_output}` | "
                    f"Attached: `{joint.attached}`"
                )
            st.caption(
                f"Coordinate space: `{space_label}`. Continuous motors use signed speed with neutral stop calibration; positional joints use logical angles."
            )
            move_angle = st.slider(
                f"{joint.name}_move",
                min_value=safe_min,
                max_value=safe_max,
                value=safe_position,
                key=f"{joint.name}_move_slider",
                help=move_label,
                label_visibility="collapsed",
            )
            nudge_amount = st.number_input(
                f"{joint.name}_nudge",
                min_value=-100 if motor_type else -180,
                max_value=100 if motor_type else 180,
                value=safe_step,
                key=f"{joint.name}_nudge_value",
                step=1,
                label_visibility="collapsed",
            )
            if is_continuous:
                test_left, test_mid, test_right = st.columns(3)
                with test_left:
                    if st.button("Reverse", key=f"{joint.name}_reverse_btn", use_container_width=True):
                        run_device_action(base_url, "/api/joint", params={"cmd": "move", "joint": joint.name, "value": -100}, refresh=True, rerun=True)
                with test_mid:
                    if st.button("Stop", key=f"{joint.name}_stop_btn", use_container_width=True):
                        run_device_action(base_url, "/api/joint", params={"cmd": "move", "joint": joint.name, "value": 0}, refresh=True, rerun=True)
                with test_right:
                    if st.button("Forward", key=f"{joint.name}_forward_btn", use_container_width=True):
                        run_device_action(base_url, "/api/joint", params={"cmd": "move", "joint": joint.name, "value": 100}, refresh=True, rerun=True)

        with right:
            if is_continuous:
                st.caption("Continuous motor calibration")
                neutral_output = st.number_input("Neutral output", min_value=0, max_value=180, value=joint.neutral_output, key=f"{joint.name}_neutral_output")
                stop_deadband = st.number_input("Stop deadband", min_value=0, max_value=20, value=joint.stop_deadband, key=f"{joint.name}_stop_deadband")
                max_speed_scale = st.number_input("Max speed scale (%)", min_value=1, max_value=100, value=joint.max_speed_scale, key=f"{joint.name}_max_speed_scale")
                step = st.number_input("Button speed step", min_value=1, max_value=30, value=safe_step, key=f"{joint.name}_step")
                min_angle = -100
                max_angle = 100
                home_angle = 0
            else:
                min_angle = st.number_input(
                    min_label,
                    min_value=range_min,
                    max_value=range_max,
                    value=safe_min,
                    key=f"{joint.name}_min",
                )
                max_angle = st.number_input(
                    max_label,
                    min_value=range_min,
                    max_value=range_max,
                    value=safe_max,
                    key=f"{joint.name}_max",
                )

                if min_angle > max_angle:
                    st.warning("Min cannot be greater than max.")
                    max_angle = min_angle

                home_angle = st.number_input(
                    home_label,
                    min_value=int(min_angle),
                    max_value=int(max_angle),
                    value=clamp(safe_home, int(min_angle), int(max_angle)),
                    key=f"{joint.name}_home",
                )
                physical_min_angle = st.number_input(
                    "Physical angle at logical min",
                    min_value=-360,
                    max_value=360,
                    value=int(joint.physical_min_angle),
                    key=f"{joint.name}_physical_min",
                )
                physical_max_angle = st.number_input(
                    "Physical angle at logical max",
                    min_value=-360,
                    max_value=360,
                    value=int(joint.physical_max_angle),
                    key=f"{joint.name}_physical_max",
                )
                step = st.number_input("Step", min_value=1, max_value=30, value=safe_step, key=f"{joint.name}_step")
                neutral_output = joint.neutral_output
                stop_deadband = joint.stop_deadband
                max_speed_scale = joint.max_speed_scale
            pulse_min = st.number_input("Pulse min", min_value=100, max_value=3000, value=joint.pulse_min, key=f"{joint.name}_pulse_min")
            pulse_max = st.number_input("Pulse max", min_value=200, max_value=4000, value=joint.pulse_max, key=f"{joint.name}_pulse_max")
            invert = st.checkbox("Invert", value=joint.invert, key=f"{joint.name}_invert")
            if is_continuous:
                st.caption(
                    f"Stored on device: neutral `{joint.neutral_output}` | deadband `{joint.stop_deadband}` | "
                    f"max scale `{joint.max_speed_scale}%` | last speed `{joint.stored_position}`"
                )
            else:
                st.caption(
                    f"Stored on device: min `{joint.stored_min_angle}` | max `{joint.stored_max_angle}` | "
                    f"home `{joint.stored_home_angle}` | physical min/max `{joint.stored_physical_min_angle}` / "
                    f"`{joint.stored_physical_max_angle}` | last position `{joint.stored_position}`"
                )
            control_mode = st.selectbox(
                "Controller input mode",
                CONTROL_MODE_OPTIONS,
                index=option_index(CONTROL_MODE_OPTIONS, joint.control_mode),
                key=f"{joint.name}_control_mode",
                format_func=lambda item: item[0],
            )
            axis_source = st.selectbox(
                "Axis source",
                AXIS_SOURCE_OPTIONS,
                index=option_index(AXIS_SOURCE_OPTIONS, joint.axis_source),
                key=f"{joint.name}_axis_source",
                format_func=lambda item: item[0],
            )
            positive_button = st.selectbox(
                "Positive button",
                BUTTON_OPTIONS,
                index=option_index(BUTTON_OPTIONS, joint.positive_button),
                key=f"{joint.name}_positive_button",
                format_func=lambda item: item[0],
            )
            negative_button = st.selectbox(
                "Negative button",
                BUTTON_OPTIONS,
                index=option_index(BUTTON_OPTIONS, joint.negative_button),
                key=f"{joint.name}_negative_button",
                format_func=lambda item: item[0],
            )
            input_invert = st.checkbox("Invert controller input", value=joint.input_invert, key=f"{joint.name}_input_invert")

        with actions:
            if st.button("Move", key=f"{joint.name}_move_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "move", "joint": joint.name, "value": move_angle}, refresh=True, rerun=True)

            if st.button("+", key=f"{joint.name}_plus_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "nudge", "joint": joint.name, "value": int(abs(nudge_amount))}, refresh=True, rerun=True)

            if st.button("-", key=f"{joint.name}_minus_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "nudge", "joint": joint.name, "value": -int(abs(nudge_amount))}, refresh=True, rerun=True)

            if st.button(home_button_label, key=f"{joint.name}_home_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "home", "joint": joint.name}, refresh=True, rerun=True)

            if st.button("Attach", key=f"{joint.name}_attach_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "attach", "joint": joint.name}, refresh=True, rerun=True)

            if st.button("Detach", key=f"{joint.name}_detach_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "detach", "joint": joint.name}, refresh=True, rerun=True)

            if st.button("Apply joint settings", key=f"{joint.name}_apply_settings", use_container_width=True):
                run_device_action(
                    base_url,
                    "/api/joint",
                    params={
                        "cmd": "apply",
                        "joint": joint.name,
                        "pin": int(pin),
                        "motor_type": 1 if motor_type else 0,
                        "min": int(min_angle),
                        "max": int(max_angle),
                        "home": int(home_angle),
                        "step": int(step),
                        "pulse_min": int(pulse_min),
                        "pulse_max": int(pulse_max),
                        "physical_min_angle": int(physical_min_angle),
                        "physical_max_angle": int(physical_max_angle),
                        "neutral_output": int(neutral_output),
                        "stop_deadband": int(stop_deadband),
                        "max_speed_scale": int(max_speed_scale),
                        "invert": 1 if invert else 0,
                        "control_mode": control_mode[1],
                        "axis_source": axis_source[1],
                        "positive_button": positive_button[1],
                        "negative_button": negative_button[1],
                        "input_invert": 1 if input_invert else 0,
                    },
                    refresh=True,
                    rerun=True,
                )


def main() -> None:
    st.set_page_config(
        page_title="Biped Robot Control Console",
        page_icon=str(BRANDING_DIR / "icon-app.png"),
        layout="wide",
    )
    render_brand_header()

    render_sidebar_brand()
    if "robot_state" not in st.session_state:
        auto_discovery_flow()
    device_url = render_connection_sidebar()
    st.session_state.device_url = device_url

    refresh_col, retry_col = st.sidebar.columns(2)
    with refresh_col:
        if st.button("Refresh current device", use_container_width=True):
            connect_device(st.session_state.get("connected_base_url", device_url))
    with retry_col:
        if st.button("Retry discovery", use_container_width=True):
            st.session_state.pop("robot_state", None)
            st.session_state.pop("connected_base_url", None)
            st.session_state.pop("device_error", None)
            st.session_state["discovery_ran"] = False
            st.rerun()

    if st.session_state.get("device_error"):
        st.error(st.session_state["device_error"])

    if "robot_state" not in st.session_state:
        st.info("The biped robot is not connected yet. The dashboard is trying the last known address, mDNS, and local subnet discovery before falling back to manual URL entry.")
        return

    state = st.session_state["robot_state"]
    device_url = st.session_state.get("connected_base_url", device_url)
    is_biped = str(state.get("device_type", "")) == "robot_biped" or "Biped" in str(state.get("device_model", ""))
    if is_biped:
        st.info("Biped mode active. Joint controls reflect the left/right leg mapping, `Home all joints` returns the robot to the configured stand pose, and the arm-only IK workflow is disabled.")
    top_left, top_mid, top_right = st.columns(3)
    with top_left:
        if st.button("Save calibration to flash", use_container_width=True):
            if run_device_action(device_url, "/api/system", params={"cmd": "save"}, refresh=True):
                st.success("Saved to ESP32 flash.")
    with top_mid:
        if st.button("Reload from flash", use_container_width=True):
            run_device_action(device_url, "/api/system", params={"cmd": "load"}, refresh=True, rerun=True)
    with top_right:
        if st.button("Stand pose" if is_biped else "Home all joints", use_container_width=True):
            run_device_action(device_url, "/api/system", params={"cmd": "home_all"}, refresh=True, rerun=True)

    render_sequence_recorder_section(device_url)
    render_ik_path_section(device_url)
    render_biped_gait_section(device_url, state)
    render_biped_vision_section(device_url, state)
    with st.expander("Wi-Fi", expanded=False):
        wifi_controls(device_url, state["wifi"])
    with st.expander("Wireless Controller", expanded=False):
        controller_controls(device_url, state["controller"])
    for joint_name in state["joints"]:
        joint_controls(device_url, state["joints"][joint_name])


if __name__ == "__main__":
    main()
