from __future__ import annotations

import time
from typing import Iterable

import requests

JOINT_ORDER = ["base", "wrist_rotate", "shoulder", "wrist_pitch", "elbow", "gripper"]

ANCHORS = {
    "stand": {
        "base": 80,
        "shoulder": 90,
        "elbow": 110,
        "wrist_pitch": 30,
        "wrist_rotate": 40,
        "gripper": 135,
    },
    "crouch_stand": {
        "base": 70,
        "shoulder": 115,
        "elbow": 115,
        "wrist_pitch": 0,
        "wrist_rotate": 60,
        "gripper": 140,
    },
    "left_forward": {
        "base": 110,
        "shoulder": 100,
        "elbow": 180,
        "wrist_pitch": 60,
        "wrist_rotate": 40,
        "gripper": 135,
    },
    "right_forward": {
        "base": 80,
        "shoulder": 55,
        "elbow": 110,
        "wrist_pitch": 15,
        "wrist_rotate": 10,
        "gripper": 66,
    },
    "shift_left": {
        "base": 92,
        "shoulder": 94,
        "elbow": 138,
        "wrist_pitch": 25,
        "wrist_rotate": 40,
        "gripper": 135,
    },
    "shift_right": {
        "base": 80,
        "shoulder": 90,
        "elbow": 110,
        "wrist_pitch": 21,
        "wrist_rotate": 28,
        "gripper": 107,
    },
}

FORWARD_BIAS = {
    "elbow": +10,
    "gripper": -8,
}

PHASE_SEQUENCE = [
    "crouch_stand",
    "shift_right",
    "right_forward",
    "crouch_stand_forward_bias",
    "shift_left",
    "left_forward",
]


def clamp_pose(pose: dict[str, int]) -> dict[str, int]:
    return {joint: max(0, min(180, int(value))) for joint, value in pose.items()}


def crouch_stand_forward_bias() -> dict[str, int]:
    pose = dict(ANCHORS["crouch_stand"])
    for joint, delta in FORWARD_BIAS.items():
        pose[joint] = int(pose[joint]) + int(delta)
    return clamp_pose(pose)


def get_phase_pose(phase_name: str) -> dict[str, int]:
    if phase_name == "crouch_stand_forward_bias":
        return crouch_stand_forward_bias()
    return dict(ANCHORS[phase_name])


def lerp_pose(pose_a: dict[str, int], pose_b: dict[str, int], t: float) -> dict[str, int]:
    return {
        joint: round(int(pose_a[joint]) + float(t) * (int(pose_b[joint]) - int(pose_a[joint])))
        for joint in pose_a
    }


def iter_phase_frames(substeps: int = 6) -> Iterable[tuple[str, int, int, dict[str, int]]]:
    safe_substeps = max(1, int(substeps))
    for phase_index, phase_name in enumerate(PHASE_SEQUENCE):
        pose_a = get_phase_pose(phase_name)
        next_phase_name = PHASE_SEQUENCE[(phase_index + 1) % len(PHASE_SEQUENCE)]
        pose_b = get_phase_pose(next_phase_name)
        for substep_index in range(1, safe_substeps + 1):
            t = substep_index / safe_substeps
            yield phase_name, substep_index, safe_substeps, clamp_pose(lerp_pose(pose_a, pose_b, t))


def execute_pose(pose: dict, esp_ip: str, delay_ms: int = 80) -> None:
    base_url = esp_ip.strip().rstrip("/")
    if not base_url.startswith(("http://", "https://")):
        base_url = f"http://{base_url}"
    session = requests.Session()
    session.trust_env = False
    session.headers.update({"Connection": "close", "User-Agent": "Biped-PhaseWalk/1.0"})
    per_joint_delay_s = max(0, int(delay_ms)) / 1000.0
    for joint_name in JOINT_ORDER:
        if joint_name not in pose:
            continue
        session.get(
            f"{base_url}/api/joint",
            params={"cmd": "move", "joint": joint_name, "value": int(pose[joint_name])},
            timeout=4,
        ).raise_for_status()
        if per_joint_delay_s > 0:
            time.sleep(per_joint_delay_s)


def walk_forward(steps: int, esp_ip: str, substeps: int = 6, inter_delay_ms: int = 80) -> None:
    safe_steps = max(1, int(steps))
    execute_pose(get_phase_pose("crouch_stand"), esp_ip, delay_ms=inter_delay_ms)
    for cycle_index in range(safe_steps):
        print(f"walk_forward cycle {cycle_index + 1}/{safe_steps}")
        for phase_name, substep_index, substep_total, pose in iter_phase_frames(substeps=substeps):
            print(f"  phase={phase_name} substep={substep_index}/{substep_total}")
            execute_pose(pose, esp_ip, delay_ms=inter_delay_ms)
    execute_pose(get_phase_pose("stand"), esp_ip, delay_ms=inter_delay_ms)
