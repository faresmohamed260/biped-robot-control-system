from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Optional

import cv2
import numpy as np


ROBOT_COLOR_RANGES = {
    "Blue": (((90, 45, 35), (135, 255, 255)),),
    "Red": (((0, 60, 35), (12, 255, 255)), ((170, 60, 35), (179, 255, 255))),
    "Yellow": (((20, 60, 50), (36, 255, 255)),),
    "Orange": (((5, 70, 45), (22, 255, 255)),),
    "Purple": (((128, 45, 35), (165, 255, 255)),),
}

BALL_COLOR_RANGES = {
    "Red": (((0, 80, 45), (12, 255, 255)), ((170, 80, 45), (179, 255, 255))),
    "Orange": (((5, 90, 55), (22, 255, 255)),),
    "Yellow": (((20, 80, 60), (36, 255, 255)),),
    "Green": (((35, 45, 35), (90, 255, 255)),),
    "Blue": (((90, 45, 35), (135, 255, 255)),),
}


@dataclass(frozen=True)
class TrackingConfig:
    robot_marker_color: str = "Blue"
    ball_color: str = "Red"
    ball_diameter_cm: float = 22.0
    kick_distance_cm: float = 18.0
    approach_distance_cm: float = 55.0
    alignment_deadband_px: int = 45
    min_robot_area_px: int = 900
    min_ball_area_px: int = 120


@dataclass(frozen=True)
class Detection:
    label: str
    bbox: tuple[int, int, int, int]
    confidence: float
    method: str

    @property
    def center(self) -> tuple[int, int]:
        x, y, w, h = self.bbox
        return x + w // 2, y + h // 2

    @property
    def bottom_center(self) -> tuple[int, int]:
        x, y, w, h = self.bbox
        return x + w // 2, y + h


class RobotBallTracker:
    def __init__(self, config: TrackingConfig | None = None):
        self.config = config or TrackingConfig()

    def process_frame(self, frame: np.ndarray) -> tuple[np.ndarray, dict]:
        annotated = frame.copy()
        ball = self._detect_ball(frame)
        robot_marker = self._detect_robot_marker(frame)
        robot = self._detect_robot(frame, ball, robot_marker)
        tracking = self._measure_tracking(ball, robot, robot_marker)
        state = self._behavior_state(tracking, ball, robot)

        if robot_marker:
            self._draw_detection(annotated, robot_marker, (0, 255, 255))
        if robot:
            self._draw_detection(annotated, robot, (255, 150, 40))
        if ball:
            self._draw_detection(annotated, ball, (0, 220, 255))
        if ball and robot:
            self._draw_distance(annotated, ball, robot, robot_marker, tracking)
        self._draw_status_panel(annotated, ball, robot, tracking, state)

        info = {
            "ball_detected": ball is not None,
            "robot_detected": robot is not None,
            "state": state,
            "distance_px": tracking.get("distance_px"),
            "distance_cm": tracking.get("distance_cm"),
            "dx_px": tracking.get("dx_px"),
            "dy_px": tracking.get("dy_px"),
            "dx_cm": tracking.get("dx_cm"),
            "dy_cm": tracking.get("dy_cm"),
            "ball_position": ball.center if ball else None,
            "robot_position": (robot_marker.bottom_center if robot_marker else robot.bottom_center) if robot else None,
            "ball_bbox": ball.bbox if ball else None,
            "robot_bbox": robot.bbox if robot else None,
            "robot_marker_detected": robot_marker is not None,
            "robot_marker_bbox": robot_marker.bbox if robot_marker else None,
        }
        return annotated, info

    def _detect_ball(self, frame: np.ndarray) -> Optional[Detection]:
        return self._detect_ball_by_color(frame)

    def _detect_ball_by_color(self, frame: np.ndarray) -> Optional[Detection]:
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_ranges = BALL_COLOR_RANGES.get(self.config.ball_color, BALL_COLOR_RANGES["Red"])
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for lower, upper in color_ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lower), np.array(upper)))
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_area = height * width
        best: Optional[Detection] = None
        best_score = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.config.min_ball_area_px or area > frame_area * 0.08:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w == 0 or h == 0:
                continue
            aspect = w / float(h)
            if aspect < 0.6 or aspect > 1.5:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.33:
                continue
            score = circularity * min(area, 5000)
            if score > best_score:
                best_score = score
                bbox = self._clip_bbox((x, y, w, h), width, height)
                best = Detection("Ball", bbox, min(0.99, circularity), f"{self.config.ball_color} HSV")
        return best

    def _detect_robot(self, frame: np.ndarray, ball: Optional[Detection], marker: Optional[Detection] = None) -> Optional[Detection]:
        if marker is None:
            marker = self._detect_robot_marker(frame)
        foreground = self._foreground_mask(frame, ball)
        if marker:
            seeded = self._foreground_box_near_marker(foreground, marker, frame.shape[:2])
            if seeded:
                return seeded
            return self._expand_marker_box(marker, frame.shape[:2])
        return self._largest_robot_foreground(foreground, frame.shape[:2])

    def _detect_robot_marker(self, frame: np.ndarray) -> Optional[Detection]:
        color_ranges = ROBOT_COLOR_RANGES.get(self.config.robot_marker_color, ROBOT_COLOR_RANGES["Blue"])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for lower, upper in color_ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lower), np.array(upper)))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        height, width = frame.shape[:2]
        best_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best_contour)
        if area < max(80, self.config.min_robot_area_px * 0.12):
            return None
        x, y, w, h = cv2.boundingRect(best_contour)
        return Detection("Robot marker", self._clip_bbox((x, y, w, h), width, height), min(0.99, area / 5000), "HSV marker")

    def _foreground_mask(self, frame: np.ndarray, ball: Optional[Detection]) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green = cv2.inRange(hsv, np.array((34, 30, 25)), np.array((92, 255, 255)))
        foreground = cv2.bitwise_not(green)
        foreground = cv2.medianBlur(foreground, 5)
        kernel = np.ones((9, 9), np.uint8)
        foreground = cv2.morphologyEx(foreground, cv2.MORPH_CLOSE, kernel, iterations=2)
        foreground = cv2.morphologyEx(foreground, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        if ball:
            x, y, w, h = self._pad_bbox(ball.bbox, frame.shape[1], frame.shape[0], pad=8)
            foreground[y : y + h, x : x + w] = 0
        return foreground

    def _foreground_box_near_marker(self, foreground: np.ndarray, marker: Detection, shape: tuple[int, int]) -> Optional[Detection]:
        height, width = shape
        marker_center = marker.center
        contours, _ = cv2.findContours(foreground, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_bbox: Optional[tuple[int, int, int, int]] = None
        best_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.config.min_robot_area_px:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            contains_marker = x <= marker_center[0] <= x + w and y <= marker_center[1] <= y + h
            overlaps_marker = self._iou((x, y, w, h), marker.bbox) > 0.02
            if not contains_marker and not overlaps_marker:
                continue
            if area > best_area:
                best_area = area
                best_bbox = self._clip_bbox((x, y, w, h), width, height)

        if not best_bbox:
            return None
        return Detection("Robot", best_bbox, min(0.99, best_area / 20000), "marker seeded foreground")

    def _expand_marker_box(self, marker: Detection, shape: tuple[int, int]) -> Detection:
        height, width = shape
        _, _, w, h = marker.bbox
        cx, cy = marker.center
        expanded_w = max(int(w * 2.8), 80)
        expanded_h = max(int(h * 4.5), 120)
        expanded_x = cx - expanded_w // 2
        expanded_y = cy - int(expanded_h * 0.35)
        bbox = self._clip_bbox((expanded_x, expanded_y, expanded_w, expanded_h), width, height)
        return Detection("Robot", bbox, marker.confidence, "expanded marker")

    def _largest_robot_foreground(self, foreground: np.ndarray, shape: tuple[int, int]) -> Optional[Detection]:
        height, width = shape
        contours, _ = cv2.findContours(foreground, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best: Optional[Detection] = None
        best_score = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.config.min_robot_area_px:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w == 0 or h == 0:
                continue
            aspect = w / float(h)
            if aspect < 0.18 or aspect > 2.6 or h < height * 0.12:
                continue
            score = area * (1.0 - min(abs(aspect - 0.7), 0.7))
            if score > best_score:
                best_score = score
                bbox = self._clip_bbox((x, y, w, h), width, height)
                best = Detection("Robot", bbox, min(0.85, area / 25000), "foreground fallback")
        return best

    def _measure_tracking(
        self,
        ball: Optional[Detection],
        robot: Optional[Detection],
        robot_marker: Optional[Detection] = None,
    ) -> dict[str, Optional[float]]:
        if not ball or not robot:
            return {"distance_px": None, "distance_cm": None, "dx_px": None, "dy_px": None, "dx_cm": None, "dy_cm": None}
        ball_point = ball.bottom_center
        robot_point = robot_marker.bottom_center if robot_marker else robot.bottom_center
        dx_px = float(ball_point[0] - robot_point[0])
        dy_px = float(ball_point[1] - robot_point[1])
        distance_px = hypot(dx_px, dy_px)
        ball_diameter_px = max(1.0, (ball.bbox[2] + ball.bbox[3]) / 2.0)
        cm_per_px = self.config.ball_diameter_cm / ball_diameter_px
        return {
            "distance_px": distance_px,
            "distance_cm": distance_px * cm_per_px,
            "dx_px": dx_px,
            "dy_px": dy_px,
            "dx_cm": dx_px * cm_per_px,
            "dy_cm": dy_px * cm_per_px,
        }

    def _behavior_state(self, tracking: dict[str, Optional[float]], ball: Optional[Detection], robot: Optional[Detection]) -> str:
        if not ball and not robot:
            return "SEARCHING"
        if not ball:
            return "SEARCH_BALL"
        if not robot:
            return "SEARCH_ROBOT"
        dx_px = tracking.get("dx_px") or 0.0
        distance_cm = tracking.get("distance_cm")
        if abs(dx_px) > self.config.alignment_deadband_px:
            return "TURN_RIGHT" if dx_px > 0 else "TURN_LEFT"
        if distance_cm is not None and distance_cm <= self.config.kick_distance_cm:
            return "KICK_READY"
        if distance_cm is not None and distance_cm > self.config.approach_distance_cm:
            return "APPROACH"
        return "ALIGN_AND_STEP"

    def _draw_detection(self, frame: np.ndarray, detection: Detection, color: tuple[int, int, int]) -> None:
        x, y, w, h = detection.bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        label = f"{detection.label}: {detection.confidence:.2f}"
        self._draw_label(frame, label, x, max(22, y), color)

    def _draw_distance(
        self,
        frame: np.ndarray,
        ball: Detection,
        robot: Detection,
        robot_marker: Optional[Detection],
        tracking: dict[str, Optional[float]],
    ) -> None:
        ball_point = ball.bottom_center
        robot_point = robot_marker.bottom_center if robot_marker else robot.bottom_center
        cv2.line(frame, robot_point, ball_point, (0, 210, 255), 2)
        cv2.circle(frame, robot_point, 4, (255, 150, 40), -1)
        cv2.circle(frame, ball_point, 4, (0, 220, 255), -1)
        mid_x = (robot_point[0] + ball_point[0]) // 2
        mid_y = (robot_point[1] + ball_point[1]) // 2
        distance_cm = tracking.get("distance_cm")
        distance_px = tracking.get("distance_px")
        text = f"{distance_cm:.1f} cm" if distance_cm is not None else (f"{distance_px:.0f} px" if distance_px is not None else "distance unknown")
        self._draw_label(frame, text, mid_x, mid_y, (0, 135, 210))

    def _draw_status_panel(self, frame: np.ndarray, ball: Optional[Detection], robot: Optional[Detection], tracking: dict[str, Optional[float]], state: str) -> None:
        lines = [f"State: {state}"]
        if ball:
            lines.append(f"Ball: {ball.method}")
        if robot:
            lines.append(f"Robot: {robot.method}")
        distance_cm = tracking.get("distance_cm")
        dx_cm = tracking.get("dx_cm")
        dy_cm = tracking.get("dy_cm")
        if distance_cm is not None and dx_cm is not None and dy_cm is not None:
            lines.append(f"Distance: {distance_cm:.1f} cm")
            lines.append(f"dx/dy: {dx_cm:.1f} / {dy_cm:.1f} cm")
        panel_h = 28 + 22 * len(lines)
        cv2.rectangle(frame, (10, 10), (350, panel_h), (15, 15, 15), -1)
        cv2.rectangle(frame, (10, 10), (350, panel_h), (70, 70, 70), 1)
        for index, line in enumerate(lines):
            cv2.putText(frame, line, (20, 35 + index * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (245, 245, 245), 1, cv2.LINE_AA)

    @staticmethod
    def _draw_label(frame: np.ndarray, text: str, x: int, y: int, color: tuple[int, int, int]) -> None:
        cv2.putText(frame, text, (x + 2, y + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (15, 15, 15), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)

    @staticmethod
    def _clip_bbox(bbox: tuple[int, int, int, int], width: int, height: int) -> tuple[int, int, int, int]:
        x, y, w, h = bbox
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = max(1, min(w, width - x))
        h = max(1, min(h, height - y))
        return x, y, w, h

    @staticmethod
    def _pad_bbox(bbox: tuple[int, int, int, int], width: int, height: int, pad: int) -> tuple[int, int, int, int]:
        x, y, w, h = bbox
        return RobotBallTracker._clip_bbox((x - pad, y - pad, w + pad * 2, h + pad * 2), width, height)

    @staticmethod
    def _iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        x1 = max(ax, bx)
        y1 = max(ay, by)
        x2 = min(ax + aw, bx + bw)
        y2 = min(ay + ah, by + bh)
        if x2 <= x1 or y2 <= y1:
            return 0.0
        intersection = float((x2 - x1) * (y2 - y1))
        union = float(aw * ah + bw * bh - intersection)
        return 0.0 if union <= 0.0 else intersection / union
