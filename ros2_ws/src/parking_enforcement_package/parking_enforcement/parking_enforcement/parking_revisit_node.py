#!/usr/bin/env python3
"""Revisit mission node for the parking-enforcement demo.

This node reuses the same hardcoded waypoint mission flow as the first pass, but
it loads the first-pass plate log, revisits the same parking spots, rescans the
plate at each spot, and records whether the same plate has overstayed.
"""

from __future__ import annotations

import math

import rclpy

from parking_enforcement.config import (
    DRONE_NS,
    FIRST_PASS_LOG_PATH,
    REVISIT_LOG_PATH,
    REVISIT_OVERSTAY_THRESHOLD_SEC,
    SCAN_AMPLITUDE,
    SCAN_PERIOD_SEC,
    SCAN_TIMEOUT_SEC,
)
from parking_enforcement.mission_utils import quaternion_from_yaw, read_json, serialize_waypoints, write_json
from parking_enforcement.parking_mission_node import ParkingMissionNode, STATE_NAVIGATE


class ParkingRevisitNode(ParkingMissionNode):
    """Fly the revisit pass and compare each parking spot against the first pass."""

    def __init__(self):
        super().__init__(node_name=f'{DRONE_NS}_revisit_mission')
        self.first_pass_payload = read_json(FIRST_PASS_LOG_PATH, {})
        self.first_pass_entries = self._index_first_pass_entries(self.first_pass_payload)
        self.revisit_log = {}

        if self.first_pass_entries:
            self.get_logger().info(
                f'Loaded {len(self.first_pass_entries)} first-pass plate entries from {FIRST_PASS_LOG_PATH}'
            )
        else:
            self.get_logger().warn(
                f'No first-pass entries found at {FIRST_PASS_LOG_PATH}. Revisit will still scan and log results.'
            )

    @staticmethod
    def _index_first_pass_entries(payload: dict) -> dict[int, dict]:
        """Key first-pass entries by waypoint index for quick revisit lookup."""

        indexed = {}
        for entry in payload.get('entries', []):
            try:
                waypoint_index = int(entry['waypoint_index'])
            except (KeyError, TypeError, ValueError):
                continue
            indexed[waypoint_index] = entry
        return indexed

    def _update_scan_vertical(self):
        """Scan vertically until we confirm a plate or time out at the parking spot."""

        if self.scan_center_pose is None or self.scan_start_time is None:
            self.state = STATE_NAVIGATE
            return

        elapsed = self._now_sec() - self.scan_start_time
        position = self.scan_center_pose.pose.position
        z_cmd = position.z + SCAN_AMPLITUDE * math.sin(2.0 * math.pi * elapsed / SCAN_PERIOD_SEC)

        self.target_pose.pose.position.x = position.x
        self.target_pose.pose.position.y = position.y
        self.target_pose.pose.position.z = z_cmd
        self.target_pose.pose.orientation = quaternion_from_yaw(self.scan_yaw)

        if self.confirmed_plate:
            self._record_plate_for_current_waypoint(self.confirmed_plate)
            self._advance_waypoint()
            return

        if elapsed > SCAN_TIMEOUT_SEC:
            self.get_logger().warn(f'[REVISIT] Timeout at waypoint {self.current_wp_index + 1}, moving on')
            self._record_timeout_for_current_waypoint()
            self._advance_waypoint()

    def _record_plate_for_current_waypoint(self, plate: str):
        """Compare the current plate reading against the first-pass result."""

        entry = self._build_revisit_entry(plate, scan_status='confirmed')
        self.revisit_log[str(self.current_wp_index + 1)] = entry
        self.get_logger().info(
            f"[REVISIT] waypoint={entry['waypoint_index']} expected={entry['expected_plate'] or 'NONE'} "
            f"observed={entry['observed_plate']} same={entry['same_plate']} overstayed={entry['overstayed']}"
        )
        self._write_revisit_log()

    def _record_timeout_for_current_waypoint(self):
        """Persist a timeout result so missed plates still show up in the revisit log."""

        entry = self._build_revisit_entry('', scan_status='timeout')
        self.revisit_log[str(self.current_wp_index + 1)] = entry
        self._write_revisit_log()

    def _build_revisit_entry(self, observed_plate: str, scan_status: str) -> dict:
        """Create one revisit-log record for the active waypoint."""

        waypoint_index = self.current_wp_index + 1
        waypoint = self.waypoints[self.current_wp_index]
        first_pass_entry = self.first_pass_entries.get(waypoint_index, {})
        expected_plate = str(first_pass_entry.get('plate', ''))
        first_seen_time = first_pass_entry.get('first_seen_time_sec')

        elapsed_sec = None
        same_plate = False
        overstayed = False
        if first_seen_time is not None:
            try:
                elapsed_sec = round(self._now_sec() - float(first_seen_time), 2)
            except (TypeError, ValueError):
                elapsed_sec = None

        if observed_plate and expected_plate:
            same_plate = observed_plate == expected_plate
            overstayed = bool(
                same_plate and elapsed_sec is not None and elapsed_sec >= REVISIT_OVERSTAY_THRESHOLD_SEC
            )

        return {
            'waypoint_index': waypoint_index,
            'position': [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])],
            'expected_plate': expected_plate,
            'observed_plate': observed_plate,
            'same_plate': same_plate,
            'overstayed': overstayed,
            'elapsed_sec': elapsed_sec,
            'first_seen_time_sec': first_seen_time,
            'revisit_time_sec': round(self._now_sec(), 2),
            'scan_status': scan_status,
        }

    def _write_first_pass_log(self):
        """Redirect the inherited write hook to the revisit output file instead."""

        self._write_revisit_log()

    def _write_revisit_log(self):
        """Write the revisit results so the demo can be inspected after flight."""

        payload = {
            'drone_ns': DRONE_NS,
            'source_first_pass_log': str(FIRST_PASS_LOG_PATH),
            'overstay_threshold_sec': REVISIT_OVERSTAY_THRESHOLD_SEC,
            'waypoints': serialize_waypoints(self.waypoints),
            'entries': [self.revisit_log[key] for key in sorted(self.revisit_log, key=lambda item: int(item))],
        }
        write_json(REVISIT_LOG_PATH, payload)


def main(args=None):
    """Spin the revisit mission node until shutdown."""

    rclpy.init(args=args)
    node = ParkingRevisitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
