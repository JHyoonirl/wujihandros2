#!/usr/bin/env python3
"""Safe four-finger open/close test for WUJI Hand.

The default is a dry run.  Set ``dry_run:=false`` only after the driver is
running and the hand is clear.  Finger 0 (thumb) is held at its measured
values.  For fingers 1..4, the project motion profile commands joints
0, 2, and 3 while joint 1 remains fixed.

Examples::

  ros2 run wujihand_bringup four_finger_cycle.py --ros-args -p dry_run:=true
  ros2 run wujihand_bringup four_finger_cycle.py --ros-args \
    -p hand_name:=hand_0 -p dry_run:=false
"""

import os
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from wujihand_msgs.srv import SetEnabled


NUM_JOINTS = 20
NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
THUMB_INDICES = tuple(range(4))


class FourFingerCycle(Node):
    """Run a bounded position trajectory while leaving the thumb untouched."""

    def __init__(self):
        super().__init__("four_finger_cycle")

        self.declare_parameter("hand_name", "hand_0")
        self.declare_parameter("hand_side", "right")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("amplitude", 0.12)
        self.declare_parameter("direction", 1.0)
        self.declare_parameter("move_duration", 2.5)
        self.declare_parameter("hold_duration", 0.5)
        self.declare_parameter("settle_duration", 0.5)
        self.declare_parameter("final_hold_duration", 0.5)
        self.declare_parameter("cycles", 1)
        self.declare_parameter("rate", 100.0)
        self.declare_parameter("startup_timeout", 5.0)
        self.declare_parameter("disable_timeout", 2.0)
        self.declare_parameter("disable_on_exit", True)
        self.declare_parameter("enable_on_start", True)

        self.hand_name = self.get_parameter("hand_name").value
        self.hand_side = str(self.get_parameter("hand_side").value).lower()
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.direction = float(self.get_parameter("direction").value)
        self.move_duration = float(self.get_parameter("move_duration").value)
        self.hold_duration = float(self.get_parameter("hold_duration").value)
        self.settle_duration = float(self.get_parameter("settle_duration").value)
        self.final_hold_duration = float(self.get_parameter("final_hold_duration").value)
        self.cycles = int(self.get_parameter("cycles").value)
        self.rate = float(self.get_parameter("rate").value)
        self.startup_timeout = float(self.get_parameter("startup_timeout").value)
        self.disable_timeout = float(self.get_parameter("disable_timeout").value)
        self.disable_on_exit = bool(self.get_parameter("disable_on_exit").value)
        self.enable_on_start = bool(self.get_parameter("enable_on_start").value)

        self._validate_parameters()

        self.base = None
        self.started_at = None
        self.wait_started_at = time.monotonic()
        self.done = False
        self.publisher = None
        self.state_subscription = None
        self.joint_lower_limits = None
        self.joint_upper_limits = None
        self._clamp_warning_sent = False
        self.set_enabled_client = None
        self.enable_future = None
        self.enable_started_at = None
        self.motors_ready = self.dry_run or not self.enable_on_start
        self.disable_future = None
        self.disable_started_at = None

        if self.hand_side not in ("left", "right"):
            raise ValueError("hand_side must be 'left' or 'right'")

        if self.dry_run:
            self._load_joint_limits(self._synthetic_joint_names(self.hand_side))
            self.timer = self.create_timer(0.01, self._dry_run_once)
            return

        topic_prefix = f"/{self.hand_name}"
        self.publisher = self.create_publisher(
            JointState, f"{topic_prefix}/joint_commands", qos_profile_sensor_data
        )
        self.state_subscription = self.create_subscription(
            JointState,
            f"{topic_prefix}/joint_states",
            self._on_state,
            qos_profile_sensor_data,
        )
        self.set_enabled_client = self.create_client(
            SetEnabled, f"{topic_prefix}/set_enabled"
        )
        if self.enable_on_start:
            self.enable_started_at = time.monotonic()
        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)
        self.get_logger().info(
            f"Waiting for {topic_prefix}/joint_states; thumb will remain fixed"
        )

    def _validate_parameters(self):
        if not 0.0 <= self.amplitude <= 0.45:
            raise ValueError("amplitude must be between 0.0 and 0.25 rad")
        if self.direction == 0.0:
            raise ValueError("direction must be +1.0 or -1.0")
        self.direction = 1.0 if self.direction > 0.0 else -1.0
        if self.move_duration <= 0.0 or self.hold_duration < 0.0:
            raise ValueError("move_duration must be > 0 and hold_duration >= 0")
        if self.settle_duration < 0.0 or self.final_hold_duration < 0.0:
            raise ValueError("settle and final hold durations must be >= 0")
        if self.cycles < 1:
            raise ValueError("cycles must be >= 1")
        if self.rate <= 0.0:
            raise ValueError("rate must be > 0")
        if self.startup_timeout <= 0.0:
            raise ValueError("startup_timeout must be > 0")
        if self.disable_timeout <= 0.0:
            raise ValueError("disable_timeout must be > 0")

    @staticmethod
    def _smoothstep(value):
        value = max(0.0, min(1.0, value))
        return value * value * (3.0 - 2.0 * value)

    def _dry_run_once(self):
        if self.done:
            return

        moving_indices = [
            finger * JOINTS_PER_FINGER + joint
            for finger in range(1, NUM_FINGERS)
            for joint in (0, 2, 3)
        ]
        peak = self._make_command([0.0] * NUM_JOINTS, self.amplitude)
        self.get_logger().info(
            "Dry run passed: "
            f"thumb_indices={list(THUMB_INDICES)}, "
            f"moving_indices={moving_indices}, "
            f"peak_delta={self.amplitude:.3f} rad, "
            f"max_command={max(abs(value) for value in peak):.3f} rad"
        )
        self.done = True

    @staticmethod
    def _synthetic_joint_names(side):
        return [
            f"{side}_finger{finger + 1}_joint{joint + 1}"
            for finger in range(NUM_FINGERS)
            for joint in range(JOINTS_PER_FINGER)
        ]

    def _load_joint_limits(self, joint_names):
        """Load per-joint limits from the installed WUJI URDF."""
        names = list(joint_names[:NUM_JOINTS])
        if names:
            first_name = names[0].lower()
            if first_name.startswith("left_"):
                self.hand_side = "left"
            elif first_name.startswith("right_"):
                self.hand_side = "right"

        try:
            description_share = get_package_share_directory("wuji_description")
            urdf_path = os.path.join(
                description_share, "urdf", f"{self.hand_side}-ros.urdf"
            )
            root = ET.parse(urdf_path).getroot()
        except (OSError, ET.ParseError, LookupError) as error:
            raise ValueError(f"could not load WUJI URDF limits: {error}") from error

        urdf_limits = {}
        for joint in root.findall("joint"):
            limit = joint.find("limit")
            if joint.attrib.get("type") != "revolute" or limit is None:
                continue
            urdf_limits[joint.attrib["name"]] = (
                float(limit.attrib["lower"]),
                float(limit.attrib["upper"]),
            )

        if len(names) < NUM_JOINTS:
            names = self._synthetic_joint_names(self.hand_side)
        missing = [name for name in names if name not in urdf_limits]
        if missing:
            raise ValueError(
                "URDF has no limits for joint(s): " + ", ".join(missing)
            )

        self.joint_lower_limits = [urdf_limits[name][0] for name in names]
        self.joint_upper_limits = [urdf_limits[name][1] for name in names]
        self.get_logger().info(
            f"Loaded per-joint limits from {self.hand_side}-ros.urdf"
        )

    def _on_state(self, message):
        if (
            not self.motors_ready
            or self.base is not None
            or len(message.position) < NUM_JOINTS
        ):
            return

        self._load_joint_limits(message.name)
        self.base = list(message.position[:NUM_JOINTS])
        self.started_at = time.monotonic()
        self.get_logger().info(
            f"Captured {NUM_JOINTS} joints; running {self.cycles} cycle(s) "
            f"with amplitude={self.amplitude:.3f} rad and direction={self.direction:+.1f}"
        )

    def _trajectory_amount(self, elapsed):
        cycle_duration = 2.0 * self.move_duration + 2.0 * self.hold_duration
        elapsed -= self.settle_duration
        if elapsed < 0.0:
            return 0.0, False

        cycle_index = int(elapsed // cycle_duration)
        if cycle_index >= self.cycles:
            return 0.0, elapsed >= self.cycles * cycle_duration + self.final_hold_duration

        local = elapsed - cycle_index * cycle_duration
        if local < self.move_duration:
            fraction = self._smoothstep(local / self.move_duration)
            return self.amplitude * fraction, False

        local -= self.move_duration
        if local < self.hold_duration:
            return self.amplitude, False

        local -= self.hold_duration
        if local < self.move_duration:
            fraction = self._smoothstep(local / self.move_duration)
            return self.amplitude * (1.0 - fraction), False

        return 0.0, False

    def _make_command(self, base, amount):
        if self.joint_lower_limits is None or self.joint_upper_limits is None:
            raise RuntimeError("joint limits must be loaded before making commands")

        positions = list(base)
        for finger in range(1, NUM_FINGERS):
            offset = finger * JOINTS_PER_FINGER
            # The project motion profile holds joint 1 and moves joints 0, 2, 3.
            positions[offset + 1] = base[offset + 1]
            for joint in (0, 2, 3):
                index = offset + joint
                requested = base[index] + self.direction * amount
                clamped = min(
                    self.joint_upper_limits[index],
                    max(self.joint_lower_limits[index], requested),
                )
                if abs(clamped - requested) > 1e-9 and not self._clamp_warning_sent:
                    self.get_logger().warning(
                        "Trajectory reached a per-joint URDF limit; "
                        "remaining joints will continue within their own limits"
                    )
                    self._clamp_warning_sent = True
                positions[index] = clamped
        return positions

    def _publish(self, positions):
        message = JointState()
        message.position = positions
        self.publisher.publish(message)

    def _on_timer(self):
        if self.done:
            return

        if self.enable_started_at is not None and not self.motors_ready:
            self._process_enable_request()
            return

        if self.disable_started_at is not None:
            self._process_disable_request()
            return

        if self.base is None:
            if time.monotonic() - self.wait_started_at > self.startup_timeout:
                self.get_logger().error(
                    "No 20-joint state received before startup_timeout; "
                    "is the WUJI driver running?"
                )
                self.done = True
            return

        elapsed = time.monotonic() - self.started_at
        amount, finished = self._trajectory_amount(elapsed)
        self._publish(self._make_command(self.base, amount))
        if finished:
            if self.disable_on_exit:
                self.get_logger().info(
                    "Cycle complete; baseline pose sent, disabling all joints"
                )
                self.disable_started_at = time.monotonic()
            else:
                self.get_logger().info(
                    "Cycle complete; baseline pose sent, leaving joints enabled"
                )
                self.done = True

    def _process_enable_request(self):
        """Enable every joint before capturing the baseline pose."""
        elapsed = time.monotonic() - self.enable_started_at

        if self.enable_future is None:
            if self.set_enabled_client.service_is_ready():
                request = SetEnabled.Request()
                request.finger_id = 255
                request.joint_id = 255
                request.enabled = True
                self.enable_future = self.set_enabled_client.call_async(request)
                self.get_logger().info("Requested enable for all joints")
            elif elapsed >= self.disable_timeout:
                self.get_logger().error(
                    "The set_enabled service was not available; cannot enable motors"
                )
                self.done = True
            return

        if self.enable_future.done():
            try:
                response = self.enable_future.result()
            except Exception as error:  # noqa: BLE001 - report service failure
                self.get_logger().error(f"Failed to enable joints: {error}")
                self.done = True
                return

            if response is not None and response.success:
                self.motors_ready = True
                self.enable_started_at = None
                self.get_logger().info(
                    f"All joints enabled: {response.message}; capturing baseline"
                )
            else:
                message = response.message if response is not None else "no response"
                self.get_logger().error(f"Could not enable all joints: {message}")
                self.done = True
        elif elapsed >= self.disable_timeout:
            self.get_logger().error(
                "Timed out waiting for enable response; motors remain disabled"
            )
            self.done = True

    def _process_disable_request(self):
        """Keep the baseline command until all joints are disabled."""
        self._publish(self._make_command(self.base, 0.0))
        elapsed = time.monotonic() - self.disable_started_at

        if self.disable_future is None:
            if self.set_enabled_client.service_is_ready():
                request = SetEnabled.Request()
                request.finger_id = 255
                request.joint_id = 255
                request.enabled = False
                self.disable_future = self.set_enabled_client.call_async(request)
                self.get_logger().info("Requested disable for all joints")
            elif elapsed >= self.disable_timeout:
                self.get_logger().error(
                    "The set_enabled service was not available; motors remain enabled"
                )
                self.done = True
            return

        if self.disable_future.done():
            try:
                response = self.disable_future.result()
            except Exception as error:  # noqa: BLE001 - report service failure
                self.get_logger().error(f"Failed to disable joints: {error}")
                self.done = True
                return

            if response is not None and response.success:
                self.get_logger().info(
                    f"All joints disabled: {response.message}"
                )
            else:
                message = response.message if response is not None else "no response"
                self.get_logger().error(f"Could not disable all joints: {message}")
            self.done = True
        elif elapsed >= self.disable_timeout:
            self.get_logger().error(
                "Timed out waiting for set_enabled response; motors may remain enabled"
            )
            self.done = True

    def return_to_baseline(self, seconds=0.2):
        """Send the captured open pose briefly when interrupted by the user."""
        if self.dry_run or self.base is None or self.publisher is None:
            return
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._publish(self._make_command(self.base, 0.0))
            time.sleep(1.0 / self.rate)

    def disable_motors_blocking(self):
        """Disable every joint synchronously during Ctrl+C cleanup."""
        if self.dry_run or self.set_enabled_client is None or not rclpy.ok():
            return
        if not self.set_enabled_client.wait_for_service(timeout_sec=self.disable_timeout):
            self.get_logger().error(
                "The set_enabled service was not available during shutdown"
            )
            return

        request = SetEnabled.Request()
        request.finger_id = 255
        request.joint_id = 255
        request.enabled = False
        future = self.set_enabled_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.disable_timeout
        )
        if not future.done():
            self.get_logger().error(
                "Timed out disabling joints during shutdown; motors may remain enabled"
            )
            return
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - report service failure
            self.get_logger().error(f"Failed to disable joints during shutdown: {error}")
            return
        if response is not None and response.success:
            self.get_logger().info(f"All joints disabled: {response.message}")
        else:
            message = response.message if response is not None else "no response"
            self.get_logger().error(f"Could not disable all joints: {message}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FourFingerCycle()
    except ValueError as error:
        print(f"four_finger_cycle: invalid parameter: {error}")
        rclpy.shutdown()
        return 2

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        node.get_logger().warning(
            "Interrupted; returning to baseline pose and disabling all joints"
        )
        node.done = True
        node.return_to_baseline()
        if node.disable_on_exit:
            node.disable_motors_blocking()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
