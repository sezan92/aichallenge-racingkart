#!/usr/bin/env python3
"""Hybrid control arbitrator node.

Forwards the command of the default controller (PilotNet) to
/control/command/control_cmd under normal conditions, and switches over to
MPC when a critical situation is detected:

  * LiDAR proximity: nearest obstacle closer than a speed-scaled threshold.
  * Steering disagreement: PilotNet and MPC steering commands diverge.

Switching uses hysteresis (N consecutive critical cycles to engage MPC,
T seconds continuously safe to return) to avoid chattering. Watchdogs guard
against stale/dead controllers, and /control/mpc/stop_request latches a
stopping state in which MPC (or a brake command) decelerates the vehicle.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Empty, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from autoware_auto_control_msgs.msg import AckermannControlCommand

PILOT_NET = "pilot_net"
MPC = "mpc"
FAILSAFE = "failsafe"


class HybridControlArbitrator(Node):
    """Switches between PilotNet and MPC commands for /control/command/control_cmd."""

    def __init__(self):
        super().__init__("hybrid_control_arbitrator")

        # --- Parameter Declaration (same pattern as pilot_net_controller) ---
        self.declare_parameter("default_controller", PILOT_NET)
        self.declare_parameter("publish_rate_hz", 50.0)

        self.declare_parameter("output_cmd_topic", "/control/command/control_cmd")
        self.declare_parameter("pilot_net_cmd_topic", "/control/command/control_cmd_pilot")
        self.declare_parameter("mpc_cmd_topic", "/control/command/control_cmd_mpc")
        self.declare_parameter("lidar_topic", "/sensing/lidar/scan")
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("stop_request_topic", "/control/mpc/stop_request")

        self.declare_parameter("watchdog.pilot_net_timeout_sec", 0.5)
        self.declare_parameter("watchdog.mpc_timeout_sec", 0.25)

        self.declare_parameter("monitor.use_lidar_proximity", True)
        self.declare_parameter("monitor.lidar_min_range_m", 3.0)
        self.declare_parameter("monitor.speed_margin_per_mps", 0.5)
        self.declare_parameter("monitor.use_steer_disagreement", True)
        self.declare_parameter("monitor.steer_disagreement_rad", 0.3)

        self.declare_parameter("hysteresis.engage_critical_cycles", 10)
        self.declare_parameter("hysteresis.release_safe_sec", 3.0)

        self.declare_parameter("failsafe.deceleration_mps2", -1.6)

        # --- Parameter Loading ---
        self._default_controller = self.get_parameter("default_controller").value
        if self._default_controller not in (PILOT_NET, MPC):
            self.get_logger().warn(
                f"default_controller '{self._default_controller}' is invalid; "
                f"falling back to '{PILOT_NET}'")
            self._default_controller = PILOT_NET
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        output_cmd_topic = self.get_parameter("output_cmd_topic").value
        pilot_net_cmd_topic = self.get_parameter("pilot_net_cmd_topic").value
        mpc_cmd_topic = self.get_parameter("mpc_cmd_topic").value
        lidar_topic = self.get_parameter("lidar_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        stop_request_topic = self.get_parameter("stop_request_topic").value

        self._pilot_timeout = float(self.get_parameter("watchdog.pilot_net_timeout_sec").value)
        self._mpc_timeout = float(self.get_parameter("watchdog.mpc_timeout_sec").value)

        self._use_lidar_proximity = bool(self.get_parameter("monitor.use_lidar_proximity").value)
        self._lidar_min_range_m = float(self.get_parameter("monitor.lidar_min_range_m").value)
        self._speed_margin_per_mps = float(self.get_parameter("monitor.speed_margin_per_mps").value)
        self._use_steer_disagreement = bool(self.get_parameter("monitor.use_steer_disagreement").value)
        self._steer_disagreement_rad = float(self.get_parameter("monitor.steer_disagreement_rad").value)

        self._engage_critical_cycles = int(self.get_parameter("hysteresis.engage_critical_cycles").value)
        self._release_safe_sec = float(self.get_parameter("hysteresis.release_safe_sec").value)

        self._failsafe_decel = float(self.get_parameter("failsafe.deceleration_mps2").value)

        # --- State ---
        self._pilot_cmd = None
        self._pilot_recv_time = None
        self._mpc_cmd = None
        self._mpc_recv_time = None
        self._scan = None
        self._speed = 0.0  # latest measured longitudinal speed [m/s]
        self._stopping = False
        self._active = self._default_controller
        self._critical_streak = 0
        self._safe_since = None
        self._last_critical_reason = ""
        self._last_steer = 0.0
        self._last_status_log_time = None

        # --- Communication Setup ---
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._sub_pilot_cmd = self.create_subscription(
            AckermannControlCommand, pilot_net_cmd_topic, self._pilot_cmd_callback, cmd_qos
        )
        self._sub_mpc_cmd = self.create_subscription(
            AckermannControlCommand, mpc_cmd_topic, self._mpc_cmd_callback, cmd_qos
        )
        if self._use_lidar_proximity:
            self._sub_scan = self.create_subscription(
                LaserScan, lidar_topic, self._scan_callback, best_effort_qos
            )
        self._sub_odom = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 1
        )
        self._sub_stop_request = self.create_subscription(
            Empty, stop_request_topic, self._stop_request_callback, 1
        )

        self._pub_cmd = self.create_publisher(
            AckermannControlCommand, output_cmd_topic, 1
        )
        self._pub_active = self.create_publisher(
            String, "/hybrid_control/active_controller", latching_qos
        )
        self._pub_is_critical = self.create_publisher(
            Bool, "/hybrid_control/is_critical", 1
        )

        self._timer = self.create_timer(1.0 / publish_rate_hz, self._tick)

        self.get_logger().info(
            f"HybridControlArbitrator is ready. default_controller={self._default_controller}, "
            f"rate={publish_rate_hz:.1f}Hz, "
            f"lidar_proximity={self._use_lidar_proximity}, "
            f"steer_disagreement={self._use_steer_disagreement}")


    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _pilot_cmd_callback(self, msg: AckermannControlCommand):
        self._pilot_cmd = msg
        self._pilot_recv_time = self.get_clock().now()

    def _mpc_cmd_callback(self, msg: AckermannControlCommand):
        self._mpc_cmd = msg
        self._mpc_recv_time = self.get_clock().now()

    def _scan_callback(self, msg: LaserScan):
        self._scan = msg

    def _odom_callback(self, msg: Odometry):
        self._speed = msg.twist.twist.linear.x

    def _stop_request_callback(self, msg: Empty):
        if not self._stopping:
            self._stopping = True
            self.get_logger().warn(
                "Stop request received -> latching STOPPING state "
                "(forwarding MPC until the vehicle stops)")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _is_fresh(self, cmd, recv_time, now, timeout_sec: float) -> bool:
        if cmd is None or recv_time is None:
            return False
        age_sec = (now - recv_time).nanoseconds / 1e9
        return age_sec < timeout_sec

    def _min_lidar_range(self):
        """Returns the minimum valid LiDAR range [m], or None if no valid return."""
        scan = self._scan
        if scan is None or len(scan.ranges) == 0:
            return None
        min_range = None
        for r in scan.ranges:
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            if min_range is None or r < min_range:
                min_range = r
        return min_range

    def _critical_reasons(self, pilot_fresh: bool, mpc_fresh: bool):
        """Returns a list of human-readable reasons why the situation is critical."""
        reasons = []

        if self._use_lidar_proximity:
            min_range = self._min_lidar_range()
            if min_range is not None:
                threshold = self._lidar_min_range_m + max(self._speed, 0.0) * self._speed_margin_per_mps
                if min_range < threshold:
                    reasons.append(
                        f"lidar_proximity(min={min_range:.2f}m < threshold={threshold:.2f}m)")

        if self._use_steer_disagreement and pilot_fresh and mpc_fresh:

            d_steer = abs(
                self._mpc_cmd.lateral.steering_tire_angle
                - self._pilot_cmd.lateral.steering_tire_angle
            )
            if d_steer > self._steer_disagreement_rad:
                reason = f"steer_disagreement(|d|={d_steer:.3f}rad > {self._steer_disagreement_rad:.3f}rad)"
                self.get_logger().info(reason)
                reasons.append(
                    reason)

        return reasons

    def _restamped(self, cmd: AckermannControlCommand) -> AckermannControlCommand:
        """Returns the given command with refreshed stamps so it never looks stale."""
        now = self.get_clock().now().to_msg()
        cmd.stamp = now
        cmd.lateral.stamp = now
        cmd.longitudinal.stamp = now
        return cmd

    def _failsafe_cmd(self) -> AckermannControlCommand:
        """Brake command used when BOTH controllers are stale."""
        cmd = AckermannControlCommand()
        now = self.get_clock().now().to_msg()
        cmd.stamp = now
        cmd.lateral.stamp = now
        cmd.lateral.steering_tire_angle = self._last_steer
        cmd.lateral.steering_tire_rotation_rate = 2.0
        cmd.longitudinal.stamp = now
        cmd.longitudinal.speed = 0.0
        cmd.longitudinal.acceleration = self._failsafe_decel if self._speed > 0.1 else 0.0
        return cmd



    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        pilot_fresh = self._is_fresh(self._pilot_cmd, self._pilot_recv_time, now, self._pilot_timeout)
        mpc_fresh = self._is_fresh(self._mpc_cmd, self._mpc_recv_time, now, self._mpc_timeout)

        # --- Criticality monitoring (skipped while stopping) ---
        reasons = []
        if not self._stopping:
            reasons = self._critical_reasons(pilot_fresh, mpc_fresh)

        # --- Hysteresis bookkeeping ---
        if reasons:
            self._critical_streak += 1
            self._safe_since = None
            self._last_critical_reason = "; ".join(reasons)
        else:
            self._critical_streak = 0
            if self._safe_since is None:
                self._safe_since = now

        # --- FSM transitions ---
        if self._stopping:
            self._active = MPC
        elif self._active == self._default_controller:
            if self._critical_streak >= self._engage_critical_cycles:
                other = MPC if self._default_controller == PILOT_NET else PILOT_NET
                other_fresh = mpc_fresh if other == MPC else pilot_fresh
                if other_fresh:
                    self._active = other
                    self.get_logger().warn(
                        f"CRITICAL -> switching {self._default_controller} -> {other}. "
                        f"Reason: {self._last_critical_reason}")
                else:
                    self.get_logger().warn(
                        "CRITICAL detected but fallback controller is stale; "
                        "keeping current controller", throttle_duration_sec=1.0)
        else:  # non-default controller active (takeover)
            safe_long_enough = (
                self._safe_since is not None
                and (now - self._safe_since).nanoseconds / 1e9 >= self._release_safe_sec
            )
            if safe_long_enough:
                default_fresh = pilot_fresh if self._default_controller == PILOT_NET else mpc_fresh
                if default_fresh:
                    self._active = self._default_controller
                    self.get_logger().info(
                        f"Situation safe for >= {self._release_safe_sec:.1f}s -> "
                        f"switching back to {self._default_controller}")
                else:
                    self.get_logger().warn(
                        f"Situation safe but {self._default_controller} is stale; "
                        "keeping takeover controller", throttle_duration_sec=1.0)


        # --- Command selection ---
        if self._stopping:
            if mpc_fresh:
                cmd, label = self._restamped(self._mpc_cmd), MPC
            else:
                cmd, label = self._failsafe_cmd(), FAILSAFE
        elif self._active == PILOT_NET:
            if pilot_fresh:
                cmd, label = self._restamped(self._pilot_cmd), PILOT_NET
            elif mpc_fresh:
                cmd, label = self._restamped(self._mpc_cmd), MPC + "_failsafe"
            else:
                cmd, label = self._failsafe_cmd(), FAILSAFE
        else:
            if mpc_fresh:
                cmd, label = self._restamped(self._mpc_cmd), MPC
            elif pilot_fresh:
                cmd, label = self._restamped(self._pilot_cmd), PILOT_NET + "_failsafe"
            else:
                cmd, label = self._failsafe_cmd(), FAILSAFE

        self._last_steer = cmd.lateral.steering_tire_angle

        # --- Publish ---
        self._pub_cmd.publish(cmd)

        active_msg = String()
        active_msg.data = label
        self._pub_active.publish(active_msg)

        critical_msg = Bool()
        critical_msg.data = bool(reasons) or self._stopping
        self._pub_is_critical.publish(critical_msg)

        # --- Periodic status log ---
        if self._last_status_log_time is None or \
                (now - self._last_status_log_time).nanoseconds / 1e9 >= 5.0:
            self.get_logger().info(
                f"active={label} | default={self._default_controller} | "
                f"pilot_fresh={pilot_fresh} mpc_fresh={mpc_fresh} | "
                f"critical_streak={self._critical_streak} | stopping={self._stopping}",
                throttle_duration_sec=5.0)
            self._last_status_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = HybridControlArbitrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
