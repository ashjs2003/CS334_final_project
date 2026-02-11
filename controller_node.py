#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose2D, PoseArray


def wrap_to_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        # Subscribers
        self.create_subscription(Pose2D, 'robot_pose', self.pose_cb, 10)
        self.create_subscription(PoseArray, 'waypoints', self.waypoints_cb, 10)

        # Publisher
        self.twist_pub = self.create_publisher(Twist, 'robot_twist', 10)

        # Control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # State
        self.pose = None
        self.waypoints = []
        self.idx = 0
        self.state = "IDLE"
        self.stop_counter = 0

        # ===== PARAMETERS =====
        self.v_max = 0.6
        self.w_max = 1.2
        self.w_min = 0.18

        self.k_turn = 2.2
        self.k_drive = 1.4

        self.theta_enter = math.radians(12)
        self.theta_exit = math.radians(6)
        self.theta_small = math.radians(2)

        self.d_stop = 0.20
        self.slow_radius = 0.8

        self.stop_cycles_required = 8  # ~0.4 sec

        self.get_logger().info("FSM Controller ready")

    # --------------------------------------------------

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        self.idx = 0

        if len(self.waypoints) == 0:
            self.state = "IDLE"
            return

        self.state = "TURN_TO_FACE"
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints")

    # --------------------------------------------------

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.twist_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    # --------------------------------------------------

    def control_loop(self):

        if self.pose is None:
            return

        if self.state == "IDLE":
            self.stop_robot()
            return

        if self.idx >= len(self.waypoints):
            self.stop_robot()
            self.state = "IDLE"
            self.get_logger().info("All waypoints complete")
            return

        x = self.pose.x
        y = self.pose.y
        th = self.pose.theta

        gx, gy = self.waypoints[self.idx]

        dx = gx - x
        dy = gy - y

        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        heading_err = wrap_to_pi(desired - th)

        # =========================
        # TURN TO FACE
        # =========================
        if self.state == "TURN_TO_FACE":

            if abs(heading_err) < self.theta_exit:
                self.state = "DRIVE_STRAIGHT"
                return

            w = self.k_turn * heading_err
            w = max(-self.w_max, min(self.w_max, w))

            # minimum angular velocity
            if abs(heading_err) > self.theta_small and abs(w) < self.w_min:
                w = math.copysign(self.w_min, w)

            self.publish_cmd(0.0, w)
            return

        # =========================
        # DRIVE STRAIGHT
        # =========================
        if self.state == "DRIVE_STRAIGHT":

            if dist < self.d_stop:
                self.state = "STOP_AT_WAYPOINT"
                self.stop_counter = 0
                self.stop_robot()
                self.get_logger().info(f"Reached waypoint {self.idx+1}")
                return

            # if heading error too large, go back to turn
            if abs(heading_err) > self.theta_enter:
                self.state = "TURN_TO_FACE"
                return

            # angular correction
            w = self.k_drive * heading_err
            w = max(-0.8, min(0.8, w))

            # speed scaling
            dist_scale = min(1.0, dist / self.slow_radius)
            turn_scale = max(0.25, 1.0 - abs(heading_err))

            v = self.v_max * dist_scale * turn_scale

            self.publish_cmd(v, w)
            return

        # =========================
        # STOP AT WAYPOINT
        # =========================
        if self.state == "STOP_AT_WAYPOINT":

            self.stop_robot()
            self.stop_counter += 1

            if self.stop_counter > self.stop_cycles_required:
                self.idx += 1

                if self.idx >= len(self.waypoints):
                    self.state = "IDLE"
                    self.get_logger().info("Final waypoint reached")
                else:
                    self.state = "TURN_TO_FACE"

            return


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
