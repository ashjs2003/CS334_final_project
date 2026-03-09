#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, PoseStamped


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
        self.create_subscription(Pose, '/gita_1/robot_pose', self.pose_cb, 10)
        self.create_subscription(PoseStamped, '/maze_goal', self.goal_cb, 10)

        # Publisher
        self.twist_pub = self.create_publisher(Twist, '/gita_1/twist_cmd', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/controller_target', 10)

        # Control loop
        self.timer = self.create_timer(0.05, self.control_loop)

        # State
        self.pose = None

        self.goal_queue = deque()
        self.current_goal = None

        self.state = "IDLE"
        self.stop_counter = 0

        self.goal_yaw = None

        # ===== PARAMETERS =====
        self.v_max = 1.0
        self.w_max = 1.0
        self.w_min = 0.05

        self.k_turn = 0.2
        self.k_drive = 1.0

        self.theta_enter = math.radians(25)
        self.theta_exit = math.radians(15)
        self.theta_small = math.radians(2)

        self.d_stop = 0.20
        self.slow_radius = 0.9

        self.stop_cycles_required = 5

        self.get_logger().info("FSM Controller ready")

    # --------------------------------------------------

    def pose_cb(self, msg):

        x = msg.position.x
        y = msg.position.y

        yaw = msg.orientation.z

        self.pose = (x, y, yaw)

    # --------------------------------------------------

    def goal_cb(self, msg):

        gx = msg.pose.position.x
        gy = msg.pose.position.y

        self.goal_queue.append((gx, gy))

        self.get_logger().info(
            f"New goal received: ({gx:.2f},{gy:.2f}) | queue={len(self.goal_queue)}"
        )

        if self.state == "IDLE":
            self.state = "TURN_TO_FACE"

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

        # assign next goal if needed
        if self.current_goal is None and len(self.goal_queue) > 0:
            self.current_goal = self.goal_queue.popleft()
            self.goal_yaw = None
            self.state = "TURN_TO_FACE"

            self.get_logger().info(
                f"Starting goal {self.current_goal} | remaining queue={len(self.goal_queue)}"
            )

        if self.current_goal is None:
            self.state = "IDLE"
            self.stop_robot()
            return

        # publish target for visualization
        target_msg = PoseStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.pose.position.x = self.current_goal[0]
        target_msg.pose.position.y = self.current_goal[1]
        self.target_pub.publish(target_msg)
        

        x, y, th = self.pose
        gx, gy = self.current_goal

        dx = gx - x
        dy = gy - y

        dist = math.hypot(dx, dy)

        self.goal_yaw = math.atan2(dy, dx)

        wrapped_th = math.atan2(math.sin(th), math.cos(th))

        heading_err = self.goal_yaw - wrapped_th
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        # =========================
        # TURN TO FACE
        # =========================
        if self.state == "TURN_TO_FACE":

            if abs(heading_err) < self.theta_exit:
                self.state = "DRIVE_STRAIGHT"
                return

            w = self.k_turn * heading_err
            w = max(-self.w_max, min(self.w_max, w))

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

                self.get_logger().info(
                    f"Reached goal ({gx:.2f},{gy:.2f})"
                )

                return

            if abs(heading_err) > self.theta_enter:
                self.state = "TURN_TO_FACE"
                return

            w = self.k_drive * heading_err
            w = max(-0.8, min(0.8, w))

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

                self.current_goal = None

                if len(self.goal_queue) == 0:
                    self.state = "IDLE"
                    self.get_logger().info("Queue empty")
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