#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose2D, Point

import matplotlib.pyplot as plt


class SimulatorNode(Node):
    """
    Differential-drive kinematic simulator.
    Subscribes:
      - /robot_twist (Twist)
      - /target_waypoint (Point) for visualization (optional)
    Publishes:
      - /robot_pose (Pose2D)
    Visualizes:
      - robot as circle + heading arrow + target waypoint + trail
    """
    def __init__(self):
        super().__init__('simulator_node')

        self.sub_twist = self.create_subscription(Twist, 'robot_twist', self.twist_cb, 10)
        self.sub_target = self.create_subscription(Point, 'target_waypoint', self.target_cb, 10)

        self.pub_pose = self.create_publisher(Pose2D, 'robot_pose', 10)

        # State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.v = 0.0
        self.w = 0.0

        self.target = None  # (x,y)
        self.trail = []

        # Sim params
        self.dt = 0.05  # 20 Hz
        self.robot_radius = 0.25

        # Plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Gita Python Simulator")
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.grid(True)
        plt.show(block=False)

        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info("Simulator running. Publishing /robot_pose at 20 Hz.")

    def twist_cb(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def target_cb(self, msg: Point):
        self.target = (float(msg.x), float(msg.y))

    def step(self):
        # Integrate kinematics
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        self.th += self.w * self.dt
        # wrap theta
        while self.th > math.pi:
            self.th -= 2.0 * math.pi
        while self.th < -math.pi:
            self.th += 2.0 * math.pi

        self.trail.append((self.x, self.y))
        if len(self.trail) > 2000:
            self.trail = self.trail[-2000:]

        # Publish pose
        p = Pose2D()
        p.x = float(self.x)
        p.y = float(self.y)
        p.theta = float(self.th)
        self.pub_pose.publish(p)

        # Draw
        self.ax.clear()
        self.ax.set_title("Gita Python Simulator")
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.grid(True)

        # trail
        if len(self.trail) > 2:
            xs = [t[0] for t in self.trail]
            ys = [t[1] for t in self.trail]
            self.ax.plot(xs, ys)

        # robot circle
        circ = plt.Circle((self.x, self.y), self.robot_radius, fill=False)
        self.ax.add_patch(circ)

        # heading arrow
        hx = self.x + 0.5 * math.cos(self.th)
        hy = self.y + 0.5 * math.sin(self.th)
        self.ax.arrow(self.x, self.y, hx - self.x, hy - self.y, head_width=0.12, length_includes_head=True)

        # target
        if self.target is not None:
            self.ax.scatter([self.target[0]], [self.target[1]], marker='x')
            self.ax.text(self.target[0], self.target[1], "  goal", fontsize=10)

        plt.pause(0.001)


def main():
    rclpy.init()
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
