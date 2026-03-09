#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, PoseStamped
import matplotlib.image as mpimg
import matplotlib.pyplot as plt


class SimulatorNode(Node):

    def __init__(self):
        super().__init__('simulator_node')

        # Subscribe to controller commands
        self.sub_twist = self.create_subscription(
            Twist,
            '/gita_1/twist_cmd',
            self.twist_cb,
            10
        )

        # subscribe to goal
        # self.sub_goal = self.create_subscription(
        #     PoseStamped,
        #     '/maze_goal',
        #     self.goal_cb,
        #     10
        # )

        # subscribe to controller target for visualization
        self.sub_target = self.create_subscription(
            PoseStamped,
            '/controller_target',
            self.goal_cb,
            10
        )

        # Publish simulated pose
        self.pub_pose = self.create_publisher(
            Pose,
            '/gita_1/robot_pose',
            10
        )



        # Robot state
        self.x = 0
        self.y = 0
        self.th = 0.0

        self.v = 0.0
        self.w = 0.0

        # Simulation params
        self.dt = 0.05
        self.robot_radius = 0.15

        # goal state
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Trail
        self.trail = []

        # Plot
        self.fig, self.ax = plt.subplots()

        self.img = mpimg.imread(
            "/home/ashjs/gita_ws/src/gita_waypoint_sim/gita_waypoint_sim/maze.jpeg"
        )

        self.img = self.img[:, ::-1, :]

        self.corners = [-0.3, 2.03, -8.0, 3.93]

        self.ax.imshow(
            self.img,
            extent=self.corners,
            origin="lower"
        )

        plt.show(block=False)

        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info("Simulator running")

    # ---------------------------------------

    def twist_cb(self, msg):

        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def goal_cb(self, msg):
        
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y


    # ---------------------------------------

    def step(self):

        # integrate kinematics
        self.x += self.v * math.cos(self.th) * self.dt
        self.y += self.v * math.sin(self.th) * self.dt
        self.th += self.w * self.dt

        # wrap heading
        while self.th > math.pi:
            self.th -= 2 * math.pi
        while self.th < -math.pi:
            self.th += 2 * math.pi

        self.trail.append((self.x, self.y))
        if len(self.trail) > 2000:
            self.trail = self.trail[-2000:]

        # publish pose
        pose = Pose()

        pose.position.x = float(self.x)
        pose.position.y = float(self.y)

        pose.orientation.z = math.sin(self.th / 2)
        pose.orientation.w = math.cos(self.th / 2)

        self.pub_pose.publish(pose)

        # -----------------------
        # Visualization
        # -----------------------

        self.ax.clear()

        self.ax.imshow(
            self.img,
            extent=self.corners,
            origin="lower"
        )

        self.ax.set_xlim(self.corners[0], self.corners[1])
        self.ax.set_ylim(self.corners[2], self.corners[3])

        self.ax.set_aspect('equal')
        self.ax.grid(False)

        self.ax.set_title(
            f"x={self.x:.2f}  y={self.y:.2f}  "
            f"θ={math.degrees(self.th):.1f}°  "
            f"v={self.v:.2f}  w={self.w:.2f}"
        )

        # draw trail
        if len(self.trail) > 2:
            xs = [p[0] for p in self.trail]
            ys = [p[1] for p in self.trail]
            self.ax.plot(xs, ys)

        # robot body
        robot = plt.Circle(
            (self.x, self.y),
            self.robot_radius,
            fill=False
        )

        self.ax.add_patch(robot)

        # heading arrow
        hx = self.x + 0.3 * math.cos(self.th)
        hy = self.y + 0.3 * math.sin(self.th)

        self.ax.arrow(
            self.x,
            self.y,
            hx - self.x,
            hy - self.y,
            head_width=0.1,
            length_includes_head=True
        )

        # goal arrow


        # velocity arrow
        vx = self.v * math.cos(self.th)
        vy = self.v * math.sin(self.th)

        self.ax.arrow(
            self.x,
            self.y,
            vx,
            vy,
            head_width=0.05,
            color="red",
            alpha=0.7
        )

        # goal
        self.ax.scatter(
            self.goal_x,
            self.goal_y,
            marker="x",
            color="green",
            s=100,
            label="goal"
        )

        plt.pause(0.001)


# ---------------------------------------

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