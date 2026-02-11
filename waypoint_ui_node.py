#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
import matplotlib.pyplot as plt


class WaypointUINode(Node):
    """
    Click 3 points in a matplotlib window. Publishes them as a PoseArray on /waypoints.
    - World frame coordinates.
    - Orientation ignored.
    """
    def __init__(self):
        super().__init__('waypoint_ui_node')

        self.pub_waypoints = self.create_publisher(PoseArray, 'waypoints', 10)

        self.points = []
        self.max_points = 3

        # Visualization window bounds (meters)
        self.xmin, self.xmax = -5.0, 5.0
        self.ymin, self.ymax = -5.0, 5.0

        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Click 3 waypoints (world frame)")
        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)

        self.scatter = self.ax.scatter([], [])
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        self.get_logger().info("Waypoint UI ready. Click 3 points in the plot window.")
        plt.show(block=False)

        # Keep matplotlib responsive
        self.timer = self.create_timer(0.05, self._spin_matplotlib)

    def _spin_matplotlib(self):
        plt.pause(0.001)

    def on_click(self, event):
        if event.inaxes != self.ax:
            return

        if len(self.points) >= self.max_points:
            self.get_logger().info("Already captured 3 points. Close window or restart node to redo.")
            return

        x, y = float(event.xdata), float(event.ydata)
        self.points.append((x, y))

        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        self.ax.clear()
        self.ax.set_title(f"Waypoints: {len(self.points)}/{self.max_points} (click)")
        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True)
        self.ax.scatter(xs, ys)

        # Label points 1..3
        for i, (px, py) in enumerate(self.points, start=1):
            self.ax.text(px, py, f"  {i}", fontsize=12)

        self.fig.canvas.draw_idle()

        self.get_logger().info(f"Clicked waypoint {len(self.points)}: ({x:.2f}, {y:.2f})")

        if len(self.points) == self.max_points:
            self.publish_waypoints()

    def publish_waypoints(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        for (x, y) in self.points:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0.0
            # orientation ignored
            p.orientation.w = 1.0
            msg.poses.append(p)

        self.pub_waypoints.publish(msg)
        self.get_logger().info("Published 3 waypoints on /waypoints.")


def main():
    rclpy.init()
    node = WaypointUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
