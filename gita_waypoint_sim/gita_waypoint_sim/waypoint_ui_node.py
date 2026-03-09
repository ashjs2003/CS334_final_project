#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
import matplotlib.pyplot as plt
import matplotlib.image as mpimg



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
        self.max_points = 10
        
        self.fig, self.ax = plt.subplots()
        # ---- load background image ----
        self.img = mpimg.imread("/home/ashjs/gita_ws/src/gita_waypoint_sim/gita_waypoint_sim/maze.jpeg")

        h, w = self.img.shape[0]/50, self.img.shape[1]/50

        # If you want world coords to equal image pixels:
        self.xmin, self.xmax = 0, w
        self.ymin, self.ymax = 0, h

        self.ax.imshow(
            self.img,
            extent=[self.xmin, self.xmax, self.ymin, self.ymax],
            origin="upper"   # or "lower" depending on coordinate system
        )

        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_aspect('equal')
        self.ax.set_title("Click 3 waypoints")
        self.ax.grid(False)


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

        # redraw background
        self.ax.imshow(
            self.img,
            extent=[self.xmin, self.xmax, self.ymin, self.ymax],
            origin="upper"
        )

        self.ax.set_title(f"Waypoints: {len(self.points)}/{self.max_points}")
        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_aspect('equal')
        self.ax.grid(False)

        self.ax.scatter(xs, ys, c="red", s=50)


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
