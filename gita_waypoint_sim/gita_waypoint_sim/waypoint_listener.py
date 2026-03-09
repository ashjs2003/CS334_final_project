import json
import math
import socket
import threading
from collections import deque
from collections import defaultdict

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


# ----------------------------
# Fixed maze waypoints
# ----------------------------
WAYPOINTS = np.array(
[
 [ 2.21050714, -5.72352208],
 [ 1.31919416, -5.70891039],
 [ 0.41326948, -5.72352208],
 [ 0.45710455, -4.86143247],
 [ 1.33380584, -4.86143247],
 [ 2.18128377, -4.89065584],
 [ 2.21050714, -4.10162468],
 [ 1.30458247, -4.05778961],
 [ 0.41326948, -4.08701299],
 [ 0.41326948, -3.1957],
 [ 1.31919416, -3.21031169],
 [ 2.18128377, -3.21031169],
 [ 2.18128377, -2.34822208],
 [ 1.28997078, -2.40666883],
 [ 0.39865779, -2.36283377],
 [ 0.41326948, -1.55919091],
 [ 1.31919416, -1.52996753],
 [ 2.19589545, -1.54457922],
 [ 2.21050714, -0.72632468],
 [ 1.34841753, -0.71171299],
 [ 0.3840461 , -0.6971013],
 [ 0.41326948,  0.10654156],
 [ 1.2607474 ,  0.13576494],
 [ 2.21050714,  0.09192987],
 [ 2.22511883,  0.99785455],
 [ 1.36302922,  1.01246623],
 [ 0.45710455,  0.99785455],
 [ 0.42788117,  1.78688571],
 [ 1.31919416,  1.77227403],
 [ 2.25434221,  1.74305065],
 [ 2.22511883,  2.59052857],
 [ 1.33380584,  2.64897532],
 [ 0.35482273,  2.6781987],
 [ 2.21050714,  3.46722987],
 [ 1.33380584,  3.49645325],
 [ 0.41326948,  3.46722987],
 [ 0.41326948,  4.25626104],
 [ 1.33380584,  4.31470779],
 [ 2.23973052,  4.3000961]
],
dtype=float,
)

corners = [0.0, 2.653, -6.571, 4.68]
gita_corners = [-0.3, 2.03, -8.0, 3.93]


# ----------------------------
# Load graph edges
# ----------------------------
edges = set()

with open("/home/ashjs/gita_ws/src/gita_waypoint_sim/gita_waypoint_sim/edges.txt", "r") as f:
    for line in f:
        a, b = map(int, line.split())
        edges.add((a, b))
        edges.add((b, a))


adjacent_nodes = defaultdict(set)

for a, b in edges:
    adjacent_nodes[a].add(b)


# ----------------------------
# Shared state
# ----------------------------
points = deque(maxlen=5000)

robot_state = {
    "x": 0.0,
    "z": 0.0,
    "yaw": 0.0
}

goals = []
goal_indices = []

lock = threading.Lock()


# ----------------------------
# ROS2 Goal Publisher
# ----------------------------
class GoalPublisher(Node):

    def __init__(self):
        super().__init__("maze_goal_publisher")

        self.pub = self.create_publisher(
            PoseStamped,
            "/maze_goal",
            10
        )

    def publish_goal(self, x, z, yaw):

        msg = PoseStamped()

        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(z)
        msg.pose.position.z = 0.0

        # yaw → quaternion
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pub.publish(msg)

        self.get_logger().info(
            f"Published goal: ({x:.2f}, {z:.2f})"
        )


# ----------------------------
# Utility functions
# ----------------------------
def parse_numeric(value):

    if isinstance(value, (int, float)):
        return float(value)

    if isinstance(value, str):
        text = value.strip()

        if ":" in text:
            text = text.split(":", 1)[1].strip()

        return float(text)

    raise ValueError(f"Unsupported numeric value {value}")


def nearest_waypoint(x, z, previous_idx=None):

    source = np.array([x, z])

    dists = np.linalg.norm(WAYPOINTS - source, axis=1)

    idx = int(np.argmin(dists))

    if previous_idx is not None:
        if idx not in adjacent_nodes[previous_idx]:
            return None, None

    return idx, WAYPOINTS[idx]


# ----------------------------
# UDP Listener
# ----------------------------
def listen(sock, goal_node):

    while True:

        data, _ = sock.recvfrom(4096)

        try:

            payload = json.loads(data.decode("utf-8"))

            x = parse_numeric(payload["x"])
            z = parse_numeric(payload["z"])
            yaw_deg = parse_numeric(payload["yaw"])

            yaw_rad = math.radians(yaw_deg)

            with lock:

                robot_state["x"] = x
                robot_state["z"] = z
                robot_state["yaw"] = yaw_rad

                points.append((x, z))

                prev = goal_indices[-1] if goal_indices else None

                goal_idx, goal_xy = nearest_waypoint(x, z, prev)

                

                if goal_idx is not None:

                    if not goal_indices or goal_indices[-1] != goal_idx:

                        goal_indices.append(goal_idx)

                        goals.append(tuple(goal_xy))

                        gx, gy = remap_point(
                            goal_xy[0],
                            goal_xy[1],
                            corners,
                            gita_corners
                        )

                        goal_node.publish_goal(gx, gy, yaw_rad)

                        print(
                            f"Source ({x:.2f},{z:.2f}) → waypoint {goal_idx}"
                        )

        except Exception as e:

            print("Bad packet:", e)

def remap_point(x, y, src, dst):

    sx0, sx1, sy0, sy1 = src
    dx0, dx1, dy0, dy1 = dst

    xn = (x - sx0) / (sx1 - sx0)
    yn = (y - sy0) / (sy1 - sy0)

    xt = dx0 + xn * (dx1 - dx0)
    yt = dy0 + yn * (dy1 - dy0)

    return xt, yt

# ----------------------------
# Main
# ----------------------------
def main():

    rclpy.init()

    goal_node = GoalPublisher()

    threading.Thread(
        target=rclpy.spin,
        args=(goal_node,),
        daemon=True
    ).start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.bind(("0.0.0.0", 8888))

    print("Listening on UDP 8888...")

    maze_img = mpimg.imread("/home/ashjs/gita_ws/src/gita_waypoint_sim/gita_waypoint_sim/maze.jpeg")

    maze_img = maze_img[:, ::-1, :]

    
    plt.ion()

    fig, ax = plt.subplots()

    ax.imshow(maze_img, extent=corners, origin="lower")

    #title
    ax.set_title("Fake Point Display")

    ax.scatter(
        WAYPOINTS[:, 0],
        WAYPOINTS[:, 1],
        s=15,
        alpha=0.4,
        label="waypoints"
    )

    trail = ax.scatter([], [], s=8)

    robot_dot, = ax.plot([], [], "ro")

    heading_line, = ax.plot([], [], linewidth=2)

    goal_scatter = ax.scatter([], [], marker="x")

    ax.legend()

    threading.Thread(
        target=listen,
        args=(sock, goal_node),
        daemon=True
    ).start()

    while True:

        with lock:

            pts = list(points)

            x = robot_state["x"]
            z = robot_state["z"]
            yaw = robot_state["yaw"]

            goal_list = list(goals)

        if pts:
            trail.set_offsets(np.array(pts))

        robot_dot.set_data([x], [z])

        hx = x + 0.4 * math.cos(yaw)
        hz = z + 0.4 * math.sin(yaw)

        heading_line.set_data([x, hx], [z, hz])

        if goal_list:
            goal_scatter.set_offsets(np.array(goal_list))

        fig.canvas.draw_idle()
        fig.canvas.flush_events()


if __name__ == "__main__":
    main()