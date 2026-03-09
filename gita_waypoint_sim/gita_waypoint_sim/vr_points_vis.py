import socket
import json
import threading
from collections import deque
import math

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# -----------------------
# UDP SETUP
# -----------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 8888))

print("Listening for VR data... visualizing live")

# -----------------------
# STATE
# -----------------------
points = deque(maxlen=5000)   # trail of received coords
robot_state = {"x": 0, "z": 0, "yaw": 0}
lock = threading.Lock()

# -----------------------
# MAZE IMAGE
# -----------------------
# -----------------------
# MAZE IMAGE (REAL WORLD COORDS)
# -----------------------
maze_img = mpimg.imread("maze.jpeg")

plt.ion()
fig, ax = plt.subplots()
ax.set_title("Live Gita Overlay")

# REAL maze bounds from Unity world
xmin = -7.59
xmax = 0.03
ymin = -33.42
ymax = 0.65

extent = (xmin, xmax, ymin, ymax)

# IMPORTANT:
# origin="lower" so coords match Unity world
ax.imshow(maze_img, extent=extent, origin="lower")

# artists
scatter = ax.scatter([], [], s=10)
robot_dot, = ax.plot([], [], marker="o", markersize=8)
heading_line, = ax.plot([], [], linewidth=2)

ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
ax.set_aspect("equal")


# -----------------------
# NETWORK THREAD
# -----------------------
def listen():
    while True:
        data, addr = sock.recvfrom(1024)
        cmd = json.loads(data.decode("utf-8"))

        with lock:
            x = cmd["x"]
            z = cmd["z"]
            yaw = math.radians(cmd["yaw"])

            robot_state["x"] = x
            robot_state["z"] = z
            robot_state["yaw"] = yaw

            points.append((x, z))

        print(f"Gita: ({x:.2f}, {z:.2f}) yaw {cmd['yaw']:.1f}")

threading.Thread(target=listen, daemon=True).start()

# -----------------------
# LIVE DRAW LOOP
# -----------------------
while True:
    with lock:
        pts = list(points)
        x = robot_state["x"]
        z = robot_state["z"]
        yaw = robot_state["yaw"]

    # update trail
    if pts:
        xs, zs = zip(*pts)
        scatter.set_offsets(list(zip(xs, zs)))

    # robot dot
    robot_dot.set_data([x], [z])

    # heading arrow
    L = 0.4
    hx = x + L * math.cos(yaw)
    hz = z + L * math.sin(yaw)
    heading_line.set_data([x, hx], [z, hz])

    fig.canvas.draw_idle()
    fig.canvas.flush_events()
