# test_sender.py
import socket
import json
import time
import math
import pandas as pd

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

t = 0
df = pd.read_csv("/home/ashjs/gita_ws/src/gita_waypoint_sim/gita_waypoint_sim/relay_log_20260308_205751.csv")
# process df["x"] to take the string piece after ":" and convert it to a float
df["x"] = df["x"].apply(lambda x: float(x.split(":")[1]))
df["z"] = df["z"].apply(lambda z: float(z.split(":")[1]))
df["yaw"] = df["yaw"].apply(lambda y: float(y.split(":")[1]))

while t < len(df):
    x = df["x"].iloc[t]
    z = df["z"].iloc[t]
    yaw = df["yaw"].iloc[t]

    msg = {
        "timestamp": time.time(),
        "x": x,
        "y": 0,
        "z": z,
        "yaw": yaw,
        "unity_x": x,
        "unity_z": z,
        "scale_factor": 1.0
    }

    sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", 8888))
    t += 1
    time.sleep(0.05)
