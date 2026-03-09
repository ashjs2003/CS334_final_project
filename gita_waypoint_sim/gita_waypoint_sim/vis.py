import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

df = pd.read_csv("gita_coordinates_live.csv")
img = mpimg.imread("maze.jpeg")

print(np.max(df["unity_x"]), np.min(df["unity_x"]))
print(np.max(df["unity_z"]), np.min(df["unity_z"]))


h, w = img.shape[:2]


img_x = df["unity_x"] + w/2
img_y = h/2 - df["unity_z"]   # flip z (Unity → image)

# convert to numpy
img_x = img_x.to_numpy()
img_y = img_y.to_numpy()

plt.figure(figsize=(8,8))
plt.imshow(img)
plt.plot(img_x, img_y, 'r-', linewidth=2, label="Unity path")
plt.scatter(img_x[0], img_y[0], c="green", s=80, label="start")
plt.scatter(img_x[-1], img_y[-1], c="blue", s=80, label="end")

plt.legend()
plt.axis("off")
plt.title("Unity trajectory on maze")
plt.show()
