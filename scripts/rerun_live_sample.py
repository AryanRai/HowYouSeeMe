#!/usr/bin/env python3
"""Live rerun sample - spawns viewer and streams animated 3D data."""
import time
import math
import numpy as np
import rerun as rr

rr.init("live_sample", spawn=True)

print("Rerun viewer launched. Streaming data...")

for frame in range(200):
    rr.set_time("frame", sequence=frame)
    t = frame * 0.1

    # Animated spiral point cloud
    n = 300
    angles = np.linspace(0, 4 * math.pi, n) + t
    radii = np.linspace(0.1, 2.0, n)
    points = np.column_stack([
        radii * np.cos(angles),
        radii * np.sin(angles),
        np.linspace(-1, 1, n) + 0.3 * np.sin(t),
    ]).astype(np.float32)

    # Color by height
    z_norm = ((points[:, 2] + 1.5) / 3.0 * 255).astype(np.uint8)
    colors = np.column_stack([z_norm, 100 * np.ones(n, dtype=np.uint8), (255 - z_norm)]).astype(np.uint8)

    rr.log("world/spiral", rr.Points3D(points, colors=colors, radii=0.02))

    # Orbiting robot pose
    rx, ry = 1.5 * math.cos(t), 1.5 * math.sin(t)
    rr.log("world/robot", rr.Transform3D(translation=[rx, ry, 0.0]))
    rr.log("world/robot_point", rr.Points3D([[rx, ry, 0.0]], colors=[[0, 255, 100]], radii=0.08))

    # Scalar metrics
    rr.log("metrics/sin", rr.Scalars(math.sin(t)))
    rr.log("metrics/cos", rr.Scalars(math.cos(t)))

    time.sleep(0.05)

print("Done.")
