#!/usr/bin/env python3
"""Render a local candidate as a skeleton animation, without ROS/Isaac."""
import argparse
from itertools import product, combinations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.animation import FFMpegWriter
import numpy as np
from scipy.spatial.transform import Rotation

from offline_rebar_model import RobotModel


def wire_box(ax, shape, color, alpha=.3):
    _, centre, half, rotation = shape
    corners = np.array(list(product((-1, 1), repeat=3)))
    points = (corners * half) @ rotation.T + centre
    for i, j in combinations(range(8), 2):
        if np.count_nonzero(corners[i] != corners[j]) == 1:
            ax.plot(*points[[i, j]].T, color=color, alpha=alpha, lw=.8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    data = np.load(args.trajectory)
    model = RobotModel()
    poses = data["tcp_xyz"]
    rotations = Rotation.from_quat(data["tcp_quat_xyzw"]).as_matrix()
    fig = plt.figure(figsize=(10, 7.5))
    ax = fig.add_subplot(projection="3d")
    for name, shape in model.environment:
        if name.startswith("tester/") or "Support" in name or "MountingBeam" in name:
            wire_box(ax, shape, "#36758c" if name.startswith("tester/") else "#df9d31")
    for link, shape in model.collision_geometry(data["joints_rad"][0]):
        if link == "base_link" and shape[0] == "box":
            wire_box(ax, shape, "#666666", .7)
    ax.plot(*poses.T, color="#008fa8", alpha=.45, lw=1.5, label="Candidate TCP path")
    arm_line, = ax.plot([], [], [], color="#798694", marker="o", markersize=4, lw=5, label="Robot joint centres")
    bar_line, = ax.plot([], [], [], color="#252525", lw=4, label="0.60 m rebar")
    tcp_dot, = ax.plot([], [], [], color="#008fa8", marker="o", markersize=6)
    ax.set(xlim=(-1.9, .7), ylim=(-.7, .85), zlim=(0, 2.5),
           xlabel="base X (m)", ylabel="base Y (m)", zlabel="Z (m)")
    ax.set_box_aspect((2.6, 1.55, 2.5))
    ax.view_init(elev=20, azim=-45)
    ax.legend(loc="upper left", fontsize=8)
    fig.suptitle("LOCAL KINEMATIC CANDIDATE — NOT ISAAC SIMULATION", fontsize=13)
    progress_text = fig.text(.5, .035, "", ha="center", fontsize=11)
    names = ("aubo_base_link", "shoulder_Link", "upperArm_Link", "foreArm_Link",
             "wrist1_Link", "wrist2_Link", "wrist3_Link")
    writer = FFMpegWriter(fps=24, codec="libx264", extra_args=["-pix_fmt", "yuv420p", "-crf", "20"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with writer.saving(fig, str(args.output), dpi=120):
        for i in np.r_[np.linspace(0, len(poses) - 1, 192).astype(int), np.repeat(len(poses) - 1, 24)]:
            frames = model.frames(data["joints_rad"][i])
            points = np.array([frames[name][:3, 3] for name in names])
            bar = poses[i] + np.outer([-0.3, 0.3], rotations[i, :, 1])
            arm_line.set_data_3d(*points.T)
            bar_line.set_data_3d(*bar.T)
            tcp_dot.set_data_3d(*poses[i:i+1].T)
            progress_text.set_text(f"Progress {100*i/(len(poses)-1):.0f}% | sampled clearance {1000*data['clearance_m'][i]:.1f} mm")
            writer.grab_frame()
    plt.close(fig)
    print(args.output)


if __name__ == "__main__":
    main()
