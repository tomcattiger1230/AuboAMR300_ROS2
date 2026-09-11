#!/usr/bin/env python3
"""Expand the finger-gripper Xacro and generate its Gazebo SDF."""
import argparse
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf_directory", type=Path)
    args = parser.parse_args()
    root = args.urdf_directory.resolve()
    stem = "composite_robot_finger_mono"
    urdf = subprocess.check_output(
        ["xacro", str(root / f"{stem}.urdf.xacro")], text=True
    )
    target = root / f"{stem}.urdf"
    target.write_text(urdf, encoding="utf-8")
    sdf = subprocess.check_output(["gz", "sdf", "-p", str(target)], text=True)
    share = get_package_share_directory("seer_description")
    target.write_text(
        urdf.replace(share, "$(find seer_description)"), encoding="utf-8"
    )
    (root / f"{stem}.sdf").write_text(
        sdf.replace(share, "$(find seer_description)"), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
