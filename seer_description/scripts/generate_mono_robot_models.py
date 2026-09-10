#!/usr/bin/env python3
"""Regenerate portable URDF/SDF after installing the new camera xacro."""
import argparse
from pathlib import Path
import subprocess
from ament_index_python.packages import get_package_share_directory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf_directory',type=Path)
    args=parser.parse_args()
    root=args.urdf_directory
    urdf=subprocess.check_output(['xacro',str(root/'composite_robot_stick_mono.urdf.xacro')],text=True)
    target=root/'composite_robot_stick_mono.urdf'
    # gz needs the expanded URDF. Portable ROS tokens are restored afterwards.
    target.write_text(urdf)
    sdf=subprocess.check_output(['gz','sdf','-p',str(target)],text=True)
    share=get_package_share_directory('seer_description')
    target.write_text(urdf.replace(share,'$(find seer_description)'))
    (root/'composite_robot_stick_mono.sdf').write_text(sdf.replace(share,'$(find seer_description)'))


if __name__=='__main__':main()
