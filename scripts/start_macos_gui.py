#!/usr/bin/env python3
"""Native GUI with ROS 2 Fast DDS; run with the RoboStack environment Python."""
import argparse
import ipaddress
import os
from pathlib import Path
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--peer', default='192.168.3.133', type=ipaddress.ip_address)
parser.add_argument('--domain-id', default=133, type=int)
parser.add_argument('--execute', action='store_true', help='Allow explicit execution (the default); planning never executes')
parser.add_argument('--check', action='store_true', help='Check feedback, FK and planning without opening a window or executing')
parser.add_argument('--plan-only', action='store_true', help='Disable the explicit Execute button')
args = parser.parse_args()
if args.check and args.execute:
    parser.error('--check cannot be combined with --execute')
if not 0 <= args.domain_id <= 232:
    parser.error('domain-id must be in 0..232')
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_STATIC_PEERS'] = str(args.peer)
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
os.environ.pop('ROS_DISCOVERY_SERVER', None)
os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = str(Path(__file__).resolve().parents[1] / 'aubo_control_gui/config/fastdds_macos.xml')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'aubo_control_gui'))
# ROS receives only ROS arguments, not this launcher's arguments.
sys.argv = [sys.argv[0], '--ros-args', '-p', 'use_sim_time:=true',
            '-p', 'enable_motion:='+str(not args.plan_only and not args.check).lower()]
if args.check:
    import runpy
    runpy.run_path(str(Path(__file__).resolve().parents[1] / 'aubo_control_gui/test/check_fastdds_client.py'), run_name='__main__')
    sys.exit(0)
from aubo_control_gui.app import main
sys.exit(main())
