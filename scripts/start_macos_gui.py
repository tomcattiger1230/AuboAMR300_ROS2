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
parser.add_argument('--check-gui', action='store_true', help='Open GUI and verify displayed feedback for 30 seconds without executing')
parser.add_argument('--check', action='store_true', help='Check feedback, FK and planning without opening a window or executing')
parser.add_argument('--plan-only', action='store_true', help='Disable the explicit Execute button')
args = parser.parse_args()
if (args.check or args.check_gui) and args.execute:
    parser.error('--check/--check-gui cannot be combined with --execute')
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
            '-p', 'enable_motion:='+str(not args.plan_only and not args.check and not args.check_gui).lower()]
print(f"[AUBO GUI] pid={os.getpid()} python={sys.executable} peer={args.peer} domain={args.domain_id} rmw=rmw_fastrtps_cpp", flush=True)
if args.check_gui:
    import runpy
    runpy.run_path(str(Path(__file__).resolve().parents[1] / 'aubo_control_gui/test/check_gui_feedback.py'), run_name='__main__')
    sys.exit(0)
if args.check:
    import socket
    try:
        with socket.socket(socket.AF_INET if args.peer.version == 4 else socket.AF_INET6, socket.SOCK_DGRAM) as probe:
            probe.connect((str(args.peer), 9))
            probe.send(b'AUBO GUI network diagnostic')
            print(f"[AUBO GUI] UDP send accepted, local={probe.getsockname()[0]} (does not verify reception)", flush=True)
    except OSError as exc:
        print(f"[AUBO GUI] UDP send failed: {exc}. Check the launching terminal's local-network permission and network route.", flush=True)
    import runpy
    runpy.run_path(str(Path(__file__).resolve().parents[1] / 'aubo_control_gui/test/check_fastdds_client.py'), run_name='__main__')
    sys.exit(0)
from aubo_control_gui.app import main
sys.exit(main())
