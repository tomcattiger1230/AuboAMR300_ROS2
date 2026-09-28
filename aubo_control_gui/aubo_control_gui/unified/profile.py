"""Connection profiles; no device access while editing or previewing."""
from dataclasses import dataclass
import ipaddress
import math
import os
import sys

@dataclass(frozen=True)
class Profile:
    mode: str = 'isaac'
    domain: int = 133
    peer: str = '192.168.0.103'
    seer_ip: str = '192.168.3.250'
    real_manual: bool = False

    def validate(self):
        if self.mode not in ('isaac', 'real'):
            raise ValueError('未知机器人模式')
        if not 0 <= self.domain <= 232:
            raise ValueError('ROS Domain 必须在 0–232')
        for address in (self.peer, self.seer_ip):
            if ipaddress.ip_address(address).version != 4:
                raise ValueError('当前连接配置使用 IPv4 地址')
        return self

    def configure_ros(self):
        self.validate()
        os.environ['ROS_DOMAIN_ID'] = str(self.domain)
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'SUBNET'
        os.environ['ROS_STATIC_PEERS'] = self.peer
        os.environ.pop('ROS_LOCALHOST_ONLY', None)
        # Do not inherit discovery-server or transport overrides from another launch.
        for name in ('FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'ROS_DISCOVERY_SERVER'):
            os.environ.pop(name, None)
        if sys.platform == 'darwin':
            from ..resources import get_package_share_directory
            from pathlib import Path
            os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = str(Path(get_package_share_directory('aubo_control_gui'))/'config/fastdds_macos.xml')


def velocity(linear, angular):
    if not all(math.isfinite(v) for v in (linear, angular)):
        raise ValueError('速度必须是有限数值')
    if abs(linear) > .3 or abs(angular) > .6:
        raise ValueError('统一控制台速度上限为 0.3 m/s、0.6 rad/s')
    return float(linear), float(angular)
