"""Read tool visual poses relative to wrist3 from the simulation URDF (metres)."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

TOOL_LINKS = ('gripper_adapter_link', 'gripper_motor_link', 'gripper1_link', 'gripper2_link')
FINGERS = ('gripper1_joint', 'gripper2_joint')


def origin(element):
    t = np.eye(4)
    if element is None:
        return t
    r, p, y = map(float, element.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = list(map(float, element.get('xyz', '0 0 0').split()))
    return t


def tool_visuals(urdf_path):
    root = ET.parse(urdf_path).getroot()
    links = {link.get('name'): link for link in root.findall('link')}
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    transforms = {'wrist3_Link': np.eye(4)}
    def pose(link):
        if link not in transforms:
            joint = joints[link]
            transforms[link] = pose(joint.find('parent').get('link')) @ origin(joint.find('origin'))
        return transforms[link]
    result = []
    for link in TOOL_LINKS:
        joint = joints[link]
        index = FINGERS.index(joint.get('name')) if joint.get('name') in FINGERS else -1
        axis = np.zeros(3)
        if index >= 0:
            axis = pose(link)[:3, :3] @ np.array(list(map(float, joint.find('axis').get('xyz').split())))
        for visual in links[link].findall('visual'):
            mesh = visual.find('geometry/mesh')
            transform = pose(link) @ origin(visual.find('origin'))
            result.append(dict(link=link, position=transform[:3, 3].tolist(),
                rotation=transform[:3, :3].tolist(), axis=axis.tolist(), finger=index,
                scale=list(map(float, mesh.get('scale', '1 1 1').split())),
                mesh=Path(mesh.get('filename')).stem.lower()+'.glb'))
    return result
