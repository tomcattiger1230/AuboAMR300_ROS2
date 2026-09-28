"""Read a Nav2 trinary YAML/image map without initializing ROS."""
import math
from pathlib import Path
from types import SimpleNamespace as Obj

import yaml
from PySide6.QtGui import QImage


def load_yaml_map(path):
    path = Path(path).resolve()
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict): raise ValueError('地图 YAML 必须是对象')
    if config.get('mode', 'trinary') != 'trinary':
        raise ValueError('当前预览支持 trinary 地图，请使用标准 Nav2 三值地图')
    resolution = float(config['resolution'])
    origin = [float(v) for v in config['origin']]
    occupied = float(config['occupied_thresh']); free = float(config['free_thresh'])
    negate = int(config.get('negate', 0))
    if (not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3
            or not all(math.isfinite(v) for v in origin) or not 0 <= free < occupied <= 1 or negate not in (0, 1)):
        raise ValueError('地图分辨率、原点或占据阈值无效')
    image_path = path.parent / str(config['image'])
    image = QImage(str(image_path))
    if image.isNull(): raise ValueError(f'无法读取地图图片：{image_path}')
    data = []
    # Image top-left becomes the last occupancy row in ROS.
    for y in range(image.height()-1, -1, -1):
        for x in range(image.width()):
            color = image.pixelColor(x, y)
            shade = (color.red()+color.green()+color.blue()) / (3*255)
            probability = shade if negate else 1-shade
            data.append(-1 if color.alpha() < 255 else 100 if probability > occupied else 0 if probability < free else -1)
    pose = Obj(position=Obj(x=origin[0], y=origin[1], z=0.0),
               orientation=Obj(x=0.0, y=0.0, z=math.sin(origin[2]/2), w=math.cos(origin[2]/2)))
    return Obj(header=Obj(frame_id='map'), info=Obj(width=image.width(), height=image.height(),
               resolution=resolution, origin=pose), data=data)
