"""An actual recorded image can initialize a search, never replace fresh evidence."""
import json
from pathlib import Path
import numpy as np
from PIL import Image
from .cylinder import resting_pose


def recorded_support_pose(path,detector,plane_height):
    path=Path(path);packet=json.loads(path.read_text())
    image=np.asarray(Image.open(path.with_suffix('.png')).convert('L'))
    return resting_pose(image,detector.probabilities(image),packet['K'],packet['T_task_optical'],
                        plane_height=plane_height)
