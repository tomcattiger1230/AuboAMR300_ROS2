"""Resolve installed ROS resources or a source checkout on desktop clients."""
from pathlib import Path

def get_package_share_directory(name):
    path = Path(__file__).resolve().parents[2] / name
    if (path / "package.xml").is_file():
        return str(path)
    try:
        from ament_index_python.packages import get_package_share_directory as resolve
        return resolve(name)
    except (ImportError, LookupError):
        path = Path(__file__).resolve().parents[2] / name
        if not path.is_dir():
            raise FileNotFoundError(path)
        return str(path)
