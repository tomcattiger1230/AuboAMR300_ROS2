"""Version-checked entry into upstream Isaac Lab PPO training/playback."""

import argparse
import os
from pathlib import Path
import runpy
import subprocess
import sys

from .preflight import preflight_subprocess as preflight
from .spec import ISAAC_LAB_TAG, asset_directory


def verify_checkout(root):
    root = Path(root)
    def rev(ref):
        try:
            return subprocess.check_output(["git", "-C", str(root), "rev-parse", ref], text=True,
                                           stderr=subprocess.STDOUT).strip()
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(f"Isaac Lab checkout must contain the pinned {ISAAC_LAB_TAG} tag: {root}") from exc
    if rev("HEAD") != rev(f"refs/tags/{ISAAC_LAB_TAG}^{{commit}}"):
        raise RuntimeError(f"Isaac Lab must be checked out at {ISAAC_LAB_TAG}")
    # Changes in runtime source invalidate the API contract even at the same HEAD.
    if subprocess.check_output(["git", "-C", str(root), "diff", "HEAD", "--", "source", "scripts"], text=True).strip():
        raise RuntimeError("Isaac Lab source/scripts have tracked modifications; use a clean pinned checkout")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("train", "play"))
    parser.add_argument("--task", choices=("grasp", "insert", "loading"), required=True)
    parser.add_argument("--isaaclab-root", type=Path, default=os.environ.get("ISAACLAB_ROOT"))
    args, remaining = parser.parse_known_args()
    if args.isaaclab_root is None:
        parser.error("set ISAACLAB_ROOT or --isaaclab-root")
    os.environ["ISAACLAB_ROOT"] = str(args.isaaclab_root.resolve())
    verify_checkout(args.isaaclab_root)
    preflight(asset_directory(), runtime=True)
    # Default SimulationCfg is PhysX. Other backend overrides are intentionally
    # unsupported until contact sensors and wrappers have been validated there.
    if any("physics=" in arg or arg.startswith("--external_callback") or arg.startswith("--external-callback")
           for arg in remaining):
        parser.error("backend/external_callback overrides are unsupported for these pinned PhysX tasks")
    script = args.isaaclab_root / "scripts/reinforcement_learning/rsl_rl" / (args.mode + ".py")
    sys.path.insert(0, str(script.parent))  # upstream cli_args.py
    sys.argv = [str(script), "--task", f"Aubo-Rebar-{args.task.title()}-Direct-v0",
                "--external_callback", "rebar_training.registration.register_tasks", *remaining]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
