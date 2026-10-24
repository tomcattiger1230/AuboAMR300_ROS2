"""Offline USD/task checks, with an optional GPU runtime/version check."""

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys

from .assets import audit_assets, build_assets
from .spec import ISAAC_LAB_TAG, ISAAC_SIM_VERSION, asset_directory


def preflight_subprocess(directory, runtime=False):
    """Audit USD outside the process that will start SimulationApp.

    Opening USD can load standalone resolver/native libraries. Kit must load its
    own versions first, so retaining the audit's pxr modules in a training process
    can cause native-library conflicts rather than a normal Python exception.
    """
    command = [sys.executable, "-m", "rebar_training.preflight", "--assets", str(directory)]
    if runtime:
        command.append("--runtime")
    result = subprocess.run(command, text=True, capture_output=True)
    if result.returncode:
        raise RuntimeError("Training preflight failed:\n" + result.stdout + result.stderr)
    return json.loads(result.stdout)


def runtime_check():
    import torch

    if sys.version_info[:2] != (3, 12):
        raise RuntimeError("use a separate Python 3.12 Isaac Lab environment")
    try:
        installed = importlib.metadata.version("isaacsim")
    except importlib.metadata.PackageNotFoundError:
        # Binary/source installations may not provide wheel metadata. Read their
        # VERSION file without importing Kit-dependent isaacsim.core pre-launch.
        sim_root = os.environ.get("ISAAC_SIM_ROOT")
        if sim_root is None and os.environ.get("ISAACLAB_ROOT"):
            sim_root = str(Path(os.environ["ISAACLAB_ROOT"]) / "_isaac_sim")
        version_file = Path(sim_root or "/nonexistent_isaacsim") / "VERSION"
        if not version_file.is_file():
            raise RuntimeError("Isaac Sim metadata missing; set ISAAC_SIM_ROOT to the binary directory containing VERSION") from None
        installed = version_file.read_text().strip().split()[0].split("-")[0].split("+")[0]
    if installed != ISAAC_SIM_VERSION and not installed.startswith(ISAAC_SIM_VERSION + "."):
        raise RuntimeError(f"expected Isaac Sim {ISAAC_SIM_VERSION}, found {installed}")
    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch cannot access a CUDA GPU")
    return {"isaac_sim": installed, "torch": torch.__version__, "gpu": torch.cuda.get_device_name(0)}


def preflight(directory, build=False, runtime=False):
    import hashlib

    directory = Path(directory)
    audit = build_assets(directory) if build else audit_assets(directory)
    manifest = json.loads((directory / "manifest.json").read_text())
    from .spec import repository_root
    for relative, expected in manifest["source_sha256"].items():
        source = repository_root() / relative
        if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"source changed; rebuild training assets: {relative}")
    if manifest["isaac_lab"] != ISAAC_LAB_TAG or manifest["isaac_sim"] != ISAAC_SIM_VERSION:
        raise RuntimeError("asset manifest version differs from the training package; rebuild assets")
    return {
        "offline_passed": True, "assets": str(directory.resolve()), "usd": audit,
        "isaac_lab_target": ISAAC_LAB_TAG, "isaac_sim_target": ISAAC_SIM_VERSION,
        "runtime": runtime_check() if runtime else "not_checked",
        "gpu_validated": False,
        "pending_on_4090": ["PhysX articulation import and contact-filter shapes", "reset stability and initial grasp",
                            "physical fixture handoff", "PPO smoke run", "held-out success-rate evaluation"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, default=asset_directory())
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--runtime", action="store_true")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    result = preflight(args.assets, args.build, args.runtime)
    text = json.dumps(result, indent=2) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text)
    print(text, end="")


if __name__ == "__main__":
    main()
