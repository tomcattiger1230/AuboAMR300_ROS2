"""Exercise offline help and version guards without importing the simulator."""

import os
from pathlib import Path
import subprocess
import sys
from types import ModuleType

import pytest
torch = pytest.importorskip("torch")

from rebar_training.launcher import verify_checkout
from rebar_training.preflight import runtime_check
from rebar_training.spec import ISAAC_LAB_TAG


@pytest.mark.parametrize("module", ["launcher", "gpu_check", "preflight", "assets", "vision", "remote_debug",
                                   "visual_check", "mobile_assets", "mobile_check", "mobile_mission_check",
                                   "mobile_jobs", "mobile_acceptance", "mobile_curriculum"])
def test_command_help_requires_no_gpu_or_isaac(module):
    environment = os.environ.copy()
    environment.pop("ISAACLAB_ROOT", None)
    environment["PYTHONPATH"] = str(Path(__file__).parents[1] / "src")
    result = subprocess.run([sys.executable, "-m", "rebar_training." + module, "--help"],
                            env=environment, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout


def test_runtime_guard_rejects_wrong_sim_before_launch(monkeypatch):
    monkeypatch.setattr("rebar_training.preflight.importlib.metadata.version", lambda _: "5.1.0")
    with pytest.raises(RuntimeError, match="expected Isaac Sim"):
        runtime_check()


def test_runtime_guard_requires_cuda(monkeypatch):
    monkeypatch.setattr("rebar_training.preflight.importlib.metadata.version", lambda _: "6.0.1")
    monkeypatch.setattr(torch.cuda, "is_available", lambda: False)
    with pytest.raises(RuntimeError, match="CUDA"):
        runtime_check()


def test_checkout_guard_rejects_changed_runtime_source(tmp_path):
    def git(*args):
        return subprocess.run(["git", "-C", str(tmp_path), *args], check=True, capture_output=True)
    git("init", "-q")
    source = tmp_path / "source/example.py"
    source.parent.mkdir()
    source.write_text("value = 1\n")
    git("add", ".")
    git("-c", "user.name=Offline Test", "-c", "user.email=offline@example.invalid", "commit", "-qm", "fixture")
    git("tag", ISAAC_LAB_TAG)
    verify_checkout(tmp_path)
    source.write_text("value = 2\n")
    with pytest.raises(RuntimeError, match="modifications"):
        verify_checkout(tmp_path)


@pytest.mark.parametrize("task_args,expected", [(["--task", "grasp"], "Aubo-Rebar-Grasp-Direct-v0"),
                                              (["--task=insert"], "Aubo-Rebar-Insert-Direct-v0")])
def test_short_task_is_rewritten_before_launcher_inspects_arguments(monkeypatch, tmp_path, task_args, expected):
    from rebar_training import gpu_check

    monkeypatch.setenv("ISAACLAB_ROOT", str(tmp_path))
    monkeypatch.setattr(gpu_check, "verify_checkout", lambda *_: None)
    monkeypatch.setattr(gpu_check, "preflight", lambda *_, **__: None)
    monkeypatch.setattr(sys, "argv", ["gpu_check", *task_args, "--report", str(tmp_path / "report.json"), "--headless"])
    gym = ModuleType("gymnasium")
    gym.registry = {}
    gym.register = lambda **kwargs: gym.registry.update({kwargs["id"]: kwargs})
    monkeypatch.setitem(sys.modules, "gymnasium", gym)
    tasks = ModuleType("isaaclab_tasks")
    utils = ModuleType("isaaclab_tasks.utils")
    hydra = ModuleType("isaaclab_tasks.utils.hydra")
    inspected = []

    def add_launcher_args(parser):
        # The real Lab helper inspects argv while adding its own flags.
        inspected.append(parser.parse_known_args()[0].task)
        parser.add_argument("--headless", action="store_true")
        parser.add_argument("--device", default=None)

    utils.add_launcher_args = add_launcher_args
    utils.setup_preset_cli = lambda parser: parser.parse_known_args()
    utils.launch_simulation = lambda *_: None
    hydra.hydra_task_config = lambda *args: lambda function: lambda: None
    for name, module in (("isaaclab_tasks", tasks), ("isaaclab_tasks.utils", utils),
                         ("isaaclab_tasks.utils.hydra", hydra)):
        monkeypatch.setitem(sys.modules, name, module)
    with pytest.raises(RuntimeError, match="GPU check did not complete"):
        gpu_check.main()
    assert inspected == [expected]


@pytest.mark.parametrize("receipt", ["missing", "stale", "failed", "complete"])
def test_supervisor_checks_fresh_completion_despite_zero_kit_exit(monkeypatch, tmp_path, receipt):
    import json
    from rebar_training import gpu_check
    report = tmp_path / "report.json"
    monkeypatch.setattr(sys, "argv", ["gpu_check", "--task", "grasp", "--report", str(report)])

    def worker(command, env):
        if receipt != "missing":
            report.write_text(json.dumps({"run_id": "old" if receipt == "stale" else env["REBAR_GPU_CHECK_WORKER"],
                                          "completed": receipt != "failed"}))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(gpu_check.subprocess, "run", worker)
    if receipt == "complete":
        gpu_check.supervised_main()
    else:
        with pytest.raises(RuntimeError, match="GPU check failed or did not complete"):
            gpu_check.supervised_main()
