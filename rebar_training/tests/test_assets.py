"""Compose actual repository assets without Isaac Sim or a GPU."""

import json
import os
from pathlib import Path
import subprocess
import sys
import pytest
pytest.importorskip("torch")
pytest.importorskip("pxr")
from pxr import Usd, UsdGeom, UsdPhysics

from rebar_training.assets import audit_assets, build_assets
from rebar_training.preflight import preflight


@pytest.fixture(scope="module")
def assets(tmp_path_factory):
    path = tmp_path_factory.mktemp("training_assets")
    build_assets(path)
    return path


def test_source_assets_compose_and_dynamic_rebar_remains_unassisted(assets):
    result = audit_assets(assets)
    assert result["rebar_dynamic"] and result["rebar_collision_enabled"]
    assert not result["gpu_validated"]
    rebar = Usd.Stage.Open(str(assets / "rebar.usda"))
    assert rebar.GetPrimAtPath("/Rebar").IsA(UsdGeom.Xform)
    assert rebar.GetPrimAtPath("/Rebar/body").IsA(UsdGeom.Cylinder)
    robot = Usd.Stage.Open(str(assets / "robot.usda"))
    assert robot.GetPrimAtPath("/Robot/base_footprint").HasAPI(UsdPhysics.RigidBodyAPI)
    assert len([p for p in robot.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]) == 1
    sensor = robot.GetPrimAtPath("/Robot/front_lidar_link/microScan3_f")
    assert not sensor or not sensor.IsActive()
    assert not robot.GetPrimAtPath("/Robot").HasAuthoredReferences()
    assert "https:" not in (assets / "robot.usda").read_text()
    excluded = UsdPhysics.FilteredPairsAPI(robot.GetPrimAtPath("/Robot/wrist3_Link")).GetFilteredPairsRel().GetTargets()
    assert [str(p) for p in excluded] == ["/Robot/gripper_motor_link"]
    manifest = json.loads((assets / "manifest.json").read_text())
    assert not manifest["artificial_rebar_support"]
    assert manifest["fixture_is_training_proxy"]


def test_fixture_has_four_physical_symmetric_jaws(assets):
    stage = Usd.Stage.Open(str(assets / "fixture.usda"))
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.PrismaticJoint)]
    assert len(joints) == 4
    for joint in joints:
        if "left" in joint.GetName():
            assert joint.GetAttribute("physics:lowerLimit").Get() < 0
        else:
            assert joint.GetAttribute("physics:upperLimit").Get() > 0
        body = stage.GetPrimAtPath(joint.GetRelationship("physics:body1").GetTargets()[0])
        assert stage.GetPrimAtPath(str(body.GetPath()) + "/collision").HasAPI(UsdPhysics.CollisionAPI)


def test_preflight_reports_pending_gpu_work_explicitly(assets):
    result = preflight(assets)
    assert result["offline_passed"] and not result["gpu_validated"]
    assert result["runtime"] == "not_checked"
    assert result["pending_on_4090"]


def test_simulation_preflight_does_not_import_usd_into_parent(assets):
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(Path(__file__).parents[1] / "src")
    code = ("import sys,json; from rebar_training.preflight import preflight_subprocess; "
            f"result=preflight_subprocess({str(assets)!r}); "
            "print(json.dumps({'passed':result['offline_passed'],'usd_loaded':'pxr' in sys.modules}))")
    result = subprocess.run([sys.executable, "-c", code], env=environment, capture_output=True,
                            text=True, timeout=30, check=True)
    assert json.loads(result.stdout) == {"passed": True, "usd_loaded": False}


def test_kinematic_steel_cannot_pass_asset_audit(assets):
    # Copying layer text to another location changes relative references, so edit
    # a local in-memory layer and export next to the originals, then restore it.
    path = assets / "rebar.usda"
    original = path.read_bytes()
    try:
        stage = Usd.Stage.Open(str(path))
        stage.GetPrimAtPath("/Rebar/body").GetAttribute("physics:kinematicEnabled").Set(True)
        stage.GetRootLayer().Save()
        with pytest.raises(RuntimeError, match="dynamic"):
            audit_assets(assets)
    finally:
        path.write_bytes(original)
        stage.GetRootLayer().Reload()
