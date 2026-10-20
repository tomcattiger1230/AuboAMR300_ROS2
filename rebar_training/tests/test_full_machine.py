"""The full model adds physical obstructions, not a visual overlay or bar hold."""
import json
import pytest
pytest.importorskip("pxr")
from pxr import Usd, UsdGeom, UsdPhysics
from rebar_training.assets import build_assets, audit_assets


def test_full_machine_geometry_and_six_axis_reset_contract(tmp_path):
    result = build_assets(tmp_path, fixture_model="full_v2")
    assert result["fixture_joint_count"] == 6
    assert result["rebar_dynamic"] and result["rebar_collision_enabled"]
    stage = Usd.Stage.Open(str(tmp_path / "fixture.usda"))
    cache = UsdGeom.XformCache()
    for label, height, tip_offset, limits in (
        ("upper", 1.88, -.075, (1.50, 1.88)), ("lower", 1.136, .075, (.92, 1.34))):
        body = stage.GetPrimAtPath(f"/Fixture/{label}_carriage")
        joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(f"/Fixture/{label}_carriage_joint"))
        assert joint.GetAxisAttr().Get() == "Z"
        assert height + joint.GetLowerLimitAttr().Get() == pytest.approx(limits[0])
        assert height + joint.GetUpperLimitAttr().Get() == pytest.approx(limits[1])
        assert not body.GetChild("Crosshead").HasAPI(UsdPhysics.CollisionAPI)
        assert body.GetChild("CrossheadCollider").HasAPI(UsdPhysics.CollisionAPI)
        for side, sign in (("left", 1), ("right", -1)):
            pad = stage.GetPrimAtPath(f"/Fixture/{label}_{side}")
            tip = pad.GetChild("collision")
            assert tuple(cache.GetLocalToWorldTransform(tip).ExtractTranslation()) == pytest.approx(
                (-.87, -.08 + sign * .10, height + tip_offset))
            assert pad.GetChild("Jaw").HasAPI(UsdPhysics.CollisionAPI)
            # Solid jaw bodies must leave clearance for the 0.6 m bar ends.
            jaw_center = cache.GetLocalToWorldTransform(pad.GetChild("Jaw")).ExtractTranslation()[2]
            if label == "upper":
                assert jaw_center - .13 / 2 >= 1.508 + .3 + .006
            else:
                assert jaw_center + .13 / 2 <= 1.508 - .3 - .006
        if label == "lower":
            # Bottom of the moving crosshead must clear the static platen top.
            assert height - .19 - .15 / 2 >= .83 + .08 / 2 + .0005
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["fixture_model"] == "full_v2"
    assert manifest["fixture_is_training_proxy"] and not manifest["gpu_validated"]
    # A lost physical jaw must fail audit, even if its rendered shape remains.
    stage.GetPrimAtPath("/Fixture/upper_left/Jaw").RemoveAPI(UsdPhysics.CollisionAPI)
    stage.GetRootLayer().Save()
    with pytest.raises(RuntimeError, match="jaw collision"):
        audit_assets(tmp_path)
