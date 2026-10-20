import json
import pytest
pytest.importorskip('pxr')
from pxr import Usd,UsdPhysics
from rebar_training.assets import build_assets
from rebar_training.mobile_assets import build_mobile_assets,audit_mobile_assets


def test_mobile_variant_keeps_dynamic_steel_and_real_tray_load_link(tmp_path):
    parked=tmp_path/'parked';mobile=tmp_path/'mobile'
    build_assets(parked,fixture_model='full_v2')
    before=(parked/'robot.usda').read_bytes()
    result=build_mobile_assets(parked,mobile)
    assert (parked/'robot.usda').read_bytes()==before
    assert not result['base_world_attachment'] and result['tray_contact_colliders']==74
    assert result['rebar_dynamic'] and result['rebar_collision_enabled']
    manifest=json.loads((mobile/'manifest.json').read_text())
    assert not manifest['gpu_validated'] and not manifest['artificial_rebar_support']
    s=Usd.Stage.Open(str(mobile/'robot.usda'))
    mount=UsdPhysics.FixedJoint(s.GetPrimAtPath('/Robot/tray_mount_joint'))
    assert str(mount.GetBody1Rel().GetTargets()[0])=='/Robot/tray_link'
    # A reintroduced parking lock must be caught before launching the mission.
    s.GetPrimAtPath('/Robot/training_base_lock').SetActive(True);s.GetRootLayer().Save()
    with pytest.raises(RuntimeError,match='locked'):
        audit_mobile_assets(mobile)
