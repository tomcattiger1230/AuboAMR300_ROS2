"""Prevent the old console collision using the actual authored CAD."""
import math
import pytest
pytest.importorskip('pxr')
from rebar_training.assets import build_assets
from rebar_training.navigation_map import validated_detour,chassis_clear


def test_full_machine_console_blocks_old_route_and_detour_clears_it(tmp_path):
    build_assets(tmp_path,fixture_model='full_v2')
    goals,obstacles=validated_detour(tmp_path)
    assert any('ConsoleCabinet' in b['path'] for b in obstacles)
    assert not chassis_clear((4.5,3.1,math.pi/2),obstacles)
    assert goals[0][0]<4.
    assert chassis_clear((6.3,4.77,0.),obstacles)
    assert not chassis_clear((6.,4.,0.),obstacles)
    # A proposed destination inside the console must fail before simulation.
    with pytest.raises(RuntimeError,match='collision|intersects'):
        validated_detour(tmp_path,dock=(4.85,3.96,0.))
