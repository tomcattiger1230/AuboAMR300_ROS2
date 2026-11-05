"""Regression checks for real maps containing different path representations."""
import sys
from pathlib import Path

import pytest
from PySide6.QtWidgets import QApplication

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aubo_control_gui.unified.seer.map_view import MapView
from aubo_control_gui.unified.seer.smap import load_smap_file


@pytest.fixture(scope='module')
def view():
    app = QApplication.instance() or QApplication([])
    widget = MapView()
    yield widget
    widget.close()


def test_hsjc01_nurbs_keeps_all_controls_and_reverse_geometry(view):
    smap = load_smap_file(Path(__file__).resolve().parents[2] / 'maps/realworld/HSJC01.smap')
    curves = {c['instanceName']: c for c in smap.advanced_curves if c['className'] == 'NURBS6'}
    forward = view._curve_points(curves['LM1-LM3'])
    reverse = view._curve_points(curves['LM3-LM1'])
    assert len(forward) == 6
    assert forward[3:5] == [(28.817, -17.47), (35.51, -18.182)]
    assert forward == list(reversed(reverse))


def test_hsjc04_supported_curves_join_station_endpoints(view):
    smap = load_smap_file(Path(__file__).resolve().parents[2] / 'maps/realworld/HSJC04.smap')
    for curve in smap.advanced_curves:
        points = view._curve_points(curve)
        start, end = curve['startPos']['pos'], curve['endPos']['pos']
        assert points[0] == (start['x'], start['y'])
        assert points[-1] == (end['x'], end['y'])
        assert len(points) == (2 if curve['className'] == 'StraightPath' else 33)


def test_nurbs_missing_controls_is_not_drawn_as_cubic(view):
    assert view._curve_points({'className': 'NURBS6', 'startPos': {'x': 0, 'y': 0},
                              'endPos': {'x': 1, 'y': 1}}) == []


def test_straight_path_ignores_unused_controls_and_unknown_type_is_hidden(view):
    curve = {'className': 'StraightPath', 'startPos': {'x': 0, 'y': 0},
             'endPos': {'x': 3, 'y': 0}, 'controlPos1': {'x': 0, 'y': 20}}
    assert view._curve_points(curve) == [(0, 0), (3, 0)]
    curve['className'] = 'UnknownPath'
    assert view._curve_points(curve) == []
