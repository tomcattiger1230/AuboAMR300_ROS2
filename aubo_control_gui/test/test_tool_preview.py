from pathlib import Path
import numpy as np
from aubo_control_gui.tool_preview import tool_visuals


def test_tool_mount_and_independent_finger_axes():
    repo = Path(__file__).parents[2]
    parts = {p['link']: p for p in tool_visuals(repo/'seer_description/urdf/composite_robot_stick_mono.urdf')}
    assert set(parts) == {'gripper_adapter_link','gripper_motor_link','gripper1_link','gripper2_link','camera_link'}
    np.testing.assert_allclose(parts['gripper_motor_link']['position'], [-.02,0,.035], atol=1e-12)
    left, right = parts['gripper1_link'], parts['gripper2_link']
    # The URDF prismatic axes rotate into opposite wrist-frame X directions.
    np.testing.assert_allclose(left['axis'], [1,0,0], atol=1e-12)
    np.testing.assert_allclose(right['axis'], [-1,0,0], atol=1e-12)
    assert (left['finger'],right['finger']) == (0,1)
    before = np.linalg.norm(np.array(left['position'])-right['position'])
    after = np.linalg.norm(np.array(left['position'])+.04*np.array(left['axis'])
                           -np.array(right['position'])-.04*np.array(right['axis']))
    assert abs(before-after-.08) < 1e-12
    for part in parts.values():
        r = np.array(part['rotation'])
        np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-12)
        if part['shape'] == 'mesh':
            assert (repo/'aubo_control_gui/meshes/gripper'/part['mesh']).is_file()


def test_camera_body_and_lens_follow_gripper_mount():
    repo = Path(__file__).parents[2]
    parts = tool_visuals(repo/'seer_description/urdf/composite_robot_stick_mono.urdf')
    camera = [p for p in parts if p['link'] == 'camera_link']
    assert len(camera) == 2
    body, lens = camera
    assert (body['shape'], lens['shape']) == ('box', 'cylinder')
    np.testing.assert_allclose(body['scale'], [.029, .044, .059])
    np.testing.assert_allclose(lens['scale'], [.035, .04, .035])
    assert body['finger'] == lens['finger'] == -1
    np.testing.assert_allclose(np.linalg.norm(np.array(lens['position'])-body['position']), .0495)
    assert np.linalg.norm(body['position']) < .25
