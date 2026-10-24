"""Contract/media tests, not GPU rendering, Cosmos generation or physics evidence."""
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest
np = pytest.importorskip('numpy')
pytest.importorskip('PIL')
pytest.importorskip('jsonschema')
torch = pytest.importorskip('torch')

from rebar_training.vision.__main__ import demo
from rebar_training.vision.augment import plan, prepare_media, nim_request, letterbox_geometry
from rebar_training.vision.capture import load_config, optical_transform, CaptureRecorder, canonical_semantics, synchronize_mounts
from rebar_training.vision.dataset import DatasetWriter, validate
from rebar_training.core import TaskState

CONFIG = Path(__file__).parents[1] / 'config/vision_capture.json'


def fixture_dataset(tmp_path):
    root = tmp_path / 'dataset'
    demo(root)
    return root


def test_fixture_is_explicitly_not_physics_or_cosmos_data_and_validates(tmp_path):
    root = fixture_dataset(tmp_path)
    assert validate(root)['frames'] == 6
    metadata = json.loads((root/'dataset.json').read_text())
    assert metadata['fixture'] and not metadata['generated_by_cosmos'] and not metadata['runtime_validated']
    with pytest.raises(FileExistsError):
        demo(root)
    with pytest.raises(ValueError, match='fixture'):
        plan(root, tmp_path/'plan', 'wrist', ['steel under dim light'], frames=5)


@pytest.mark.parametrize("defect", ["escape", "corrupt"])
def test_corrupted_or_escaping_media_is_rejected(tmp_path, defect):
    root = fixture_dataset(tmp_path)
    frame_file = next(root.glob('episodes/*/frames.jsonl'))
    lines = frame_file.read_text().splitlines()
    first = json.loads(lines[0])
    if defect == 'corrupt':
        asset = root / first['cameras']['wrist']['image']
        asset.write_bytes(b'corrupt')
        with pytest.raises(ValueError, match='Corrupted'):
            validate(root)
        return
    first['cameras']['wrist']['image'] = '../../../../outside.png'
    lines[0] = json.dumps(first)
    frame_file.write_text('\n'.join(lines)+'\n')
    with pytest.raises(ValueError, match='escapes'):
        validate(root)


def test_missing_control_history_and_time_gaps_are_not_silently_accepted(tmp_path):
    root = fixture_dataset(tmp_path)
    frame_file = next(root.glob('episodes/*/frames.jsonl'))
    lines = frame_file.read_text().splitlines()
    row = json.loads(lines[1]); row['action_history'][0]['t_start_ns'] += 1; row['t_start_ns'] += 1
    lines[1] = json.dumps(row); frame_file.write_text('\n'.join(lines)+'\n')
    with pytest.raises(ValueError, match='Missing actions'):
        validate(root)


def test_letterbox_keeps_calibration_in_the_same_pixel_coordinates():
    size, offset, matrix = letterbox_geometry(1024, 615, [832, 480])
    assert size == (799, 480) and offset == (16, 0)
    center = matrix @ np.array([512., 307.5, 1.])
    assert np.allclose(center, [415.5, 240., 1.])
    assert np.allclose(optical_transform([1,2,3], [0,0,0,1]),
                       [[1,0,0,1],[0,1,0,2],[0,0,1,3],[0,0,0,1]])


@pytest.mark.parametrize('control', ['edge', 'depth', 'seg', 'blur'])
def test_offline_media_and_nim_request_roundtrip(tmp_path, control):
    if not shutil.which('ffmpeg'):
        pytest.skip('ffmpeg not installed')
    root = fixture_dataset(tmp_path)
    planned = plan(root, tmp_path/'plan', 'wrist', ['Keep geometry fixed; change lighting.'],
                   frames=5, control=control, allow_fixture=True)
    job = planned['jobs'][0]
    prepared = prepare_media(root, tmp_path/'plan/jobs.json', job['job_id'], tmp_path/'media')
    assert prepared['labels_reusable'] is False
    receipt = nim_request(tmp_path/'media', tmp_path/'request.json')
    request = json.loads((tmp_path/'request.json').read_text())
    assert not receipt['sent'] and request['model_mode'] == 'video2video'
    if control in ('edge', 'blur'):
        assert request['input_reference'].startswith('data:video/mp4;base64,')
    else:
        assert request['transfer'][control]['video'].startswith('data:video/mp4;base64,')
        assert 'input_reference' not in request
    assert request['num_frames'] == 5 and request['fps'] == 10


def test_no_transfer_support_is_claimed_for_edge_model(tmp_path):
    root = fixture_dataset(tmp_path)
    with pytest.raises(ValueError, match='Nano/Super'):
        plan(root, tmp_path/'plan', 'wrist', ['steel'], frames=5, model='Cosmos3-Edge', allow_fixture=True)


def test_other_frame_rates_require_explicit_resampling(tmp_path):
    root = fixture_dataset(tmp_path)
    path = root/'dataset.json'
    metadata=json.loads(path.read_text());metadata['nominal_fps']=15
    path.write_text(json.dumps(metadata))
    with pytest.raises(ValueError,match='resample'):
        plan(root,tmp_path/'plan','wrist',['steel'],frames=5,allow_fixture=True)


def test_capture_config_retains_existing_wrist_camera_and_separate_diagnostic_view():
    config = load_config(CONFIG)
    wrist, overview = config['cameras']
    assert wrist['mode'] == 'existing' and wrist['mono'] and wrist['policy_observable']
    assert not overview['policy_observable']
    transform=np.asarray(optical_transform(overview['position_env_m'], overview['quaternion_xyzw']))
    points=np.array([[-1.45,0,.522],[-1.15,0,.522],[-.3,-.77,1.5],[-.3,-.77,1.8]])
    optical=(points-transform[:3,3]) @ transform[:3,:3]
    focal=overview['width']*overview['focal_length_mm']/overview['horizontal_aperture_mm']
    pixels=optical[:,:2]/optical[:,2:] * focal + np.array([overview['width']/2,overview['height']/2])
    assert (optical[:,2]>0).all()
    assert ((pixels>=0)&(pixels<np.array([overview['width'],overview['height']]))).all()


@pytest.mark.parametrize("stale", [False, True])
def test_recorder_keeps_terminal_frame_and_sparse_intervening_actions(tmp_path, stale):
    proxy = lambda x: SimpleNamespace(torch=x)
    outputs = {'rgb': proxy(torch.full((1, 16, 16, 3), 120, dtype=torch.uint8)),
               'distance_to_image_plane': proxy(torch.ones(1,16,16,1)),
               'semantic_segmentation': proxy(torch.ones(1,16,16,1,dtype=torch.int32))}
    data = SimpleNamespace(output=outputs, info={'semantic_segmentation': {'idToLabels': {'1': {'class': 'rebar'}}}},
           intrinsic_matrices=proxy(torch.tensor([[[10.,0,8],[0,10,8],[0,0,1]]])),
           pos_w=proxy(torch.zeros(1,3)), quat_w_ros=proxy(torch.tensor([[0.,0,0,1]])))
    camera = SimpleNamespace(data=data, frame=proxy(torch.tensor([1])))
    env = SimpleNamespace(vision_config={'stride':3,'env_ids':[0], 'cameras':[{'name':'wrist','mono':True}]},
           cfg=SimpleNamespace(camera_dataset_root=tmp_path/'capture',task='loading',seed=42),
           task_spec=SimpleNamespace(control_dt=1/30), observation_contract='loading_route_v1',
           vision_cameras={'wrist':camera}, scene=SimpleNamespace(env_origins=torch.zeros(1,3)),
           robot=SimpleNamespace(data=SimpleNamespace(joint_pos=proxy(torch.zeros(1,8)),joint_vel=proxy(torch.zeros(1,8)))),
           bar=SimpleNamespace(data=SimpleNamespace(root_quat_w=proxy(torch.tensor([[0.,0,0,1]])))),
           logic=SimpleNamespace(phase=torch.tensor([0]),pick_complete=torch.tensor([False]),elapsed=torch.tensor([4])),
           control_ids=list(range(8)),actions=torch.zeros(1,7),targets=torch.zeros(1,8),route=None)
    state = TaskState(tcp=torch.zeros(1,3),bar=torch.ones(1,3),bar_axis=torch.tensor([[1.,0,0]]),
                     linear_velocity=torch.zeros(1,3),angular_velocity=torch.zeros(1,3),fingers=torch.zeros(1,2),
                     finger_forces=torch.zeros(1,2),jaw_forces=torch.zeros(1,4),unsafe_force=torch.zeros(1),assisted=torch.tensor([False]))
    result = SimpleNamespace(terminated=torch.tensor([False]),truncated=torch.tensor([False]),
                             success=torch.tensor([False]),failure=torch.tensor([False]))
    recorder = CaptureRecorder(env);recorder.reset(torch.tensor([0]))
    for _ in range(3):recorder.record(state,result)
    if stale:
        result.terminated[:] = True
        with pytest.raises(RuntimeError, match='Stale camera'):
            recorder.record(state, result)
        recorder.close()
        assert validate(tmp_path/'capture')['frames'] == 1
        return
    camera.frame.torch[:] = 2
    result.terminated[:] = True;result.failure[:] = True
    recorder.record(state,result);recorder.close()
    rows=[json.loads(line) for line in next((tmp_path/'capture').glob('episodes/*/frames.jsonl')).read_text().splitlines()]
    assert len(rows[0]['action_history']) == 3
    assert rows[-1]['terminal'] and not rows[-1]['on_capture_grid']
    assert validate(tmp_path/'capture')['frames'] == 2


def test_stale_frames_fail_instead_of_pairing_with_new_state(tmp_path):
    root = fixture_dataset(tmp_path)
    p=next(root.glob('episodes/*/frames.jsonl')); rows=[json.loads(s) for s in p.read_text().splitlines()]
    rows[0]['cameras']['wrist']['sensor_t_end_ns'] -= 1
    p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    with pytest.raises(ValueError,match='Unsynchronized'):
        validate(root)


def test_rtx_packed_semantics_are_paired_with_integer_class_mapping():
    mapping = {'idToLabels': {'(0, 0, 0, 0)': {'class': 'BACKGROUND'},
                              '(220, 179, 62, 255)': {'class': 'rebar'}}}
    ids, normalized = canonical_semantics(np.array([[0, -12667940]], np.int32), mapping)
    assert ids.tolist() == [[0, 3]]
    assert normalized['idToLabels']['3']['class'] == 'rebar'
    more = {'idToLabels': {**mapping['idToLabels'], '(1, 2, 3, 255)': {'class': 'machine'}}}
    assert canonical_semantics(np.array([[-12667940]], np.int32), more)[0].item() == 3
    with pytest.raises(RuntimeError, match='Unmapped'):
        canonical_semantics(np.array([[123]], np.int32), mapping)


def test_moving_camera_uses_measured_link_pose_and_fixed_optical_extrinsics():
    calls = []
    camera = SimpleNamespace(set_world_poses=lambda p, q, **kw: calls.append((p, q, kw)))
    robot = SimpleNamespace(find_bodies=lambda name: ([0], [name]), data=SimpleNamespace(
        body_pos_w=SimpleNamespace(torch=torch.tensor([[[1., 2., 3.]]])),
        body_quat_w=SimpleNamespace(torch=torch.tensor([[[0., 0., 0., 1.]]]))))
    env = SimpleNamespace(device='cpu', num_envs=1, robot=robot, vision_cameras={'wrist': camera},
        vision_mounts={'wrist': {'body': 'mono_camera_link', 'position': [.1, 0, 0],
                                 'quaternion_xyzw': [1., 0., 0., 0.]}})
    synchronize_mounts(env)
    torch.testing.assert_close(calls[0][0], torch.tensor([[1.1, 2., 3.]]))
    assert calls[0][2]['convention'] == 'ros'
    robot.data.body_pos_w.torch[0, 0, 0] = 2
    synchronize_mounts(env)
    torch.testing.assert_close(calls[-1][0], torch.tensor([[2.1, 2., 3.]]))


def test_rejected_capture_cannot_be_sent_to_augmentation(tmp_path):
    root = fixture_dataset(tmp_path)
    (root/'quality_review.json').write_text(json.dumps({'status': 'rejected', 'issues': ['stale camera mount']}))
    with pytest.raises(ValueError, match='quality review rejected'):
        plan(root, tmp_path/'plan', 'wrist', ['steel'], frames=5, allow_fixture=True)


def test_augmentation_clips_retain_the_last_uniform_frame_of_handoff(tmp_path):
    root = fixture_dataset(tmp_path)
    result = plan(root, tmp_path/'plan', 'wrist', ['steel'], frames=5, allow_fixture=True)
    assert len(result['jobs']) == 2
    assert result['jobs'][-1]['frames'][-1]['frame_index'] == 5
