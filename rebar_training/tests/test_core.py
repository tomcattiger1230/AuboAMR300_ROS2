"""Behavior tests of the same tensor logic used by the real DirectRLEnv."""

import math

import pytest
torch = pytest.importorskip("torch")

from rebar_training.core import EpisodeLogic, TaskState, action_targets
from rebar_training.reset import ResetSampler, quat_apply, quat_multiply
from rebar_training.spec import TaskSpec


def state(n=1, task="grasp"):
    bar = torch.tensor([[-1.15, 0.0, 0.65]]).repeat(n, 1)
    if task == "insert":
        bar[:] = torch.tensor(TaskSpec().insert_goal)
    return TaskState(tcp=bar.clone(), bar=bar, bar_axis=torch.tensor([[0.0, 0.0, 1.0]]).repeat(n, 1),
                     linear_velocity=torch.zeros(n, 3), angular_velocity=torch.zeros(n, 3),
                     fingers=torch.full((n, 2), 0.017), finger_forces=torch.ones(n, 2),
                     jaw_forces=torch.zeros(n, 4), unsafe_force=torch.zeros(n),
                     assisted=torch.zeros(n, dtype=torch.bool))


def logic(task="grasp", n=1, **kwargs):
    spec = TaskSpec(task=task, **kwargs)
    tracker = EpisodeLogic(spec, n)
    tracker.reset(list(range(n)), torch.full((n,), 0.522 if task in ("grasp", "loading") else 1.5))
    return tracker


def step(tracker, measured, actions=None):
    return tracker.advance(measured, torch.zeros(tracker.num_envs, 7) if actions is None else actions)


def test_grasp_requires_sustained_lift_and_contact():
    tracker, measured = logic(), state()
    for _ in range(tracker.spec.grasp_hold_steps - 1):
        result = step(tracker, measured)
        assert not result.success.any()
        assert (result.reward <= 0).all()
    result = step(tracker, measured)
    assert result.success.all() and result.terminated.all() and not result.truncated.any()
    assert result.reward.item() > 0


@pytest.mark.parametrize("defect", ["one_finger", "far_tcp", "too_low", "moving"])
def test_closed_gripper_or_single_pose_is_not_grasp_success(defect):
    tracker, measured = logic(), state()
    if defect == "one_finger":
        measured.finger_forces[:, 1] = 0
    elif defect == "far_tcp":
        measured.tcp[:, 0] += 0.1
    elif defect == "too_low":
        measured.bar[:, 2] = 0.55
    else:
        measured.linear_velocity[:, 0] = 0.1
    for _ in range(tracker.spec.grasp_hold_steps + 1):
        assert not step(tracker, measured).success.any()


def test_lost_contact_resets_grasp_dwell():
    tracker, measured = logic(), state()
    for _ in range(8):
        step(tracker, measured)
    measured.finger_forces[:, 1] = 0
    step(tracker, measured)
    assert tracker.held.item() == 0
    measured.finger_forces[:] = 1
    for _ in range(tracker.spec.grasp_hold_steps - 1):
        assert not step(tracker, measured).success.any()


def align_fixture(tracker, measured):
    for _ in range(tracker.spec.align_hold_steps):
        result = step(tracker, measured)
        assert not result.success.any()
    assert result.close_fixture.all()


def release(measured):
    measured.jaw_forces[:] = 1
    measured.finger_forces[:] = 0
    measured.fingers[:] = 0
    measured.tcp[:, 0] += 0.08


def test_insertion_requires_physical_handoff_and_retraction():
    tracker, measured = logic("insert"), state(task="insert")
    align_fixture(tracker, measured)
    # An aligned, grasped bar with the fixture commanded closed is insufficient.
    for _ in range(10):
        assert not step(tracker, measured).success.any()
    release(measured)
    for _ in range(tracker.spec.handoff_hold_steps - 1):
        assert not step(tracker, measured).success.any()
    assert step(tracker, measured).success.all()


@pytest.mark.parametrize("defect", ["no_fixture_contact", "one_missing_pad", "not_retracted", "robot_not_open", "tilted", "off_target", "moving"])
def test_false_handoff_is_rejected(defect):
    tracker, measured = logic("insert"), state(task="insert")
    align_fixture(tracker, measured)
    release(measured)
    if defect == "no_fixture_contact":
        measured.jaw_forces[:] = 0
    elif defect == "one_missing_pad":
        measured.jaw_forces[:, 3] = 0
    elif defect == "not_retracted":
        measured.tcp[:] = measured.bar
    elif defect == "robot_not_open":
        measured.fingers[:] = 0.02
    elif defect == "tilted":
        measured.bar_axis[:] = torch.tensor([1.0, 0.0, 0.0])
    elif defect == "off_target":
        measured.bar[:, 0] += 0.03
    else:
        measured.angular_velocity[:, 0] = 1
    for _ in range(tracker.spec.handoff_hold_steps + 1):
        assert not step(tracker, measured).success.any()


@pytest.mark.parametrize("defect", ["nan", "zero_axis", "dropped", "over_force", "assisted", "nan_action"])
def test_failure_precedes_success_and_is_finite(defect):
    tracker, measured = logic(), state()
    for _ in range(tracker.spec.grasp_hold_steps - 1):
        step(tracker, measured)
    actions = torch.zeros(1, 7)
    if defect == "nan":
        measured.bar[:, 0] = float("nan")
    elif defect == "zero_axis":
        measured.bar_axis[:] = 0
    elif defect == "dropped":
        measured.bar[:, 2] = 0.3
    elif defect == "over_force":
        measured.unsafe_force[:] = 300
    elif defect == "assisted":
        measured.assisted[:] = True
    else:
        actions[:, 2] = float("nan")
    result = step(tracker, measured, actions)
    assert result.failure.all() and result.terminated.all() and not result.success.any()
    assert torch.isfinite(result.reward).all()
    with pytest.raises(RuntimeError, match="reset"):
        step(tracker, measured)


def test_timeout_is_truncation_not_success():
    tracker, measured = logic(episode_seconds=0.6), state()
    measured.finger_forces[:] = 0
    for _ in range(tracker.spec.episode_steps):
        result = step(tracker, measured)
    assert result.truncated.all() and not result.terminated.any() and not result.success.any()


def test_reported_position_error_uses_each_tasks_own_goal():
    grasp = logic()
    measured = state()
    measured.bar[:, 2] = 0.55
    result = step(grasp, measured)
    assert result.terms["position_error_m"].item() == pytest.approx(0.092, abs=1e-6)
    insert = logic("insert")
    measured = state(task="insert")
    measured.bar[:, 0] += 0.01
    result = step(insert, measured)
    assert result.terms["position_error_m"].item() == pytest.approx(0.01, abs=1e-6)


def test_partial_reset_clears_only_selected_episode_memory():
    tracker, measured = logic(n=3), state(3)
    for _ in range(4):
        step(tracker, measured)
    before = {name: getattr(tracker, name).clone() for name in ("elapsed", "held", "initial_height", "previous_actions")}
    tracker.reset([1], torch.tensor([0.6]))
    assert tracker.elapsed.tolist() == [4, 0, 4]
    assert tracker.held.tolist() == [4, 0, 4]
    assert tracker.initial_height[1] == 0.6
    for name, values in before.items():
        torch.testing.assert_close(getattr(tracker, name)[[0, 2]], values[[0, 2]])


def test_action_limits_and_target_anti_windup():
    spec = TaskSpec()
    measured = torch.zeros(2, 8)
    targets = torch.ones(2, 8)
    actions = torch.full((2, 7), 10.0)
    lower, upper = torch.full((2, 8), -1.0), torch.ones(2, 8)
    output = action_targets(actions, measured, targets, lower, upper, spec)
    assert (output[:, :6] <= spec.target_lead).all()
    torch.testing.assert_close(output[:, 6:], torch.full((2, 2), spec.finger_travel))
    actions[:, 6] = -1
    assert (action_targets(actions, measured, targets, lower, upper, spec)[:, 6:] == 0).all()


def test_reset_is_seeded_bounded_and_keeps_bar_on_station():
    first, second = ResetSampler(42), ResetSampler(42)
    a, b = first.sample("grasp", 100), second.sample("grasp", 100)
    torch.testing.assert_close(a.joints, b.joints)
    torch.testing.assert_close(a.bar_pose, b.bar_pose)
    torch.testing.assert_close(a.bar_pose[:, :2], a.station_pose[:, :2])
    assert (a.bar_pose[:, 2] == 0.522).all()
    assert (a.station_pose[:, 0] + 1.15).abs().max() <= 0.03
    torch.testing.assert_close(torch.linalg.vector_norm(a.bar_pose[:, 3:], dim=-1), torch.ones(100))
    c = first.sample("grasp", 100)
    assert not torch.equal(a.bar_pose, c.bar_pose)


def test_quaternion_convention_is_xyzw_and_composes():
    identity = torch.tensor([[0.0, 0.0, 0.0, 1.0]])
    q = torch.tensor([[0.0, -2 ** -0.5, 0.0, 2 ** -0.5]])
    x = torch.tensor([[1.0, 0.0, 0.0]])
    torch.testing.assert_close(quat_apply(q, x), torch.tensor([[0.0, 0.0, 1.0]]), atol=1e-6, rtol=0)
    torch.testing.assert_close(quat_multiply(identity, q), q)


@pytest.mark.parametrize("kwargs", [{"task": "invalid"}, {"control_dt": 0}, {"lift_height": float("nan")},
                                   {"position_tolerance": -1}, {"episode_seconds": 0.1},
                                   {"axis_tolerance_rad": math.pi}, {"insert_goal": (0, 0, float("inf"))}])
def test_bad_task_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        TaskSpec(**kwargs)


@pytest.mark.parametrize("task", ["grasp", "insert"])
def test_early_failure_cannot_save_remaining_episode_running_cost(task):
    tracker = logic(task=task)
    measured = state()
    measured.bar[:, 2] = 0.0  # immediate drop
    result = step(tracker, measured)
    assert result.failure.all()
    spec = tracker.spec
    worst_step_cost = 8 * spec.control_dt + .01 * 7 + .02 * 4 * 7 + .01
    # The terminal charge alone covers every possible remaining running cost.
    assert (result.reward < -10 - (spec.episode_steps - 1) * worst_step_cost).all()
    measured.bar[:, 0] = float("nan")
    tracker.reset([0], torch.tensor([0.5]))
    invalid = step(tracker, measured)
    assert torch.isfinite(invalid.reward).all()
    assert (invalid.reward < -spec.episode_steps * worst_step_cost).all()


def test_grasp_phase_exposes_measured_contact_dwell_and_resets_on_loss():
    tracker, measured = logic(), state()
    measured.bar[:, 2] = .522
    measured.tcp[:, 2] = .522
    step(tracker, measured)
    assert tracker.phase.item() == 1
    for _ in range(7):
        step(tracker, measured)
    assert tracker.phase.item() == 2 and tracker.progress()[0, 1] == 1
    measured.finger_forces[:, 0] = 0
    step(tracker, measured)
    assert tracker.phase.item() == 0 and tracker.contact_dwell.item() == 0


def test_loading_requires_real_pick_before_fixture_handoff():
    tracker, measured = logic(task='loading'), state(task='insert')
    # Holding an already inserted bar cannot skip the pickup requirement.
    measured.finger_forces[:] = 0
    measured.jaw_forces[:] = 1
    measured.fingers[:] = 0
    measured.tcp[:, 0] += .08
    for _ in range(35):
        result = step(tracker, measured)
        assert not result.success.any() and not result.close_fixture.any()
    assert not tracker.pick_complete.any()


def test_loading_drop_is_relative_to_peak_lift_not_only_initial_rack():
    tracker, measured = logic(task='loading'), state()
    measured.bar[:, 2] = 1.2
    step(tracker, measured)
    measured.bar[:, 2] = 1.0
    assert step(tracker, measured).failure.all()


def test_loading_pick_latch_then_physical_handoff_succeeds():
    tracker, measured = logic(task='loading'), state()
    for _ in range(tracker.spec.grasp_hold_steps):
        result = step(tracker, measured)
        assert not result.success.any()
    assert tracker.pick_complete.all() and tracker.phase.item() == 1
    measured.bar[:] = torch.tensor(tracker.spec.insert_goal)
    measured.tcp[:] = measured.bar
    for _ in range(tracker.spec.align_hold_steps):
        result = step(tracker, measured)
    assert result.close_fixture.all()
    release(measured)
    for _ in range(tracker.spec.handoff_hold_steps):
        result = step(tracker, measured)
    assert result.success.all() and not result.failure.any()


def test_loading_deliberate_lowering_while_physically_grasped_is_not_drop():
    tracker, measured = logic(task='loading'), state()
    measured.bar[:, 2] = measured.tcp[:, 2] = 1.2
    step(tracker, measured)
    measured.bar[:, 2] = measured.tcp[:, 2] = 1.0
    assert not step(tracker, measured).failure.any()
