import copy
from rebar_training.mobile_acceptance import acceptance,wilson_lower


def receipt(seed,n=32):
    rows=[dict(episode=i,success=True,until='clamp',rebar_pose_writes_after_reset=False,
        actual_handoff=True,machine_error_m=.002,unsafe_force_N=50.,tray_verified=True,
        mission_transitions=[{'to_stage':s} for s in ('tray_release','tray_retreat','observe_tray',
            'navigate','dock','pickup_tray','tray_lift_high','loading')]) for i in range(n)]
    return dict(completed=True,seed=seed,privileged_control_guard=True,steel_truth_control=False,
        residual_policy=True,residual_checkpoint_sha256='a'*64,randomization={'name':'expanded'},
        residual_total_updates=10,residual_trained_steps=1000,
        residual_actor_updates=10,residual_reward_contract='mobile_residual_potential_v2',
        source_sha256='c'*64,detector_sha256='d'*64,asset_manifest_sha256='e'*64,
        end_to_end=True,steel_initialized_on_tray_at_reset=False,results=rows,episodes=n,requested_episodes=n)


def test_small_perfect_sample_is_not_stable_and_full_64_gate_passes():
    assert not acceptance([receipt(1,1),receipt(2,1)])['passed']
    assert acceptance([receipt(1),receipt(2)])['passed']
    assert wilson_lower(2,2)<.5 and wilson_lower(64,64)>.94


def test_training_duplicate_seed_pose_cheat_and_mixed_weights_fail_closed():
    baseline=[receipt(1),receipt(2)]
    for field,value in [('residual_training',True),('seed',1),('residual_checkpoint_sha256','b'*64),
                        ('steel_initialized_on_tray_at_reset',True),('steel_truth_control',True)]:
        reports=copy.deepcopy(baseline);reports[1][field]=value
        assert not acceptance(reports)['passed'],field
    reports=copy.deepcopy(baseline);reports[1]['results'][0]['rebar_pose_writes_after_reset']=True
    assert not acceptance(reports)['passed']


def test_false_success_and_missing_episodes_are_not_allowed_to_inflate_success_rate():
    reports=[receipt(1),receipt(2)]
    reports[0]['results'][0]['machine_error_m']=.0031
    assert not acceptance(reports)['passed']
    reports=[receipt(1),receipt(2)];reports[1]['results'].pop()
    assert not acceptance(reports)['passed']


def test_failures_are_retained_in_denominator_and_diagnostics():
    reports=[receipt(1),receipt(2)]
    reports[0]['results'][0].update(success=False,failure='stage timeout: tray_raise',stage='tray_raise')
    result=acceptance(reports)
    assert result['episodes']==64 and result['successes']==63
    assert result['failures'][0]['stage']=='tray_raise'
