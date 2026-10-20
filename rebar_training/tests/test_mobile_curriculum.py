import copy
import json
from pathlib import Path
import pytest
from rebar_training.mobile_curriculum import validate_plan,require_successful_training


def plan():
    return json.loads((Path(__file__).parents[1]/'config/mobile_curriculum.json').read_text())


def test_frozen_curriculum_predeclares_disjoint_training_and_test_seeds():
    validate_plan(plan())
    invalid=plan();invalid['phases'][-1]['evaluation_seeds'][0]=invalid['phases'][0]['train_seed']
    with pytest.raises(ValueError,match='overlap'):validate_plan(invalid)


def test_curriculum_cannot_skip_a_gate_or_promote_reset_only_evidence_to_full():
    invalid=plan();invalid['phases']=invalid['phases'][1:]
    with pytest.raises(ValueError,match='in order'):validate_plan(invalid)
    invalid=plan();invalid['phases'][-1]['start_on_tray']=True
    with pytest.raises(ValueError,match='Full mission'):validate_plan(invalid)
    invalid=plan();invalid['phases'][-1]['evaluation_episodes_per_seed']=4
    with pytest.raises(ValueError,match='Insufficient'):validate_plan(invalid)


def test_evaluation_pool_bounds_resources_and_joins_workers_on_failure():
    import threading,time
    from rebar_training.mobile_curriculum import evaluate_group
    lock=threading.Lock();active=maximum=finished=0
    def task(index):
        nonlocal active,maximum,finished
        with lock:active+=1;maximum=max(maximum,active)
        try:
            time.sleep(.02)
            if index==1:raise RuntimeError('deliberate simulator failure')
            return index
        finally:
            with lock:active-=1;finished+=1
    with pytest.raises(RuntimeError,match='deliberate'):
        evaluate_group(task,[(index,) for index in range(6)],2)
    assert active==0 and finished==6 and maximum==2
    invalid=plan();invalid['evaluation_parallel_jobs']=4
    with pytest.raises(ValueError,match='concurrency'):validate_plan(invalid)


def test_training_failure_or_critic_only_updates_cannot_promote_to_holdouts():
    report=dict(completed=True,episodes=2,successes=2,
        results=[dict(success=True,failure=None),dict(success=True,failure=None)],
        ppo_updates=[dict(active_steps=64)])
    require_successful_training(report)
    failed=copy.deepcopy(report);failed['successes']=1;failed['results'][1]=dict(success=False,failure='stage timeout')
    with pytest.raises(RuntimeError,match='Training episode failed'):require_successful_training(failed)
    failed=copy.deepcopy(report);failed['ppo_updates']=[dict(active_steps=0)]
    with pytest.raises(RuntimeError,match='No new active actor'):require_successful_training(failed)
