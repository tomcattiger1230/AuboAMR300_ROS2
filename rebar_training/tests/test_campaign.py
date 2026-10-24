from rebar_training.campaign import policy_gate


def report(checkpoint):
    return {'mode': 'checkpoint_evaluation', 'completed': True, 'finite': True,
            'task': 'loading', 'observation_contract': 'loading_route_v1', 'seed': 20042, 'checkpoint': str(checkpoint.resolve()),
            'episodes': [{'success': True, 'failure': False, 'timeout': False, 'picked': True} for _ in range(100)]}


def test_policy_gate_requires_continuous_pick_and_real_policy_not_expert(tmp_path):
    checkpoint = tmp_path / 'model.pt'
    data = report(checkpoint)
    assert policy_gate(data, 'loading', 20042, 100, .95, checkpoint)
    for item in data['episodes'][:6]:
        item['picked'] = False
    assert not policy_gate(data, 'loading', 20042, 100, .95, checkpoint)
    data = report(checkpoint)
    data['mode'] = 'expert_physical_check'
    assert not policy_gate(data, 'loading', 20042, 100, .95, checkpoint)
    data['mode'] = 'online_dagger'
    assert not policy_gate(data, 'loading', 20042, 100, .95, checkpoint)


def test_policy_gate_rejects_stale_seed_incomplete_or_nonfinite_result(tmp_path):
    checkpoint = tmp_path / 'model.pt'
    data = report(checkpoint)
    assert not policy_gate(data, 'loading', 10042, 100, .95, checkpoint)
    assert not policy_gate(data, 'loading', 20042, 101, .95, checkpoint)
    data['finite'] = False
    assert not policy_gate(data, 'loading', 20042, 100, .95, checkpoint)
    data['finite'] = True
    data['episodes'] = data['episodes'][:90]
    data['success_rate'] = 1.0
    assert not policy_gate(data, 'loading', 20042, 100, .95, checkpoint)
