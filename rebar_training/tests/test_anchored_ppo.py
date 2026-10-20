import copy

import pytest
import torch

pytest.importorskip('rsl_rl')
from rsl_rl.models import MLPModel
from rsl_rl.storage import RolloutStorage
from tensordict import TensorDict
from rebar_training.anchored_ppo import AnchoredPPO
from rebar_training.route_actor import RouteActor


def test_route_policy_resume_keeps_prior_labels_in_both_replay_pools(tmp_path):
    observations = torch.randn(8, 71)
    observations[:, 53] = .4
    td = TensorDict({'policy': observations}, batch_size=[8])
    distribution = {'class_name': 'GaussianDistribution', 'init_std': .03, 'std_type': 'scalar'}
    actor = RouteActor(td, {'actor': ['policy']}, 'actor', 7, hidden_dims=[8],
                       distribution_cfg=copy.deepcopy(distribution))
    critic = MLPModel(td, {'critic': ['policy']}, 'critic', 1, hidden_dims=[8])
    storage = RolloutStorage('rl', 8, 2, td, [7])
    replay = tmp_path / 'replay.pt'
    torch.save({'observations': observations, 'actions': torch.full((8, 7), 20.),
                'reference_observations': observations + .1, 'reference_actions': torch.full((8, 7), -20.),
                'task': 'loading', 'observation_contract': 'loading_route_v1'}, replay)
    alg = AnchoredPPO(actor, critic, storage, replay_file=replay,
                       observation_contract='loading_route_v1', device='cpu')
    checkpoint = alg.save()
    assert checkpoint['rebar_actor_type'] == 'route_heads_v1'
    alg.load(checkpoint)
    for obs, labels in ((alg.replay.obs, alg.replay.actions),
                        (alg.replay.reference_obs, alg.replay.reference_actions)):
        with torch.no_grad():
            expected = alg.prior(TensorDict({'policy': obs}, batch_size=[len(obs)])).clamp(-1, 1)
        torch.testing.assert_close(labels, expected)
