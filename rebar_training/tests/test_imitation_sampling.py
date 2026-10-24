import pytest
import torch

from rebar_training.imitation import OnlineImitation, stage_weights, training_observations


def test_rare_route_stages_receive_equal_training_mass():
    observations = torch.zeros(1030, 71)
    observations[10:30, 53] = .1
    observations[30:, 53] = .4
    weights = stage_weights(observations)
    # Approach has 10 states, descent 20 and carry 1000. Duration must not
    # determine their relative importance in the contact-learning objective.
    masses = torch.stack((weights[:10].sum(), weights[10:30].sum(), weights[30:].sum()))
    torch.testing.assert_close(masses, torch.ones(3))


def test_history_augmentation_preserves_physical_state_and_original_data():
    observations = torch.zeros(64, 71)
    original = observations.clone()
    augmented = training_observations(observations, loading=True)
    torch.testing.assert_close(observations, original)
    torch.testing.assert_close(augmented[:, :56], original[:, :56])
    torch.testing.assert_close(augmented[:, 63:], original[:, 63:])
    assert augmented[:, 56:63].abs().max() <= 1
    assert augmented[:, 56:63].abs().sum() > 0
    assert training_observations(observations, loading=False) is observations


def test_replay_capacity_does_not_evict_reference_expert_states(tmp_path):
    pytest.importorskip('tensordict')
    initial = torch.zeros(2, 71)
    path = tmp_path / 'demonstrations.pt'
    torch.save({'task': 'loading', 'observation_contract': 'loading_route_v1',
                'observations': initial, 'actions': torch.zeros(2, 7)}, path)
    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.mlp = torch.nn.Linear(71, 7)

        def forward(self, observations):
            return self.mlp(observations['policy'])

    actor = Actor()
    replay = OnlineImitation(actor, path, 'cpu', batch_size=4, observation_contract='loading_route_v1')
    replay.obs = torch.ones(300001, 71)
    replay.actions = torch.ones(300001, 7)
    replay.add(torch.ones(1, 71), torch.ones(1, 7))
    replay.update(updates=1)
    assert len(replay.obs) == 300000
    torch.testing.assert_close(replay.reference_obs, initial)
    assert len(replay.reference_actions) == 2
    assert torch.isfinite(torch.tensor(replay.loss))
