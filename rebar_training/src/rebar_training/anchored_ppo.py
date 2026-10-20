"""PPO with optional frozen learned-prior replay to retain contact skills.

The actor is deployed alone. The replay dataset and frozen learned prior are training-only.
"""
import copy
import torch
from tensordict import TensorDict
from rsl_rl.algorithms import PPO
from .imitation import OnlineImitation


class AnchoredPPO(PPO):
    def __init__(self, *args, replay_file=None, replay_updates=8, observation_contract="state_v1", **kwargs):
        super().__init__(*args, **kwargs)
        self.observation_contract = observation_contract
        self.replay = OnlineImitation(self.actor, replay_file, self.device, observation_contract=observation_contract) if replay_file else None
        self.replay_updates = replay_updates
        self.prior = copy.deepcopy(self.actor).eval().requires_grad_(False) if self.replay else None

    def act(self, obs):
        if self.replay:
            with torch.no_grad():
                labels = self.prior(obs).clamp(-1, 1)
            self.replay.add(obs['policy'], labels)
        return super().act(obs)

    def update(self):
        losses = super().update()
        if self.replay:
            losses['imitation_replay'] = self.replay.update(self.replay_updates)
            with torch.no_grad():
                self.actor.distribution.std_param.clamp_(.01, .05)
        return losses

    def save(self):
        saved = super().save()
        saved["rebar_observation_contract"] = self.observation_contract
        saved.update(getattr(self.actor, 'architecture_metadata', {}))
        if self.replay:
            saved['imitation_optimizer_state_dict'] = self.replay.optimizer.state_dict()
            saved['imitation_prior_state_dict'] = self.prior.state_dict()
        return saved

    def load(self, loaded_dict, load_cfg=None, strict=True):
        if loaded_dict.get("rebar_observation_contract", "state_v1") != self.observation_contract:
            raise RuntimeError("Checkpoint observation contract does not match this task")
        result = super().load(loaded_dict, load_cfg, strict)
        if self.replay and 'imitation_optimizer_state_dict' in loaded_dict:
            self.replay.optimizer.load_state_dict(loaded_dict['imitation_optimizer_state_dict'])
        if self.replay:
            self.prior.load_state_dict(loaded_dict.get('imitation_prior_state_dict', loaded_dict['actor_state_dict']))
            # Keep the verified policy's recovery behavior, rather than pulling
            # it back toward the original teacher-forced BC trajectories.
            with torch.no_grad():
                for observations, attribute in ((self.replay.obs, 'actions'),
                                                (self.replay.reference_obs, 'reference_actions')):
                    labels = [self.prior(TensorDict({'policy': batch}, batch_size=[len(batch)])).clamp(-1, 1)
                              for batch in observations.split(4096)]
                    setattr(self.replay, attribute, torch.cat(labels))
        return result
