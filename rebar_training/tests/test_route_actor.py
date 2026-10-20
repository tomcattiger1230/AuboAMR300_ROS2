import copy
from types import SimpleNamespace

import pytest
import torch

pytest.importorskip('rsl_rl')
from tensordict import TensorDict
from rebar_training.route_actor import RouteActor, configure_actor,actor_constructor_config


def test_lab_deprecated_aliases_cannot_reach_rsl5_model_constructor():
    raw=dict(class_name='MLPModel',hidden_dims=[256,128,64],activation='elu',obs_normalization=True,
        distribution_cfg={'class_name':'GaussianDistribution','init_std':.03},
        stochastic='???',init_noise_std='???',noise_std_type='scalar',state_dependent_std=False)
    config=actor_constructor_config(raw,{'rebar_actor_hidden_dims':[16,8]})
    td=TensorDict({'policy':torch.zeros(2,71)},batch_size=[2])
    actor=RouteActor(td,{'actor':['policy']},'actor',7,**config).eval()
    assert actor(td).shape==(2,7)
    assert config['hidden_dims']==[16,8] and 'state_dependent_std' not in config
    assert config['distribution_cfg']['class_name']=='GaussianDistribution'
    old=copy.deepcopy(config);old['distribution_cfg'].pop('class_name')
    metadata=dict(rebar_actor_constructor=old,rebar_actor_hidden_dims=[16,8],actor_state_dict=actor.state_dict())
    migrated=actor_constructor_config(raw,metadata)
    rebuilt=RouteActor(td,{'actor':['policy']},'actor',7,**migrated).eval()
    rebuilt.load_state_dict(actor.state_dict())
    torch.testing.assert_close(rebuilt(td),actor(td))


def test_route_actor_uses_state_feedback_and_ignores_command_history():
    torch.manual_seed(42)
    obs = torch.randn(8, 71)
    obs[:, 53] = .4
    actor = RouteActor(TensorDict({'policy': obs}, batch_size=[8]), {'actor': ['policy']},
                       'actor', 7, hidden_dims=[16, 8], obs_normalization=True)
    actor.update_normalization(TensorDict({'policy': obs}, batch_size=[8]))
    actor.eval()
    baseline = actor({'policy': obs})
    altered = obs.clone()
    altered[:, 56:] += 30
    torch.testing.assert_close(actor({'policy': altered}), baseline)
    altered[:, 37:40] += .01
    assert not torch.allclose(actor({'policy': altered}), baseline)
    baseline.square().mean().backward()
    assert any(p.grad.abs().sum() > 0 for p in actor.mlp.heads[4].parameters())
    assert all(p.grad.abs().sum() == 0 for p in actor.mlp.heads[1].parameters())


def test_export_preserves_stage_selection_normalization_and_history_mask(tmp_path):
    obs = torch.randn(22, 71)
    obs[:, 53] = torch.arange(11).repeat(2) / 10
    td = TensorDict({'policy': obs}, batch_size=[22])
    actor = RouteActor(td, {'actor': ['policy']}, 'actor', 7, hidden_dims=[16, 8],
                       obs_normalization=True,
                       distribution_cfg={'class_name': 'GaussianDistribution', 'init_std': .03})
    actor.update_normalization(td)
    actor.eval()
    compiled = torch.jit.script(actor.as_jit().eval())
    path = tmp_path / 'policy.pt'
    compiled.save(str(path))
    loaded = torch.jit.load(str(path))
    torch.testing.assert_close(loaded(obs), actor(td))
    obs[:, 56:] += 100
    torch.testing.assert_close(loaded(obs), actor({'policy': obs}))
    loaded.reset()
    exported = actor.as_onnx()
    torch.testing.assert_close(exported(obs), actor({'policy': obs}))


def test_route_actor_checkpoint_selects_exact_architecture():
    cfg = SimpleNamespace(actor=SimpleNamespace(class_name='MLPModel', hidden_dims=[256, 128, 64]))
    configure_actor(cfg, {})
    assert cfg.actor.class_name == 'MLPModel'
    metadata = {'rebar_actor_type': 'route_heads_v1', 'rebar_actor_hidden_dims': [128, 128, 64],
                'rebar_observation_contract': 'loading_route_v1'}
    configure_actor(cfg, metadata)
    assert cfg.actor.class_name == 'rebar_training.route_actor:RouteActor'
    assert cfg.actor.hidden_dims == [128, 128, 64]
    invalid = copy.deepcopy(metadata)
    invalid['rebar_observation_contract'] = 'state_v1'
    with pytest.raises(ValueError):
        configure_actor(cfg, invalid)
