"""Stage-conditioned neural policy with no teacher or command-history input.

All seven joint/gripper commands come from learned heads. The measured route
stage selects a head; it never supplies an action or changes physics criteria.
"""
import copy
import torch
from torch import nn
from rsl_rl.models import MLPModel
from rsl_rl.modules import MLP


def actor_constructor_config(config, metadata):
    """Lab config contains deprecated aliases that RSL 5 cannot accept."""
    raw=copy.deepcopy(metadata.get('rebar_actor_constructor',config))
    distribution=raw.get('distribution_cfg')
    if distribution is not None and 'class_name' not in distribution:
        # Early visual-adaptation receipts saved a dictionary after RSL's
        # constructor popped this key. Migrate only the known Gaussian shape.
        if ('distribution.std_param' not in metadata.get('actor_state_dict',{})
                or set(distribution)-{'init_std','std_type'}):
            raise ValueError('Checkpoint distribution constructor is incomplete')
        distribution['class_name']='GaussianDistribution'
    result={key:raw[key] for key in ('activation','obs_normalization','distribution_cfg')}
    result['hidden_dims']=metadata['rebar_actor_hidden_dims']
    return result


class RouteHeads(nn.Module):
    def __init__(self, inputs, outputs, hidden_dims, activation):
        super().__init__()
        self.outputs = outputs
        self.heads = nn.ModuleList(MLP(inputs, outputs, hidden_dims, activation) for _ in range(11))

    def forward(self, latent):
        features, gates = latent[:, :-11], latent[:, -11:]
        # Empty heads still participate in autograd, keeping PPO parameter
        # groups consistent across batches that visit different route stages.
        result = features.new_zeros((features.shape[0], self.outputs))
        for i, head in enumerate(self.heads):
            result = result + head(features) * gates[:, i:i + 1]
        return result


class RouteActor(MLPModel):
    def __init__(self, obs, obs_groups, obs_set, output_dim, **kwargs):
        super().__init__(obs, obs_groups, obs_set, output_dim, **copy.deepcopy(kwargs))
        if self.obs_dim != 71:
            raise ValueError('RouteActor requires the 71-channel loading_route_v1 observation')
        outputs = self.distribution.input_dim if self.distribution else output_dim
        self.mlp = RouteHeads(56, outputs, kwargs.get('hidden_dims', [128, 128, 64]),
                              kwargs.get('activation', 'elu'))
        self.architecture_metadata = {'rebar_actor_type': 'route_heads_v1',
                                      'rebar_actor_hidden_dims': kwargs.get('hidden_dims', [128, 128, 64])}

    def get_latent(self, obs, masks=None, hidden_state=None):
        raw = torch.cat([obs[group] for group in self.obs_groups], dim=-1)
        normalized = self.obs_normalizer(raw)
        # The previous action and integrated joint target are consequences of
        # earlier commands, and let BC learn a shortcut instead of feedback.
        features = normalized[:, :56]
        stage = (raw[:, 53] * 10).round().long().clamp(0, 10)
        gates = torch.nn.functional.one_hot(stage, 11).to(features.dtype)
        return torch.cat((features, gates), dim=-1)

    def as_jit(self):
        return RouteExport(self)

    def as_onnx(self, verbose=False):
        return RouteOnnxExport(self, verbose)


class RouteExport(nn.Module):
    """Raw observation -> learned action, including stage and mask preprocessing."""
    def __init__(self, actor):
        super().__init__()
        self.obs_normalizer = copy.deepcopy(actor.obs_normalizer)
        self.mlp = copy.deepcopy(actor.mlp)
        self.deterministic_output = (actor.distribution.as_deterministic_output_module()
                                     if actor.distribution else nn.Identity())

    def forward(self, raw: torch.Tensor) -> torch.Tensor:
        features = self.obs_normalizer(raw)[:, :56]
        stage = (raw[:, 53] * 10).round().long().clamp(0, 10)
        gates = torch.nn.functional.one_hot(stage, 11).to(features.dtype)
        return self.deterministic_output(self.mlp(torch.cat((features, gates), dim=-1)))

    @torch.jit.export
    def reset(self) -> None:
        pass


class RouteOnnxExport(RouteExport):
    is_recurrent = False

    def __init__(self, actor, verbose):
        super().__init__(actor)
        self.verbose = verbose

    def get_dummy_inputs(self):
        return (torch.zeros(1, 71),)

    @property
    def input_names(self):
        return ['obs']

    @property
    def output_names(self):
        return ['actions']


def configure_actor(agent_cfg, metadata):
    """Use an explicit, local architecture contract when loading a checkpoint."""
    kind = metadata.get('rebar_actor_type', 'mlp')
    if kind == 'route_heads_v1':
        if metadata.get('rebar_observation_contract') not in ('loading_route_v1','loading_visual_v1'):
            raise ValueError('Route actor requires a known state/visual loading observation contract')
        agent_cfg.actor.class_name = 'rebar_training.route_actor:RouteActor'
        agent_cfg.actor.hidden_dims = metadata['rebar_actor_hidden_dims']
    elif kind != 'mlp':
        raise ValueError(f'Unknown checkpoint actor type: {kind}')
