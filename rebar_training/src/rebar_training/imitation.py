"""Behavioral-cloning warm start for the pinned RSL-RL actor.

Runs without Kit; expert demonstrations and policy evaluation are distinct. The
output remains an ordinary RSL-RL checkpoint for subsequent PPO fine tuning.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import torch


def stage_weights(observations):
    """Give every observed route stage equal total weight, including rare ones."""
    stages = (observations[:, 53] * 10).round().long().clamp(0, 10)
    counts = torch.bincount(stages, minlength=11).clamp_min(1)
    return counts[stages].float().reciprocal()


def training_observations(observations, loading):
    if not loading:
        return observations
    # The teacher uses measured state and the route, not the previous action.
    # Perturb this non-causal history to prevent an autoregressive BC shortcut.
    result = observations.clone()
    result[:, 56:63] = (result[:, 56:63] + .08 * torch.randn_like(result[:, 56:63])).clamp(-1, 1)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demonstrations', type=Path, required=True)
    parser.add_argument('--template', type=Path, required=True)
    parser.add_argument('--agent-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=300)
    parser.add_argument('--batch-size', type=int, default=4096)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--actor-type', choices=('mlp', 'route_heads_v1'), default='mlp')
    parser.add_argument('--warm-start-actor', action='store_true', help='Adapt the template actor with its normalization frozen')
    parser.add_argument('--allow-state-transfer', action='store_true', help='Explicitly initialize a visual actor from loading_route_v1')
    parser.add_argument('--replay', type=Path, help='Additional physically visited states labeled by the teacher')
    args = parser.parse_args()
    if args.allow_state_transfer and not args.warm_start_actor:
        parser.error('--allow-state-transfer requires --warm-start-actor')
    if args.output.exists():
        parser.error('Use a fresh output checkpoint path')
    from rsl_rl.models import MLPModel
    from tensordict import TensorDict
    import yaml

    torch.manual_seed(args.seed)
    data = torch.load(args.demonstrations, weights_only=True, map_location=args.device)
    if data['success_rate'] < .95:
        raise RuntimeError('Expert demonstrations require at least 95% completed-episode success')
    if data["task"] == "loading" and data.get("observation_contract") not in ("loading_route_v1", "loading_visual_v1"):
        raise RuntimeError("Loading demonstrations require the observed route contract")
    observations, actions = data['observations'], data['actions']
    loading = data['task'] == 'loading'
    if args.replay:
        replay_data = torch.load(args.replay, weights_only=True, map_location=args.device)
        if replay_data.get('observation_contract') != data.get('observation_contract') or replay_data.get('task') != data['task']:
            raise RuntimeError('Additional replay contract/task mismatch')
        observations = torch.cat((observations, replay_data['observations']))
        actions = torch.cat((actions, replay_data['actions']))
    if not torch.isfinite(observations).all() or not torch.isfinite(actions).all():
        raise RuntimeError('Demonstrations contain non-finite data')
    config = yaml.safe_load(args.agent_config.read_text())
    actor_config = copy.deepcopy(config['actor'])
    actor_config.pop('class_name')
    actor_class = MLPModel
    if args.actor_type == 'route_heads_v1':
        if not loading:
            raise RuntimeError('Route heads are only supported for loading')
        from .route_actor import RouteActor
        actor_class = RouteActor
        actor_config['hidden_dims'] = [128, 128, 64]
    actor = actor_class(TensorDict({'policy': observations[:1]}, batch_size=[1]), {'actor': ['policy']},
                     'actor', actions.shape[-1], **copy.deepcopy(actor_config)).to(args.device)
    if args.warm_start_actor:
        template = torch.load(args.template, weights_only=True, map_location=args.device)
        allowed_transfer = (args.allow_state_transfer and data.get('observation_contract') == 'loading_visual_v1'
                            and template.get('rebar_observation_contract') == 'loading_route_v1')
        if (template.get('rebar_actor_type', 'mlp') != args.actor_type
                or (template.get('rebar_observation_contract', 'state_v1') != data.get('observation_contract', 'state_v1')
                    and not allowed_transfer)):
            raise ValueError('Warm-start actor architecture/observation contract differs from demonstrations')
        actor.load_state_dict(template['actor_state_dict'])
    else:
        actor.update_normalization(TensorDict({'policy': observations}, batch_size=[len(observations)]))
    actor.eval()  # fixed normalization during supervised fitting
    optimizer = torch.optim.Adam(actor.mlp.parameters(), lr=3e-4)
    split = torch.randperm(len(observations), device=args.device)
    validation = split[:max(1, len(split) // 20)]
    training = split[len(validation):]
    sampling = stage_weights(observations[training]) if loading else None
    validation_weights = stage_weights(observations[validation]) if loading else None
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, args.epochs, eta_min=3e-5) if loading else None
    history = []
    best_loss, best_state = float('inf'), None
    for epoch in range(args.epochs):
        permutation = training[torch.multinomial(sampling, len(training), replacement=True)] if loading else training[torch.randperm(len(training), device=args.device)]
        for ids in permutation.split(args.batch_size):
            predicted = actor(TensorDict({'policy': training_observations(observations[ids], loading)}, batch_size=[len(ids)]))
            loss = (predicted - actions[ids]).square().mean()
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(actor.parameters(), 1.)
            optimizer.step()
        with torch.no_grad():
            predicted = actor(TensorDict({'policy': observations[validation]}, batch_size=[len(validation)]))
            errors = (predicted - actions[validation]).square().mean(-1)
            overall = float(errors.mean())
            value = float((errors * validation_weights).sum() / validation_weights.sum()) if loading else overall
        if value < best_loss:
            best_loss, best_state = value, copy.deepcopy(actor.state_dict())
        if epoch % 20 == 0 or epoch == args.epochs - 1:
            history.append({'epoch': epoch, 'validation_mse': value, 'overall_validation_mse': overall})
            print(history[-1], flush=True)
        if scheduler:
            scheduler.step()
    actor.load_state_dict(best_state)
    with torch.no_grad():
        actor.distribution.std_param.fill_(.03)
    checkpoint = torch.load(args.template, weights_only=True, map_location='cpu')
    source_contract = checkpoint.get('rebar_observation_contract')
    checkpoint['actor_state_dict'] = {k: v.cpu() for k, v in actor.state_dict().items()}
    # A new supervised fit must not resume imitation momentum or the prior from
    # a different fixture. Subsequent PPO starts its prior from this new actor.
    checkpoint.pop('imitation_prior_state_dict', None)
    checkpoint.pop('imitation_optimizer_state_dict', None)
    checkpoint['optimizer_state_dict']['state'] = {}
    if args.actor_type == 'route_heads_v1':
        # Adam has no momentum after a supervised warm start. Rebuild its
        # parameter IDs for the new actor plus the unchanged template critic.
        groups = checkpoint['optimizer_state_dict']['param_groups']
        if len(groups) != 1:
            raise RuntimeError('Expected the pinned PPO single Adam parameter group')
        critic_count = sum(k.startswith('mlp.') for k in checkpoint['critic_state_dict'])
        groups[0]['params'] = list(range(len(list(actor.parameters())) + critic_count))
    checkpoint['rebar_actor_type'] = args.actor_type
    checkpoint['rebar_actor_hidden_dims'] = actor_config['hidden_dims']
    checkpoint['rebar_actor_constructor'] = actor_config
    checkpoint['iter'] = 0
    checkpoint['rebar_observation_contract'] = data.get('observation_contract', 'state_v1')
    checkpoint['infos'] = {'source': 'behavioral_cloning', 'demonstrations': str(args.demonstrations),
                           'validation_mse': best_loss, 'seed': args.seed,
                           'stage_balanced': loading, 'history_augmentation': loading}
    checkpoint['infos']['warm_start_actor'] = args.warm_start_actor
    checkpoint['infos']['explicit_state_to_visual_transfer'] = args.allow_state_transfer
    provenance={'template_sha256':hashlib.sha256(args.template.read_bytes()).hexdigest(),
                'demonstrations_sha256':hashlib.sha256(args.demonstrations.read_bytes()).hexdigest(),
                'replay_sha256':hashlib.sha256(args.replay.read_bytes()).hexdigest() if args.replay else None,
                'demonstration_source':data.get('source'),
                'template_observation_contract':source_contract,
                'validation_split':'random visited-state fit check; not independent episode evaluation'}
    checkpoint['infos']['provenance']=provenance
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, args.output)
    report = {'task': data['task'], 'samples': len(observations), 'epochs': args.epochs,
              'warm_start_actor': args.warm_start_actor,
              'validation_mse': best_loss, 'history': history, 'policy_success_rate': None,
              'validation_metric': 'equal_route_stage_mse' if loading else 'sample_mean_mse',
              'note': 'Supervised fit only; independent closed-loop physics evaluation required'}
    report['provenance']=provenance
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()


class OnlineImitation:
    """DAgger replay of labels on states actually visited by the student."""
    def __init__(self, actor, path, device, batch_size=2048, observation_contract="state_v1"):
        data = torch.load(path, weights_only=True, map_location=device)
        if data.get("observation_contract", "state_v1") != observation_contract:
            raise RuntimeError("Replay observation contract does not match this task")
        self.obs, self.actions = data['observations'], data['actions']
        self.loading = data.get('task') == 'loading'
        self.reference_obs = data.get('reference_observations', self.obs)
        self.reference_actions = data.get('reference_actions', self.actions)
        self.reference_sampling = stage_weights(self.reference_obs) if self.loading else None
        self.actor, self.batch_size = actor, batch_size
        self.optimizer = torch.optim.Adam(actor.mlp.parameters(), lr=3e-5 if self.loading else 1e-4)
        self.pending_obs, self.pending_actions = [], []
        self.loss = None

    def add(self, observation, actions):
        self.pending_obs.append(observation.detach().clone())
        self.pending_actions.append(actions.detach().clone())

    def update(self, updates=24):
        from tensordict import TensorDict
        self.obs = torch.cat((self.obs, *self.pending_obs))[-300000:]
        self.actions = torch.cat((self.actions, *self.pending_actions))[-300000:]
        self.pending_obs.clear()
        self.pending_actions.clear()
        sampling = stage_weights(self.obs) if self.loading else None
        with torch.enable_grad():
            for _ in range(updates):
                if self.loading:
                    count = self.batch_size // 2
                    anchor = torch.multinomial(self.reference_sampling, count, replacement=True)
                    ids = torch.multinomial(sampling, self.batch_size - count, replacement=True)
                    obs = torch.cat((self.reference_obs[anchor], self.obs[ids]))
                    actions = torch.cat((self.reference_actions[anchor], self.actions[ids]))
                else:
                    ids = torch.randint(len(self.obs), (self.batch_size,), device=self.obs.device)
                    obs, actions = self.obs[ids], self.actions[ids]
                predicted = self.actor(TensorDict({'policy': training_observations(obs, self.loading)}, batch_size=[len(obs)]))
                loss = (predicted - actions).square().mean()
                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.actor.mlp.parameters(), 1.)
                self.optimizer.step()
            self.loss = float(loss.detach())
        return self.loss
