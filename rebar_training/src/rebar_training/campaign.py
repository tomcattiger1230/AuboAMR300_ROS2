"""Supervised curriculum campaign: expert -> BC -> DAgger -> PPO -> held-out gates.

A successful expert run or mixed-control training run never satisfies the final
policy gate. Each learned policy must pass both independent seeds by itself.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid


def policy_gate(report, task, seed, episodes, threshold, checkpoint):
    records = report.get('episodes', [])
    if (report.get('mode') != 'checkpoint_evaluation' or not report.get('completed')
            or not report.get('finite') or report.get('seed') != seed or report.get('task') != task
            or (task == 'loading' and report.get('observation_contract') != 'loading_route_v1')
            or len(records) != episodes or report.get('checkpoint') != str(Path(checkpoint).resolve())):
        return False
    successes = sum(bool(r.get('success')) and not r.get('failure', True) and not r.get('timeout', True)
                    and (task != 'loading' or r.get('picked', False)) for r in records)
    return successes / episodes >= threshold


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=('grasp', 'insert', 'loading'), required=True)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--demonstrations', type=Path, required=True)
    parser.add_argument('--template', type=Path, required=True)
    parser.add_argument('--agent-config', type=Path, required=True)
    parser.add_argument('--num-envs', type=int, default=32)
    parser.add_argument('--dagger-steps', type=int, default=6000)
    parser.add_argument('--dagger-attempts', type=int, default=3)
    parser.add_argument('--iterations', type=int, default=1500)
    parser.add_argument('--eval-episodes', type=int, default=100)
    parser.add_argument('--threshold', type=float, default=.95)
    parser.add_argument('--max-rounds', type=int, default=3)
    parser.add_argument('--initial-checkpoint', type=Path, help='Resume a fitted policy; still requires a standalone gate')
    parser.add_argument('--initial-replay', type=Path, help='Matching DAgger replay, retaining expert reference states')
    args = parser.parse_args()
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    if not (0 < args.threshold <= 1) or min(args.num_envs, args.iterations, args.eval_episodes, args.max_rounds) < 1:
        parser.error('positive counts and threshold in (0,1] are required')
    summary = {'task': args.task, 'stable': False, 'rounds': [], 'seeds': [10042, 20042],
               'threshold': args.threshold, 'episodes_per_seed': args.eval_episodes}

    def write():
        temporary = root / 'campaign_report.tmp'
        temporary.write_text(json.dumps(summary, indent=2) + '\n')
        temporary.replace(root / 'campaign_report.json')

    def run(module, arguments, logfile):
        command = [sys.executable, '-m', 'rebar_training.' + module, *map(str, arguments)]
        print('RUN', ' '.join(command), flush=True)
        with (root / logfile).open('w') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
            while process.poll() is None:
                time.sleep(15)
                print({'module': module, 'pid': process.pid, 'log': str(root / logfile)}, flush=True)
        if process.returncode:
            raise RuntimeError(f'{module} failed ({process.returncode}); see {root / logfile}')

    checkpoint = args.initial_checkpoint.resolve() if args.initial_checkpoint else root / 'bc.pt'
    if args.initial_checkpoint:
        if not checkpoint.is_file():
            parser.error('Initial checkpoint does not exist')
    else:
        run('imitation', ['--demonstrations', args.demonstrations.resolve(), '--template', args.template.resolve(),
                         '--agent-config', args.agent_config.resolve(), '--output', checkpoint,
                         '--epochs', 300], 'bc.log')
    for round_index in range(args.max_rounds):
        round_root = root / f'round_{round_index}'
        round_root.mkdir(exist_ok=True)
        replay_file = args.initial_replay.resolve() if args.initial_replay and round_index == 0 else args.demonstrations.resolve()
        # A resume never trusts the filename or an earlier mixed-control rate.
        # Recheck the actual policy before freezing it as the PPO prior.
        initial_gate = round_root / 'initial_gate.json'
        initial_seed = 89042 + round_index
        run('gpu_check', ['--task', args.task, '--checkpoint', checkpoint, '--seed', initial_seed,
                         '--num_envs', args.num_envs, '--episodes', 32, '--headless', '--report', initial_gate],
            f'initial_gate_{round_index}.log')
        ready = policy_gate(json.loads(initial_gate.read_text()), args.task, initial_seed, 32, args.threshold, checkpoint)
        dagger = checkpoint
        for attempt in range(0 if ready else args.dagger_attempts):
            dagger = round_root / f'dagger_{attempt}.pt'
            run('gpu_check', ['--task', args.task, '--dagger', '--checkpoint', checkpoint,
                             '--replay-file', replay_file, '--output-checkpoint', dagger,
                             '--steps', args.dagger_steps, '--num_envs', args.num_envs,
                             '--seed', 43 + 10 * round_index + attempt,
                             '--headless', '--report', round_root / f'dagger_train_{attempt}.json'],
                f'dagger_{round_index}_{attempt}.log')
            gate_seed = 90042 + attempt + 10 * round_index
            gate_report = round_root / f'dagger_gate_{attempt}.json'
            run('gpu_check', ['--task', args.task, '--checkpoint', dagger, '--seed', gate_seed,
                             '--num_envs', args.num_envs, '--episodes', 32, '--headless',
                             '--report', gate_report], f'dagger_gate_{round_index}_{attempt}.log')
            passed = policy_gate(json.loads(gate_report.read_text()), args.task, gate_seed, 32,
                                 args.threshold, dagger)
            checkpoint = dagger
            replay_file = dagger.with_suffix('.replay.pt')
            if passed:
                ready = True
                break
        if not ready:
            raise RuntimeError('DAgger standalone policy gate failed; PPO must not freeze an unverified prior')
        name = f'{args.task}_campaign_{uuid.uuid4().hex[:10]}'
        logs = Path(os.environ['REBAR_REPO_ROOT']) / 'logs/rsl_rl/aubo_rebar'
        warmstart = logs / f'warmstart_{name}'
        warmstart.mkdir(parents=True)
        shutil.copy2(dagger, warmstart / 'model.pt')
        import torch
        architecture = torch.load(dagger, weights_only=True, map_location='cpu')
        actor_overrides = []
        if architecture.get('rebar_actor_type') == 'route_heads_v1':
            from types import SimpleNamespace
            from .route_actor import configure_actor
            cfg = SimpleNamespace(actor=SimpleNamespace())
            configure_actor(cfg, architecture)
            actor_overrides = ['agent.actor.class_name=' + cfg.actor.class_name,
                               'agent.actor.hidden_dims=' + json.dumps(cfg.actor.hidden_dims, separators=(',', ':'))]
        run('launcher', ['train', '--task', args.task, '--num_envs', args.num_envs,
                         '--max_iterations', args.iterations, '--resume', '--load_run', warmstart.name,
                         '--checkpoint', 'model.pt', '--run_name', name, '--headless',
                         'agent.num_steps_per_env=8', 'agent.algorithm.learning_rate=0.00001',
                         'agent.algorithm.schedule=fixed', 'agent.algorithm.entropy_coef=0.0',
                         'agent.algorithm.replay_file=' + str(args.demonstrations.resolve()), *actor_overrides], f'ppo_{round_index}.log')
        folders = [p for p in logs.glob('*_' + name) if not p.name.startswith('warmstart_')]
        if len(folders) != 1:
            raise RuntimeError('PPO run folder missing or ambiguous; native Kit exit status is insufficient')
        # Native RSL-RL resumes its iteration counter; --max_iterations is the
        # number of additional updates, not the final checkpoint index.
        final_iteration = int(architecture.get('iter', 0)) + args.iterations - 1
        checkpoint = folders[0] / f'model_{final_iteration}.pt'
        if not checkpoint.is_file():
            raise RuntimeError('PPO did not write the final requested checkpoint')
        record = {'round': round_index, 'checkpoint': str(checkpoint),
                  'sha256': hashlib.sha256(checkpoint.read_bytes()).hexdigest(), 'evaluations': []}
        for seed in summary['seeds']:
            report = round_root / f'eval_{seed}.json'
            run('gpu_check', ['--task', args.task, '--checkpoint', checkpoint, '--seed', seed,
                             '--num_envs', args.num_envs, '--episodes', args.eval_episodes, '--headless',
                             '--report', report], f'eval_{round_index}_{seed}.log')
            data = json.loads(report.read_text())
            passed = policy_gate(data, args.task, seed, args.eval_episodes, args.threshold, checkpoint)
            record['evaluations'].append({'seed': seed, 'passed': passed, 'report': str(report),
                                          'success_rate': data.get('success_rate')})
        summary['rounds'].append(record)
        summary['stable'] = all(item['passed'] for item in record['evaluations'])
        write()
        if summary['stable']:
            print(json.dumps(summary, indent=2), flush=True)
            return
    raise RuntimeError('Campaign has not met the held-out stability gate; inspect failure episodes before more compute')


if __name__ == '__main__':
    main()
