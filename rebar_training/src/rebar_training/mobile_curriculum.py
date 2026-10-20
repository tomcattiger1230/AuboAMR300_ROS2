"""One finite, gated residual-PPO curriculum. Stops for analysis on failed evidence.

The plan and source are frozen before the first update. This is a simulation
experiment process, not an app automation, and does not launch ROS/hardware.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from .mobile_acceptance import acceptance
from .mobile_jobs import atomic_json
from .spec import repository_root


def validate_plan(plan):
    if plan.get('schema_version')!='mobile_residual_curriculum_v1':raise ValueError('Unknown curriculum schema')
    from .randomization import profile
    profile(plan['randomization'])
    if type(plan.get('evaluation_parallel_jobs',1)) is not int or not 1<=plan.get('evaluation_parallel_jobs',1)<=3:
        raise ValueError('Evaluation concurrency must be between one and three simulation jobs')
    if not 0<plan['residual_scale']<=.15:raise ValueError('Invalid residual scale')
    if [v['name'] for v in plan['phases']]!=['tray','dock_clamp','full']:
        raise ValueError('Curriculum must gate tray, dock clamp and full mission in order')
    seen=set()
    for phase in plan['phases']:
        seeds=[phase['train_seed'],*phase['evaluation_seeds']]
        if len(seeds)!=len(set(seeds)) or seen.intersection(seeds):raise ValueError('Training/evaluation seeds overlap')
        seen.update(seeds)
        if phase['train_episodes']<1 or phase['minimum_updates']<1:raise ValueError('Empty training stage')
        if len(phase['evaluation_seeds'])<2:raise ValueError('Two independent evaluation seeds required')
        count=phase['evaluation_episodes_per_seed']*len(phase['evaluation_seeds'])
        if count<(64 if phase['name']=='full' else 8):raise ValueError('Insufficient prespecified evaluation episodes')
        if phase['name']=='full' and (phase['start_on_tray'] or phase['start_at_dock'] or phase.get('visual_hint')):
            raise ValueError('Full mission must start before pickup, without a recorded hint')


def evaluate_group(function,requests,parallel):
    # Leaving the context waits for every dispatched job, also on exceptions.
    # No orphan evaluations are left behind while the next phase starts.
    with ThreadPoolExecutor(max_workers=parallel) as pool:
        futures=[pool.submit(function,*request) for request in requests]
        return [future.result() for future in futures]


def require_successful_training(report):
    rows=report.get('results',[])
    if (not report.get('completed') or not rows or len(rows)!=report.get('episodes')
            or report.get('successes')!=len(rows)
            or any(not row.get('success') or row.get('failure') for row in rows)):
        raise RuntimeError('Training episode failed or incomplete; analyze before independent evaluation')
    if not any(update.get('active_steps',0)>0 for update in report.get('ppo_updates',[])):
        raise RuntimeError('No new active actor update; analyze before independent evaluation')


def run(root,plan,snapshot,initial_checkpoint=None):
    repo=repository_root();state=dict(state='running',phase=None,jobs=[],gates=[],
        started_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        standalone_full_mission_policy=False,training_algorithm='bounded arm residual PPO over camera-only servo')
    status=root/'curriculum.status.json';atomic_json(status,state)
    env={**os.environ,'PYTHONPATH':str(snapshot),'REBAR_REPO_ROOT':str(repo)}
    checkpoint=initial_checkpoint;state['active_jobs']=[];state_lock=threading.Lock()

    def job(name,arguments):
        output=subprocess.check_output([sys.executable,'-m','rebar_training.mobile_jobs',
            '--artifact-dir',str(root),'--name',name,'--',*arguments],env=env,text=True)
        launched=json.loads(output.strip().splitlines()[-1])
        with state_lock:
            state['jobs'].append(launched);state['active_jobs'].append(name)
            state['current_job']=name;atomic_json(status,state)
        try:
            file=root/(name+'.job.status.json')
            while True:
                result=json.loads(file.read_text())
                if result['state'] not in ('queued','running'):break
                time.sleep(10)
            if result['state']!='completed':raise RuntimeError('Simulation job failed: '+name)
            report=json.loads((root/(name+'.json')).read_text())
            if not report.get('completed'):raise RuntimeError('Missing fresh complete simulation receipts: '+name)
            return report
        finally:
            with state_lock:
                state['active_jobs'].remove(name);atomic_json(status,state)

    try:
        for phase in plan['phases']:
            state['phase']=phase['name'];atomic_json(status,state)
            common=['--until',phase['until'],'--randomization',plan['randomization'],
                    '--residual-scale',str(plan['residual_scale'])]
            if phase['start_on_tray']:common+=['--start-on-tray']
            if phase['start_at_dock']:common+=['--start-at-dock']
            if phase.get('initial_joints'):common+=['--initial-joints',*[str(x) for x in phase['initial_joints']]]
            if phase.get('visual_hint'):common+=['--survey-from-packet',str(repo/phase['visual_hint'])]
            output=root/(phase['name']+'_policy.pt')
            args=[*common,'--learn-residual','--seed',str(phase['train_seed']),
                  '--episodes',str(phase['train_episodes']),'--rl-iterations',str(phase['minimum_updates']),
                  '--save-policy',str(output)]
            if checkpoint:args+=['--residual-checkpoint',str(checkpoint)]
            training=job(phase['name']+'_train',args)
            require_successful_training(training)
            checkpoint=output;requests=[]
            for seed in phase['evaluation_seeds']:
                requests.append((phase['name']+'_holdout_'+str(seed),[*common,'--seed',str(seed),
                    '--episodes',str(phase['evaluation_episodes_per_seed']),'--residual-checkpoint',str(checkpoint)]))
            reports=evaluate_group(job,requests,plan.get('evaluation_parallel_jobs',1))
            gate=acceptance(reports,phase['name'],plan['randomization'])
            atomic_json(root/(phase['name']+'_gate.json'),gate);state['gates'].append(gate);atomic_json(status,state)
            if not gate['passed']:raise RuntimeError('Independent '+phase['name']+' gate failed; no automatic promotion')
        if plan.get('record_completion_video',False):
            state['phase']='full_demo';atomic_json(status,state)
            demo=job('full_demo',[*common,'--seed',str(phase['evaluation_seeds'][0]),'--episodes','1',
                '--residual-checkpoint',str(checkpoint),'--regression','--video',str(root/'full_policy_demo.mp4')])
            if demo.get('successes')!=1:raise RuntimeError('Post-gate demonstration failed; analyze before delivery')
        state.update(state='completed',stable_within_declared_scope=True,checkpoint=str(checkpoint),
                     finished_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
        atomic_json(status,state)
    except BaseException as error:
        state.update(state='needs_analysis',stable_within_declared_scope=False,error=str(error),
                     checkpoint=str(checkpoint) if checkpoint else None)
        atomic_json(status,state);raise


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,default=repository_root()/'rebar_training/config/mobile_curriculum.json')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--initial-checkpoint',type=Path,
                   help='Copy a trusted residual checkpoint, then rerun every phase and independent gate')
    p.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    args=p.parse_args();plan=json.loads(args.plan.read_text());validate_plan(plan)
    if args.worker:return run(args.output,plan,Path(__file__).resolve().parent.parent,args.initial_checkpoint)
    if args.initial_checkpoint and not args.initial_checkpoint.is_file():p.error('Initial checkpoint is missing')
    if args.output.exists():p.error('Use a fresh experiment directory')
    args.output.mkdir(parents=True);root=args.output.resolve()
    frozen=root/'plan.json';atomic_json(frozen,plan)
    checkpoint=None
    if args.initial_checkpoint:
        checkpoint=root/'initial_policy.pt';shutil.copyfile(args.initial_checkpoint,checkpoint)
    snapshot=root/'campaign_source'
    shutil.copytree(Path(__file__).resolve().parent.parent,snapshot,ignore=shutil.ignore_patterns('__pycache__'))
    digest=hashlib.sha256()
    for source in sorted(snapshot.rglob('*.py')):
        digest.update(str(source.relative_to(snapshot)).encode());digest.update(source.read_bytes())
    atomic_json(root/'provenance.json',dict(source_sha256=digest.hexdigest(),
        plan_sha256=hashlib.sha256(frozen.read_bytes()).hexdigest(),plan=str(frozen),source_snapshot=str(snapshot),
        initial_checkpoint=str(checkpoint) if checkpoint else None,
        initial_checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest() if checkpoint else None))
    command=[sys.executable,'-m','rebar_training.mobile_curriculum',
             '--worker','--plan',str(frozen),'--output',str(root)]
    if checkpoint:command+=['--initial-checkpoint',str(checkpoint)]
    if shutil.which('systemd-inhibit'):
        # Cover the gaps between simulator jobs too. Child-only inhibition
        # would let an already-idle workstation suspend during gate checks.
        command=['systemd-inhibit','--what=idle','--mode=block','--who=rebar-curriculum',
                 '--why=Finite visual residual-PPO curriculum',*command]
    with (root/'curriculum.log').open('x') as log:
        process=subprocess.Popen(command,
            env={**os.environ,'PYTHONPATH':str(snapshot),'REBAR_REPO_ROOT':str(repository_root())},
            start_new_session=True,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
    print(json.dumps(dict(root=str(root),runner_pid=process.pid,source_sha256=digest.hexdigest())))


if __name__=='__main__':main()
