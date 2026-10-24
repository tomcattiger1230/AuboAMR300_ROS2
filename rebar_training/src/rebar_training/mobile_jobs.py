"""Launch a simulation-only job from immutable source and persist its receipts.

Run on the Ubuntu training host. No ROS node, robot driver or messaging service
is started. SDK locations can be overridden when moving to another computer.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from .spec import repository_root


def atomic_json(path,value):
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(value,indent=2)+'\n');temporary.replace(path)


def run_job(file):
    job=json.loads(file.read_text());status=file.with_suffix('.status.json')
    common=dict(job=str(file),runner_pid=os.getpid(),source_sha256=job['source_sha256'],log=job['log'])
    try:
        with Path(job['log']).open('x') as stream:
            process=subprocess.Popen(job['command'],cwd=job['cwd'],stdin=subprocess.DEVNULL,
                stdout=stream,stderr=subprocess.STDOUT)
            atomic_json(status,{**common,'state':'running','pid':process.pid,'updated':time.time()})
            code=process.wait()
        atomic_json(status,{**common,'state':'completed' if code==0 else 'failed','exit_code':code,'updated':time.time()})
    except BaseException as error:
        atomic_json(status,{**common,'state':'failed','error':str(error),'updated':time.time()});raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--name')
    parser.add_argument('--artifact-dir',type=Path)
    parser.add_argument('--lab-root',type=Path,default=os.environ.get('ISAACLAB_ROOT'))
    parser.add_argument('--sim-root',type=Path,default=os.environ.get('ISAAC_SIM_ROOT'))
    parser.add_argument('--python',type=Path,default=os.environ.get('REBAR_TRAINING_PYTHON'))
    parser.add_argument('--assets',type=Path)
    parser.add_argument('--status',action='store_true')
    parser.add_argument('mission_args',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    if args.run:return run_job(args.run)
    repo=repository_root();root=args.artifact_dir or repo/'rebar_training/artifacts/mobile_curriculum_20261020'
    root=root.resolve();root.mkdir(parents=True,exist_ok=True)
    if args.status:
        for file in sorted(root.glob('*.job.status.json')):
            result=json.loads(file.read_text());name=file.name.removesuffix('.job.status.json')
            row=dict(name=name,state=result['state'],pid=result.get('pid'),runner_pid=result.get('runner_pid'))
            for suffix in ('.progress.json','.training.json'):
                progress=root/(name+suffix)
                if progress.exists():
                    data=json.loads(progress.read_text())
                    row[suffix]={k:data[k] for k in ('episodes','successes','steps','new_updates','policy_steps') if k in data}
            print(json.dumps(row))
        return
    if not args.name or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]{0,95}',args.name):
        parser.error('Use --name with letters, digits, dash or underscore')
    home=Path.home();lab=args.lab_root or home/'Develop/IsaacLab-rebar'
    sim=args.sim_root or home/'isaacsim';python=args.python or home/'Develop/rebar-training-runtime/bin/python'
    assets=args.assets or repo/'rebar_training/artifacts/assets_mobile_v2'
    for path in (lab,sim,python,assets/'manifest.json'):
        if not path.exists():parser.error('Missing runtime/asset path: '+str(path))
    file=root/(args.name+'.job.json')
    if file.exists() or (root/(args.name+'.json')).exists():parser.error('Use a fresh job name')
    snapshot=root/'source_snapshots'/args.name
    shutil.copytree(Path(__file__).resolve().parent.parent,snapshot,ignore=shutil.ignore_patterns('__pycache__'))
    digest=hashlib.sha256()
    for source in sorted(snapshot.rglob('*.py')):
        digest.update(str(source.relative_to(snapshot)).encode());digest.update(source.read_bytes())
    forwarded=args.mission_args[1:] if args.mission_args[:1]==['--'] else args.mission_args
    if any(value=='--report' or value.startswith('--report=') for value in forwarded):
        parser.error('The job owns its fresh --report path')
    old=repo/'rebar_training/artifacts/visual_control_20261019'
    command=['/usr/bin/env','OMP_NUM_THREADS=4','REBAR_STARTUP_TRACE=1','PYTHONPATH='+str(snapshot),
        'REBAR_REPO_ROOT='+str(repo),'REBAR_TRAINING_ASSETS='+str(assets),
        'ISAACLAB_ROOT='+str(lab),'ISAAC_SIM_ROOT='+str(sim),'REBAR_TRAINING_PYTHON='+str(python),
        'bash',str(repo/'rebar_training/scripts/run_isaac_python.sh'),'-m','rebar_training.mobile_mission_check',
        '--headless','--enable_cameras','--detector',str(old/'segment_v1.pt'),
        '--camera-config',str(repo/'rebar_training/config/vision_runtime.json'),*forwarded,
        '--report',str(root/(args.name+'.json'))]
    if not any(value.startswith('env.sim.render_interval=') for value in forwarded):command+=['env.sim.render_interval=12']
    if shutil.which('systemd-inhibit'):
        command=['systemd-inhibit','--what=idle','--mode=block','--who=rebar-simulation',
                 '--why=Visual manipulation simulation experiment',*command]
    job=dict(command=command,cwd=str(repo),log=str(root/(args.name+'.log')),
        source_snapshot=str(snapshot),source_sha256=digest.hexdigest(),
        created_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
    with file.open('x') as stream:stream.write(json.dumps(job,indent=2)+'\n')
    atomic_json(file.with_suffix('.status.json'),dict(state='queued',job=str(file),updated=time.time()))
    process=subprocess.Popen([sys.executable,'-m','rebar_training.mobile_jobs','--run',str(file)],
        env={**os.environ,'PYTHONPATH':str(snapshot)},cwd=repo,start_new_session=True,
        stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    print(json.dumps(dict(job=str(file),runner_pid=process.pid,source_sha256=digest.hexdigest())))


if __name__=='__main__':main()
