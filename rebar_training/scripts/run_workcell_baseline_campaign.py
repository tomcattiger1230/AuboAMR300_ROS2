#!/usr/bin/env python3
"""Finite six-episode frozen physical campaign. Resume observes, never resubmits.

Run on the authorized simulation node only. Existing job descriptors are owned
by their original runners. No training, hardware service, global power setting,
or automatic failed-case retry is provided.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time


def atomic(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2)+'\n');tmp.replace(path)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--version',type=int,default=4);args=p.parse_args();version='v'+str(args.version)
    if not 4<=args.version<=9:p.error('Unsupported bounded candidate version')
    if socket.gethostname()!='arnold-NUC16JNK9XX11':p.error('Only the authorized simulation node is supported')
    root=args.root.resolve();template=json.loads((root/('baseline_source_A_machine_1_'+version+'.job.json')).read_text())
    source=Path(template['source_snapshot']);sys.path.insert(0,str(source))
    from rebar_training.workcells import WorkcellLayout,FROZEN_POLICY_SHA256
    from rebar_training.workcell_sim_review import review
    layout=WorkcellLayout.load(source/'config/multi_workcell_offline.json')
    manifest=json.loads((root/('candidate_source_'+version+'.manifest.json')).read_text())
    checkpoint=Path(template['command'][template['command'].index('--checkpoint')+1])
    state=dict(schema='workcell_physical_baseline_campaign_v1',manager_pid=os.getpid(),started_unix=time.time(),
        training_started=False,new_learning_updates=0,task_interface_accepted=False,physical_acceptance=False,
        scope='six physical controller probes with surveyed simulation geometry; machine visual identity remains unverified',
        results=[],state='running',current=None)
    status=root/'campaign.status.json'
    # Exclusive ownership: an existing live or dead manager is not replaced silently.
    lock=root/'campaign.lock';fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY);os.write(fd,str(os.getpid()).encode());os.close(fd)
    def persist():state['updated_unix']=time.time();atomic(status,state)
    try:
        for i,s in enumerate(('source_A','source_B')):
            for j,m in enumerate(('machine_1','machine_2','machine_3')):
                name='baseline_'+s+'_'+m+'_'+version;jobfile=root/(name+'.job.json');reportfile=root/(name+'.json')
                state['current']=name;persist()
                for rel,digest in manifest.items():
                    if hashlib.sha256((source/rel).read_bytes()).hexdigest()!=digest:raise RuntimeError('Candidate source changed: '+rel)
                if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=FROZEN_POLICY_SHA256:raise RuntimeError('Frozen policy changed')
                if not jobfile.exists():
                    inventory=subprocess.check_output(['nvidia-smi','--query-compute-apps=pid,process_name,used_memory','--format=csv,noheader'],text=True).strip()
                    if inventory:raise RuntimeError('GPU not idle before new task: '+inventory)
                    command=list(template['command'])
                    for option,value in (('--source',s),('--machine',m),('--seed',str(12501001+(args.version-4)*1000+i*3+j)),('--report',str(reportfile))):
                        command[command.index(option)+1]=value
                    job=dict(template,command=command,log=str(root/(name+'.log')),created_unix=time.time())
                    with jobfile.open('x') as f:f.write(json.dumps(job,indent=2)+'\n')
                    process=subprocess.Popen(['python3',str(root/'bounded_runner.py'),str(jobfile)],start_new_session=True,
                        stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                    state['new_runner_pid']=process.pid;persist()
                # Existing descriptor without a running/completed status is an ambiguous launch, never a retry.
                started=time.monotonic();jobstatus=jobfile.with_suffix('.status.json')
                while True:
                    data=json.loads(jobstatus.read_text()) if jobstatus.exists() else {}
                    if data.get('state') in ('completed','failed','wall_deadline'):break
                    if not data and time.monotonic()-started>15:raise RuntimeError('Unconfirmed existing launch: '+name)
                    if time.monotonic()-started>1900:raise RuntimeError('Job runner did not finalize within deadline: '+name)
                    if data.get('state')=='running':
                        try:os.kill(data['runner_pid'],0)
                        except ProcessLookupError:raise RuntimeError('Job runner disappeared: '+name)
                    state['job_status']=data;persist();time.sleep(5)
                if data.get('state')!='completed' or data.get('exit_code')!=0:raise RuntimeError('Physical job failed: '+name+' '+json.dumps(data))
                if not reportfile.exists():raise RuntimeError('Fresh report absent despite process exit: '+name)
                result=json.loads(reportfile.read_text());checked=review(result,layout)
                checked.update(report_sha256=hashlib.sha256(reportfile.read_bytes()).hexdigest(),report=str(reportfile),reviewed_unix=time.time())
                atomic(root/(name+'.review.json'),checked);state['results'].append(checked);persist()
                if not checked['physical_candidate_passed']:raise RuntimeError('Independent physical review failed: '+name+' '+json.dumps(checked['errors']))
        state['state']='physical_candidate_baseline_complete';state['physical_candidate_passed_count']=len(state['results']);state['current']=None
    except Exception as ex:
        state['state']='needs_analysis';state['error']=type(ex).__name__+': '+str(ex)
    finally:persist()


if __name__=='__main__':main()
