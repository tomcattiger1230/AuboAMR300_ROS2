#!/usr/bin/env python3
"""One-shot v5->v6 handoff after the existing native probe naturally exits.

Only the reproduced measured-frame mismatch or an independently successful v5
probe permits the prepared v6 campaign. Unexpected failures stop for analysis.
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


def atomic(path,data):
    t=path.with_suffix('.tmp');t.write_text(json.dumps(data,indent=2)+'\n');t.replace(path)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--previous',type=Path,required=True);args=p.parse_args()
    if socket.gethostname()!='arnold-NUC16JNK9XX11':p.error('Authorized simulation host only')
    root=args.root.resolve();previous=args.previous.resolve()
    if root.name!='workcell_frozen_validation_20261025_v6' or previous.name!='workcell_frozen_validation_20261025_v5' or root.parent!=previous.parent:p.error('Only the declared v5 to v6 handoff is allowed')
    source=root/'source_v6';sys.path.insert(0,str(source))
    from rebar_training.workcells import WorkcellLayout,FROZEN_POLICY_SHA256
    from rebar_training.workcell_sim_review import review
    state=dict(launcher_pid=os.getpid(),state='waiting_for_previous_probe',simulation_started=False,training_started=False,updated_unix=time.time())
    status=root/'prelaunch.status.json';atomic(status,state)
    try:
        deadline=time.monotonic()+2000
        while time.monotonic()<deadline:
            oldjob=json.loads((previous/'baseline_source_A_machine_1_v5.job.status.json').read_text())
            if oldjob['state'] in ('completed','failed','wall_deadline'):break
            time.sleep(5)
        else:raise RuntimeError('Previous native job has not finalized; no new launch')
        reportfile=previous/'baseline_source_A_machine_1_v5.json'
        if not reportfile.exists():raise RuntimeError('Previous native report absent; no new launch')
        report=json.loads(reportfile.read_text());layout=WorkcellLayout.load(source/'config/multi_workcell_offline.json')
        checked=review(report,layout);checked.update(report_sha256=hashlib.sha256(reportfile.read_bytes()).hexdigest(),reviewed_unix=time.time())
        atomic(previous/'baseline_source_A_machine_1_v5.review.json',checked)
        oldstatus=json.loads((previous/'campaign.status.json').read_text());oldstatus.update(state='needs_analysis',current='baseline_source_A_machine_1_v5',results=[checked],
            reason='Dispatcher stopped after recorded measured-SE3 replay exposed a planar target mismatch; this native job was allowed to exit naturally',updated_unix=time.time());atomic(previous/'campaign.status.json',oldstatus)
        if report.get('error')!='ValueError: Station geometry and loading goal disagree' and not checked['physical_candidate_passed']:
            raise RuntimeError('Unexpected previous probe outcome; no v6 launch: '+str(report.get('error')))
        frozen=report.get('frozen_identity_after',{})
        if frozen.get('sha256')!=FROZEN_POLICY_SHA256 or frozen.get('file_and_tensor_payload_unchanged') is not True:raise RuntimeError('Previous frozen proof mismatch')
        manifest=json.loads((root/'candidate_source_v6.manifest.json').read_text())
        for rel,sha in manifest.items():
            if hashlib.sha256((source/rel).read_bytes()).hexdigest()!=sha:raise RuntimeError('Prepared source changed: '+rel)
        inventory=subprocess.check_output(['nvidia-smi','--query-compute-apps=pid,process_name,used_memory','--format=csv,noheader'],text=True,timeout=10).strip()
        if inventory:raise RuntimeError('GPU has an existing compute process; no new launch: '+inventory)
        template=json.loads((previous/'baseline_source_A_machine_1_v5.job.json').read_text());command=[a.replace(str(previous),str(root)).replace('source_v5','source_v6').replace('baseline_source_A_machine_1_v5','baseline_source_A_machine_1_v6') for a in template['command']]
        if any(x in command for x in ('--learn-residual','--save-policy','--start-on-tray','--start-at-dock','--survey-from-packet')) or 'rebar_training.workcell_sim_check' not in command:raise RuntimeError('Unexpected candidate command')
        checkpoint=Path(command[command.index('--checkpoint')+1])
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=FROZEN_POLICY_SHA256:raise RuntimeError('Frozen checkpoint changed')
        command[command.index('--seed')+1]='12503001';file=root/'baseline_source_A_machine_1_v6.job.json'
        if file.exists():raise RuntimeError('Existing v6 descriptor; never resubmit')
        job=dict(template,command=command,source_snapshot=str(source),log=str(root/'baseline_source_A_machine_1_v6.log'),created_unix=time.time())
        with file.open('x') as f:f.write(json.dumps(job,indent=2)+'\n')
        runner=subprocess.Popen(['python3',str(root/'bounded_runner.py'),str(file)],start_new_session=True,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        manager=subprocess.Popen(['python3',str(root/'run_workcell_baseline_campaign.py'),'--root',str(root),'--version','6'],start_new_session=True,
            stdin=subprocess.DEVNULL,stdout=(root/'campaign.stdout.log').open('x'),stderr=(root/'campaign.stderr.log').open('x'))
        state.update(state='launched_once',simulation_started=True,runner_pid=runner.pid,manager_pid=manager.pid,previous_report_sha256=checked['report_sha256'],updated_unix=time.time())
        atomic(root/'campaign.launch.json',state);atomic(status,state)
    except Exception as ex:
        state.update(state='needs_analysis',error=type(ex).__name__+': '+str(ex),updated_unix=time.time());atomic(status,state)
        c=json.loads((root/'campaign.status.json').read_text());c.update(state='needs_analysis',error=state['error'],updated_unix=time.time());atomic(root/'campaign.status.json',c)
        raise


if __name__=='__main__':main()
