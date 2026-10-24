#!/usr/bin/env python3
"""Finite Mac collector for an already launched campaign; never starts jobs.

Reads node status, then backs up finalized raw evidence with per-file SHA. No
simulator/learner imports, launch commands, notifications or hardware endpoints.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tarfile
import time

REMOTE_REPO='/home/arnold/Develop/rebar-training-resume-0103-20261022'
RELATIVE='rebar_training/artifacts/workcell_frozen_validation_20261025'
SSH=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2','arnold@192.168.0.103']
SSH_COMMAND='ssh -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=2'


def atomic(path,data):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(path)


def bounded(command,deadline=40,input=None):
    p=subprocess.Popen(command,stdin=subprocess.PIPE if input else subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
    try:out,err=p.communicate(input,timeout=deadline)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM)
        try:p.communicate(timeout=3)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.communicate()
        raise RuntimeError('Bounded collector transport timeout')
    if p.returncode:raise RuntimeError('Collector transport failed: '+err[-2000:])
    return out


def remote(code):return json.loads(bounded(SSH+['python3 -'],40,code))


def verify_archive_detached(verification_code,destination,expected_sha):
    """Keep slow NAS verification outside a bounded SSH request, then poll SSD status."""
    import re
    if not re.fullmatch(r'[0-9a-f]{64}',expected_sha):raise ValueError('Invalid archive SHA')
    if not destination.startswith('/mnt/zspace/rebar_migration_20261024_142019/supplements/'):raise ValueError('Unexpected archive destination')
    work=REMOTE_REPO+'/rebar_training/artifacts/workcell_archive_verifiers/'+expected_sha
    home_status=work+'/status.json'
    # Successful prior checks are recovered by full content-addressed archive identity.
    sidecar=str(Path(destination).with_suffix('.both_hosts_verified.json'))
    worker=("from pathlib import Path\nimport json,time,traceback\ns=Path("+repr(home_status)+")\n"
        +"try:\n p=Path("+repr(sidecar)+"); old=json.loads(p.read_text()) if p.exists() else {}\n"
        +" if old.get('sha256')=="+repr(expected_sha)+" and old.get('all_files_verified') is True:\n  result=old\n else:\n"
        +"  exec(compile("+repr(verification_code)+",'<archive-verification>','exec'))\n  result=json.loads(Path("+repr(sidecar)+").read_text())\n"
        +" s.write_text(json.dumps(dict(state='complete',proof=result,updated_unix=time.time()))+'\\n')\n"
        +"except BaseException as e:\n s.write_text(json.dumps(dict(state='failed',error=type(e).__name__+': '+str(e),updated_unix=time.time()))+'\\n');raise\n")
    starter=("from pathlib import Path\nimport subprocess,json,os,time,socket\nassert socket.gethostname()=='arnold-NUC16JNK9XX11'\n"
        +"r=Path("+repr(work)+");r.mkdir(parents=True,exist_ok=True);s=r/'status.json';receipt=r/'launch.json'\n"
        +"if not receipt.exists():\n script=r/'verify.py';script.write_text("+repr(worker)+")\n"
        +" with receipt.open('x') as f:f.write(json.dumps(dict(state='launching',created_unix=time.time())))\n"
        +" log=(r/'worker.log').open('a');p=subprocess.Popen(['python3',str(script)],stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)\n"
        +" receipt.write_text(json.dumps(dict(pid=p.pid,created_unix=time.time())))\n"
        +"print(json.dumps(dict(state='started_or_existing',status=str(s))))\n")
    remote(starter)
    for _ in range(180):
        reply=remote("from pathlib import Path\nimport json\np=Path("+repr(home_status)+");print(p.read_text() if p.exists() else json.dumps(dict(state='running')))\n")
        if reply['state']=='complete':
            proof=reply['proof']
            if proof.get('sha256')!=expected_sha or proof.get('all_files_verified') is not True:raise RuntimeError('Detached verification identity mismatch')
            return proof
        if reply['state']=='failed':raise RuntimeError('Detached archive verification failed: '+reply.get('error','unknown'))
        time.sleep(10)
    raise RuntimeError('Detached archive verification pending after bounded polling')


def main():
    global RELATIVE
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--repo',type=Path,required=True);p.add_argument('--max-hours',type=float,default=6);p.add_argument('--artifact-name',default='workcell_frozen_validation_20261025');args=p.parse_args()
    import re
    if not re.fullmatch(r'workcell_frozen_validation_20261025(?:_v[4-9])?',args.artifact_name):p.error('Unsupported campaign artifact name')
    RELATIVE='rebar_training/artifacts/'+args.artifact_name
    if not 0<args.max_hours<=6:p.error('Collector must be bounded to at most six hours')
    root=args.repo.resolve()/RELATIVE;status=root/'mac_followup.status.json';started=time.monotonic()
    lock=root/'mac_followup.lock';fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY);os.write(fd,str(os.getpid()).encode());os.close(fd)
    state=dict(pid=os.getpid(),phase='following_existing_campaign',training_started=False,simulation_launch_allowed=False,started_unix=time.time())
    def persist():state['updated_unix']=time.time();atomic(status,state)
    header="from pathlib import Path\nimport json,hashlib,socket,time\nassert socket.gethostname()=='arnold-NUC16JNK9XX11'\nr=Path("+repr(REMOTE_REPO+'/'+RELATIVE)+")\n"
    try:
        while time.monotonic()-started<args.max_hours*3600:
            try:
                data=remote(header+"c=json.loads((r/'campaign.status.json').read_text());n=c.get('current');p=r/(n+'.status.json') if n else None;print(json.dumps(dict(campaign=c,active=json.loads(p.read_text()) if p and p.exists() else None,captured_unix=time.time())))\n")
                atomic(root/'campaign.remote_snapshot.json',data);atomic(root/'campaign.status.json',data['campaign'])
                state.update(remote_live_state_fresh=True,campaign_state=data['campaign']['state'],current=data['campaign'].get('current'),results=len(data['campaign'].get('results',[])));persist()
                if data['campaign']['state'] in ('needs_analysis','physical_candidate_baseline_complete'):break
            except Exception as ex:state.update(remote_live_state_fresh=False,last_transport_error=str(ex));persist()
            time.sleep(30)
        else:raise RuntimeError('Finite follow-up deadline reached; pending node evidence retained')
        state['phase']='collecting_finalized_evidence';persist()
        manifest=remote(header+'''files={}
for p in sorted(r.rglob('*')):
 if not p.is_file() or '__pycache__' in p.parts or p.name=='campaign.final_manifest.json' or p.suffix=='.tmp':continue
 before=p.stat();h=hashlib.sha256(p.read_bytes()).hexdigest();after=p.stat();assert (before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns)
 files[str(p.relative_to(r))]=dict(sha256=h,bytes=after.st_size)
policy=Path('''+repr(REMOTE_REPO+"/rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/frozen_policy.pt")+''')
sha=hashlib.sha256(policy.read_bytes()).hexdigest();assert sha=='393b47f8385a9163e3c9f88b53375923327aa3c80239a3af105e700cf1f93a82'
m=dict(schema='workcell_final_raw_manifest_v1',files=files,campaign=json.loads((r/'campaign.status.json').read_text()),policy_optimizer_sha256=sha,created_unix=time.time())
(r/'campaign.final_manifest.json').write_text(json.dumps(m,indent=2)+'\\n');print(json.dumps(m))
''')
        for attempt in range(90):
            try:
                bounded(['rsync','-rl','--partial','--exclude=__pycache__/','-e',SSH_COMMAND,'arnold@192.168.0.103:'+REMOTE_REPO+'/'+RELATIVE+'/',str(root)+'/'],55);break
            except Exception as ex:state.update(transfer_attempt=attempt+1,last_transport_error=str(ex));persist();time.sleep(2)
        else:raise RuntimeError('Final raw transfer pending after bounded attempts')
        mismatches=[rel for rel,d in manifest['files'].items() if not (root/rel).is_file() or hashlib.sha256((root/rel).read_bytes()).hexdigest()!=d['sha256']]
        if mismatches:raise RuntimeError('Final raw SHA mismatch: '+repr(mismatches))
        proof=dict(all_files_verified=True,files=len(manifest['files']),campaign=manifest['campaign'],policy_optimizer_sha256=manifest['policy_optimizer_sha256'],verified_unix=time.time(),new_layout_task_acceptance=False)
        atomic(root/'campaign.raw_both_hosts_verified.json',proof)
        # Freeze a self-contained supplement before writing any subsequent collector status.
        state['phase']='archiving_finalized_evidence';persist()
        backup=Path('/Users/weiluo/Develop/backups')/('rebar_workcell_results_20261025_'+str(int(time.time())));backup.mkdir()
        inventory={str(q.relative_to(root)):dict(sha256=hashlib.sha256(q.read_bytes()).hexdigest(),bytes=q.stat().st_size) for q in sorted(root.rglob('*')) if q.is_file() and q.name not in ('mac_followup.stdout.log','mac_followup.stderr.log','mac_followup.lock')}
        archive=backup/'workcell_results.tar.gz'
        with tarfile.open(archive,'w:gz') as t:
            for rel in inventory:t.add(root/rel,arcname='evidence/'+rel)
            import io
            body=json.dumps(dict(files=inventory,remote_proof=proof),indent=2).encode();info=tarfile.TarInfo('manifest.json');info.size=len(body);t.addfile(info,io.BytesIO(body))
        with tarfile.open(archive,'r:gz') as t:
            for rel,rec in inventory.items():
                data=t.extractfile('evidence/'+rel).read();assert len(data)==rec['bytes'] and hashlib.sha256(data).hexdigest()==rec['sha256'],rel
        sha=hashlib.sha256(archive.read_bytes()).hexdigest();dest='/mnt/zspace/rebar_migration_20261024_142019/supplements/workcell_results_'+sha[:12]+'.tar.gz'
        archive_proof=dict(archive=str(archive),sha256=sha,bytes=archive.stat().st_size,files=len(inventory),all_files_verified=True,remote=dest)
        atomic(backup/'LOCAL_ARCHIVE_VERIFIED.json',archive_proof)
        for attempt in range(90):
            try:bounded(['rsync','-l','--append','--partial','-e',SSH_COMMAND,str(archive),'arnold@192.168.0.103:'+dest],55);break
            except Exception as ex:state.update(nas_transfer_attempt=attempt+1,last_transport_error=str(ex));persist();time.sleep(2)
        else:raise RuntimeError('NAS archive transfer pending')
        verification_code=("from pathlib import Path\nimport json,hashlib,tarfile,time,socket\nassert socket.gethostname()=='arnold-NUC16JNK9XX11'\np=Path("+repr(dest)+")\n"
            +"digest=hashlib.sha256()\nwith p.open('rb') as handle:\n for block in iter(lambda:handle.read(8*1024*1024),b''):digest.update(block)\nsha=digest.hexdigest();assert sha=="+repr(sha)+"\n"+'''records={};metadata=None
with tarfile.open(p,'r|gz',bufsize=8*1024*1024) as stream:
 for member in stream:
  if not member.isfile():continue
  handle=stream.extractfile(member)
  if member.name=='manifest.json':metadata=json.load(handle);continue
  assert member.name.startswith('evidence/'),member.name
  digest=hashlib.sha256();size=0
  for block in iter(lambda:handle.read(8*1024*1024),b''):digest.update(block);size+=len(block)
  records[member.name[len('evidence/'):]]=dict(sha256=digest.hexdigest(),bytes=size)
assert metadata is not None and records==metadata['files'],'Archive file inventory mismatch'
proof=dict(archive=str(p),sha256=sha,files=len(records),all_files_verified=True,verified_unix=time.time(),campaign_state=metadata['remote_proof']['campaign']['state'],new_layout_task_acceptance=False,verification='streamed complete archive plus every file SHA')
p.with_suffix('.both_hosts_verified.json').write_text(json.dumps(proof,indent=2)+'\\n');print(json.dumps(proof))
''')
        verified=verify_archive_detached(verification_code,dest,sha)
        atomic(backup/'BOTH_HOSTS_VERIFIED.json',verified);atomic(root/'campaign.final_backup.json',dict(local=archive_proof,nas=verified))
        state.update(phase='complete',backup=str(backup),all_raw_files_both_hosts_verified=True,nas_archive_verified=True);persist()
    except Exception as ex:state.update(phase='sync_pending',metadata_sync_pending=True,error=type(ex).__name__+': '+str(ex));persist();raise


if __name__=='__main__':main()
