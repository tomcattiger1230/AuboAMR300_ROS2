"""Control-plane tests with fake native jobs; these claim no physical success."""
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from rebar_training import workcells,workcell_sim_review


@pytest.mark.parametrize('first_pass,existing,expected_launches',[(False,False,1),(True,False,5),(True,True,4)])
def test_followup_gates_failed_retest_and_never_resubmits_existing(tmp_path,monkeypatch,first_pass,existing,expected_launches):
    script=Path(__file__).parents[1]/'scripts/run_workcell_navigation_followup.py'
    spec=importlib.util.spec_from_file_location('navigation_followup_test',script);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    source=tmp_path/'source';(source/'config').mkdir(parents=True);(source/'rebar_training').mkdir()
    config=Path(__file__).parents[1]/'config/multi_workcell_offline.json'
    (source/'config/multi_workcell_offline.json').write_bytes(config.read_bytes())
    env=source/'rebar_training/workcell_sim_env.py';env.write_text('immutable fake candidate')
    digest=hashlib.sha256(env.read_bytes()).hexdigest();checkpoint=tmp_path/'checkpoint';checkpoint.write_bytes(b'unchanged')
    monkeypatch.setattr(workcells,'FROZEN_POLICY_SHA256',hashlib.sha256(checkpoint.read_bytes()).hexdigest())
    monkeypatch.setattr(workcell_sim_review,'review',lambda r,l:dict(physical_candidate_passed=r['pass'],errors=[] if r['pass'] else ['native failure']))
    prior=tmp_path/'prior.json';prior.write_text(json.dumps({'pass':True}))
    tasks=[dict(source=s,machine=m,seed=12504002+i,name='task_'+str(i)) for i,(s,m) in enumerate([
        ('source_A','machine_2'),('source_A','machine_3'),('source_B','machine_1'),('source_B','machine_2'),('source_B','machine_3')])]
    command=['rebar_training.workcell_sim_check','--checkpoint',str(checkpoint),'--source','x','--machine','y','--seed','0','--report','z']
    plan=dict(template=dict(command=command,source_snapshot=str(source)),tasks=tasks,
        previous_coverage=dict(report=str(prior),sha256=hashlib.sha256(prior.read_bytes()).hexdigest()))
    (tmp_path/'followup.plan.json').write_text(json.dumps(plan));(tmp_path/'candidate_source_v8.manifest.json').write_text(json.dumps({'rebar_training/workcell_sim_env.py':digest}))
    launches=[]
    def finish(jobfile):
        jobfile=Path(jobfile);c=json.loads(jobfile.read_text())['command'];value=lambda flag:c[c.index(flag)+1]
        report=dict(seed=int(value('--seed')),source_id=value('--source'),machine_id=value('--machine'),candidate_env_sha256=digest,
            **{'pass':first_pass or int(value('--seed'))!=12504002})
        Path(value('--report')).write_text(json.dumps(report));jobfile.with_suffix('.status.json').write_text(json.dumps(dict(state='completed',exit_code=0)))
    def popen(args,**kwargs):
        launches.append(args);finish(args[-1]);return SimpleNamespace(pid=12345)
    if existing:
        c=command.copy()
        for flag,value in (('--source','source_A'),('--machine','machine_2'),('--seed','12504002'),('--report',str(tmp_path/'task_0.json'))):c[c.index(flag)+1]=value
        jobfile=tmp_path/'task_0.job.json';jobfile.write_text(json.dumps(dict(command=c)));finish(jobfile)
    monkeypatch.setattr(module.socket,'gethostname',lambda:'arnold-NUC16JNK9XX11')
    monkeypatch.setattr(module.subprocess,'check_output',lambda *a,**k:'')
    monkeypatch.setattr(module.subprocess,'Popen',popen);monkeypatch.setattr(module.sys,'argv',[str(script),'--root',str(tmp_path)])
    module.main();state=json.loads((tmp_path/'campaign.status.json').read_text())
    assert len(launches)==expected_launches
    assert state['state']==('physical_candidate_baseline_complete' if first_pass else 'needs_analysis')
    assert len(state['results'])==(5 if first_pass else 1)
    assert not any('source_A_machine_1' in str(x) for x in launches)
    assert not state['training_started'] and state['new_learning_updates']==0
    assert checkpoint.read_bytes()==b'unchanged'
    # The exclusive lock also refuses a second manager after natural completion.
    with pytest.raises(FileExistsError):module.main()
    assert len(launches)==expected_launches
