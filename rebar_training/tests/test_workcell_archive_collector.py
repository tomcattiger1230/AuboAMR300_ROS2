"""Verify the detached archive protocol locally, without SSH or a node process."""
import ast
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import socket
import tarfile
import pytest


def test_detached_stream_verification_and_failure_receipt(tmp_path,monkeypatch):
    path=Path(__file__).parents[1]/'scripts/follow_workcell_campaign_0103.py'
    spec=importlib.util.spec_from_file_location('workcell_collector_test',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    tree=ast.parse(path.read_text());assignment=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='verification_code' for t in n.targets))
    def exercise(corrupt):
        folder=tmp_path/str(corrupt);folder.mkdir();archive=folder/'payload.tar.gz';body=b'actual evidence bytes'
        meta=dict(files={'sensor.json':dict(sha256=hashlib.sha256(b'wrong' if corrupt else body).hexdigest(),bytes=len(body))},remote_proof={'campaign':{'state':'needs_analysis'}})
        with tarfile.open(archive,'w:gz') as stream:
            for name,data in (('evidence/sensor.json',body),('manifest.json',json.dumps(meta).encode())):
                info=tarfile.TarInfo(name);info.size=len(data);stream.addfile(info,io.BytesIO(data))
        sha=hashlib.sha256(archive.read_bytes()).hexdigest();dest='/mnt/zspace/rebar_migration_20261024_142019/supplements/workcell_results_'+sha[:12]+'.tar.gz'
        code=eval(compile(ast.Expression(assignment.value),str(path),'eval'),module.__dict__,dict(dest=dest,sha=sha))
        status=folder/'status.json';sidecar=archive.with_suffix('.both_hosts_verified.json')
        def rpc(source):
            if 'script.write_text' in source:
                parsed=ast.parse(source);call=next(n for n in ast.walk(parsed) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='write_text' and isinstance(n.func.value,ast.Name) and n.func.value.id=='script')
                worker=ast.literal_eval(call.args[0]);worker=worker.replace(module.REMOTE_REPO+'/rebar_training/artifacts/workcell_archive_verifiers/'+sha+'/status.json',str(status)).replace(str(Path(dest).with_suffix('.both_hosts_verified.json')),str(sidecar)).replace(dest,str(archive))
                try:exec(compile(worker,'<local-detached-worker>','exec'),{})
                except AssertionError:pass  # A detached child records its own failure.
                return {'state':'started'}
            return json.loads(status.read_text())
        monkeypatch.setattr(module,'remote',rpc);monkeypatch.setattr(socket,'gethostname',lambda:'arnold-NUC16JNK9XX11')
        if corrupt:
            with pytest.raises(RuntimeError,match='Detached archive verification failed'):module.verify_archive_detached(code,dest,sha)
            assert json.loads(status.read_text())['state']=='failed'
        else:
            result=module.verify_archive_detached(code,dest,sha);assert result['all_files_verified'] is True and result['files']==1
    exercise(False);exercise(True)
