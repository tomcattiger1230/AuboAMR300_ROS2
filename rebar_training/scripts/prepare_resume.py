#!/usr/bin/env python3
"""Prepare fresh validation inputs after a move. Never starts training or SSH."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def source_sha(folder):
    result=hashlib.sha256()
    for p in sorted(folder.rglob('*.py')):
        result.update(str(p.relative_to(folder)).encode());result.update(p.read_bytes())
    return result.hexdigest()


def prepare(root,template,output):
    root,template,output=map(lambda p:Path(p).resolve(),(root,template,output))
    if output.exists():raise FileExistsError('Use a fresh directory; preserve interrupted evidence')
    pre=json.loads((template/'predeclaration.json').read_text())
    if source_sha(root/'rebar_training/src')!=pre['source_sha256']:
        raise ValueError('Canonical source differs from the frozen validation source')
    if source_sha(template/'validation_source')!=pre['source_sha256']:
        raise ValueError('Frozen source differs from its declaration')
    expected={'validate_precision.py':pre['controller_sha256'],'plan_v8.json':pre['plan_sha256'],
              'initial_policy.pt':pre['warm_start_sha256'],'failed_policy.pt':pre['failed_policy_sha256']}
    for name,digest in expected.items():
        if hashlib.sha256((template/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Validation input changed: '+name)
    output.mkdir(parents=True)
    for name in [*expected,'predeclaration.json']:shutil.copy2(template/name,output/name)
    shutil.copytree(template/'validation_source',output/'validation_source',
                    ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    receipt=dict(template=str(template),output=str(output),source_sha256=pre['source_sha256'],
                 preserved_seed_declaration=True,training_started=False,requires_explicit_resume=True,
                 excluded=['old CPU-test log','launcher and process IDs','job JSON with old absolute paths','native reports and job status'])
    (output/'PREPARED_ONLY.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    p.add_argument('--template',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(json.dumps(prepare(a.root,a.template,a.output),indent=2))


if __name__=='__main__':main()
