"""Content-addressed portable backup. Standard library only; never starts a job."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import tarfile
import time

SKIP_DIRS={'.git','__pycache__','.pytest_cache','.ruff_cache','.mypy_cache'}
SKIP_NAMES={'.DS_Store'}


def save(path,value):
    pending=Path(str(path)+'.pending')
    pending.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
    pending.replace(path)


def digest(path):
    with open(path,'rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def safe_path(root,name):
    p=PurePosixPath(name)
    if p.is_absolute() or '..' in p.parts or not p.parts:raise ValueError('Unsafe path: '+name)
    target=root.joinpath(*p.parts)
    for parent in target.parents:
        if parent==root:break
        if parent.is_symlink():raise ValueError('Symlink ancestor: '+str(parent))
    return target


def scan(args):
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    target=out/'source_manifest.json'
    if target.exists():raise FileExistsError(target)
    roots=json.loads(args.roots.read_text());entries=[];excluded=[];objects={};total=0;started=time.monotonic();last=started
    for label,folder in roots.items():
        base=Path(folder).resolve()
        if base==out or out.is_relative_to(base):raise ValueError('Backup output cannot be inside a source root')
        for parent,dirs,names in os.walk(base,followlinks=False):
            for name in list(dirs):
                path=Path(parent)/name
                if name in SKIP_DIRS:
                    dirs.remove(name);excluded.append(str(PurePosixPath(label)/path.relative_to(base).as_posix()));continue
                if path.is_symlink():dirs.remove(name);names.append(name)
            rel=Path(parent).relative_to(base)
            entries.append({'path':str(PurePosixPath(label)/rel.as_posix()),'type':'directory','mode':stat.S_IMODE(Path(parent).stat().st_mode)})
            for name in sorted(names):
                path=Path(parent)/name;logical=str(PurePosixPath(label)/path.relative_to(base).as_posix())
                if name in SKIP_NAMES or name.endswith('.pyc'):
                    excluded.append(logical);continue
                st=path.lstat();item={'path':logical,'mode':stat.S_IMODE(st.st_mode),'mtime_ns':st.st_mtime_ns}
                if path.is_symlink():item.update(type='symlink',target=os.readlink(path))
                elif stat.S_ISREG(st.st_mode):
                    sha=digest(path);after=path.stat()
                    if (st.st_size,st.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):raise RuntimeError('Source changed while hashing: '+logical)
                    item.update(type='file',size=st.st_size,sha256=sha,source=str(path));total+=st.st_size
                    obj=objects.setdefault(sha,{'size':st.st_size,'sources':[]});obj['sources'].append(str(path))
                    if obj['size']!=st.st_size:raise RuntimeError('Inconsistent object size')
                else:raise ValueError('Unsupported source type: '+logical)
                entries.append(item)
                if time.monotonic()-last>10:
                    p={'phase':'hashing','entries':len(entries),'source_bytes':total,'unique_objects':len(objects),'unique_bytes':sum(o['size'] for o in objects.values())}
                    save(out/'progress.json',p);print(json.dumps(p),flush=True);last=time.monotonic()
    m={'schema':'rebar_content_addressed_backup_v1','created_unix':time.time(),'source_roots':roots,'entries':entries,'objects':objects,'excluded_regenerable_or_git_metadata':excluded,'git_history':'metadata/repository.bundle','source_bytes':total,'unique_bytes':sum(o['size'] for o in objects.values()),'files':sum(e['type']=='file' for e in entries),'symlinks':sum(e['type']=='symlink' for e in entries),'training_started':False,'simulation_started':False}
    save(target,m)
    print(json.dumps({k:m[k] for k in ['files','symlinks','source_bytes','unique_bytes']}),flush=True)


class CheckedReader:
    def __init__(self,path):self.stream=open(path,'rb');self.hash=hashlib.sha256()
    def read(self,n=-1):
        b=self.stream.read(n);self.hash.update(b);return b
    def close(self):self.stream.close()


def build(args):
    out=args.output.resolve();m=json.loads((out/'source_manifest.json').read_text())
    archive=out/'rebar_migration.tar';pending=out/'rebar_migration.tar.pending'
    if archive.exists() or pending.exists():raise FileExistsError('Preserve prior archive; choose a new backup directory')
    overhead=len(m['entries'])*512+len(m['objects'])*1024+len((out/'source_manifest.json').read_bytes())+1024*1024
    free=shutil.disk_usage(out).free
    if m['unique_bytes']+overhead+8*1024**3>free:raise OSError('Insufficient capacity with 8GiB reserve; copy manifest to a larger destination')
    written=0;last=time.monotonic()
    with tarfile.open(pending,'w',format=tarfile.PAX_FORMAT) as tar:
        tar.add(out/'source_manifest.json',arcname='SOURCE_MANIFEST.json',recursive=False)
        for sha,obj in sorted(m['objects'].items()):
            source=next((Path(s) for s in obj['sources'] if Path(s).is_file() and Path(s).stat().st_size==obj['size']),None)
            if source is None:raise FileNotFoundError(sha)
            info=tarfile.TarInfo('objects/'+sha);info.size=obj['size'];info.mode=0o600
            reader=CheckedReader(source)
            try:tar.addfile(info,reader)
            finally:reader.close()
            if reader.hash.hexdigest()!=sha:raise RuntimeError('Source changed during archive write: '+str(source))
            written+=obj['size']
            if time.monotonic()-last>10:
                p={'phase':'building','unique_bytes_written':written,'unique_bytes_total':m['unique_bytes']};save(out/'progress.json',p);print(json.dumps(p),flush=True);last=time.monotonic()
    # Detect source mutations after inventory, including duplicate paths not chosen as object representatives.
    changed=[]
    for e in m['entries']:
        if e['type']=='file':
            st=Path(e['source']).stat()
            if (st.st_size,st.st_mtime_ns)!=(e['size'],e['mtime_ns']):changed.append(e['path'])
    save(out/'source_stability.json',{'changed_paths':changed,'passed':not changed})
    if changed:raise RuntimeError('Source files changed since inventory; archive remains pending')
    pending.replace(archive);os.chmod(archive,0o600)
    print(json.dumps({'archive':str(archive),'bytes':archive.stat().st_size}),flush=True)


def verify(args):
    archive=args.archive.resolve();seen=set();total=0;last=time.monotonic()
    with tarfile.open(archive,'r|') as tar:
        members=iter(tar);first=next(members)
        if first.name!='SOURCE_MANIFEST.json':raise ValueError('Manifest must be first')
        m=json.load(tar.extractfile(first))
        for member in members:
            if not member.isfile() or not member.name.startswith('objects/'):raise ValueError('Unexpected archive entry')
            sha=member.name.removeprefix('objects/');obj=m['objects'].get(sha)
            if obj is None or sha in seen or member.size!=obj['size']:raise ValueError('Invalid object '+sha)
            stream=tar.extractfile(member);actual=hashlib.file_digest(stream,'sha256').hexdigest()
            if actual!=sha:raise ValueError('Object digest mismatch '+sha)
            seen.add(sha);total+=member.size
            if time.monotonic()-last>10:print(json.dumps({'phase':'verifying','objects':len(seen),'unique_bytes':total}),flush=True);last=time.monotonic()
    if seen!=set(m['objects']):raise ValueError('Missing objects')
    for e in m['entries']:
        safe_path(Path('/unused_restore_root'),e['path'])
        if e['type']=='file' and (e['sha256'] not in seen or e['size']!=m['objects'][e['sha256']]['size']):raise ValueError('Broken file mapping')
    outer=digest(archive)
    proof={'archive':archive.name,'archive_bytes':archive.stat().st_size,'archive_sha256':outer,'source_files':m['files'],'source_bytes':m['source_bytes'],'unique_objects':len(seen),'verified_object_bytes':total,'all_file_mappings_verified':True,'mismatches':[],'verified_unix':time.time(),'simulation_started':False}
    save(archive.with_suffix('.verify.json'),proof);archive.with_suffix('.tar.sha256').write_text(outer+'  '+archive.name+'\n');print(json.dumps(proof),flush=True)


def restore(args):
    dest=args.destination.resolve()
    if dest.exists():raise FileExistsError('Restore only into a NEW directory')
    dest.mkdir(parents=True)
    with tarfile.open(args.archive,'r|') as tar:
        members=iter(tar);first=next(members);assert first.name=='SOURCE_MANIFEST.json';m=json.load(tar.extractfile(first))
        entries=[e for e in m['entries'] if not args.prefix or any(e['path']==p or e['path'].startswith(p.rstrip('/')+'/') for p in args.prefix)]
        by_sha={}
        for e in entries:
            if e['type']=='file':by_sha.setdefault(e['sha256'],[]).append(e)
        restored=set()
        for member in members:
            sha=member.name.removeprefix('objects/')
            if sha not in by_sha:continue
            targets=by_sha[sha];first_path=safe_path(dest,targets[0]['path']);first_path.parent.mkdir(parents=True,exist_ok=True)
            with open(first_path,'xb') as f:shutil.copyfileobj(tar.extractfile(member),f,8*1024*1024)
            if digest(first_path)!=sha:raise ValueError('Restore digest mismatch')
            for i,e in enumerate(targets):
                path=safe_path(dest,e['path']);path.parent.mkdir(parents=True,exist_ok=True)
                if i:shutil.copyfile(first_path,path)
                path.chmod(e['mode']);os.utime(path,ns=(e['mtime_ns'],e['mtime_ns']))
                restored.add(e['path'])
        if len(restored)!=sum(e['type']=='file' for e in entries):raise ValueError('Incomplete restore')
        for e in entries:
            path=safe_path(dest,e['path'])
            if e['type']=='directory':path.mkdir(parents=True,exist_ok=True);path.chmod(e['mode'])
            elif e['type']=='symlink':path.parent.mkdir(parents=True,exist_ok=True);path.symlink_to(e['target'])
        save(dest/'RESTORE_RECEIPT.json',{'archive':str(args.archive.resolve()),'files_restored':len(restored),'selected_prefixes':args.prefix,'all_selected_files_verified':True,'simulation_started':False})
    print(json.dumps({'files_restored':len(restored),'destination':str(dest)}),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('scan');a.add_argument('--roots',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('build');a.add_argument('--output',type=Path,required=True)
    a=sub.add_parser('verify');a.add_argument('--archive',type=Path,required=True)
    a=sub.add_parser('restore');a.add_argument('--archive',type=Path,required=True);a.add_argument('--destination',type=Path,required=True);a.add_argument('--prefix',action='append')
    args=p.parse_args();{'scan':scan,'build':build,'verify':verify,'restore':restore}[args.command](args)


if __name__=='__main__':main()
