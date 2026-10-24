"""Portable content-addressed folder; APFS clone creation, ordinary portable restoration."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import tarfile
import time
from migration_backup import save,digest,safe_path,CheckedReader


def clone_objects(root):
    m=json.loads((root/'source_manifest.json').read_text());store=root/'objects';store.mkdir(exist_ok=False)
    lib=ctypes.CDLL('/usr/lib/libSystem.B.dylib',use_errno=True);clone=lib.clonefile
    clone.argtypes=[ctypes.c_char_p,ctypes.c_char_p,ctypes.c_int];clone.restype=ctypes.c_int
    count=0;total=0;last=time.monotonic()
    for sha,obj in sorted(m['objects'].items()):
        source=next((Path(s) for s in obj['sources'] if Path(s).is_file() and Path(s).stat().st_size==obj['size']),None)
        if source is None:raise FileNotFoundError(sha)
        target=store/sha
        if clone(bytes(source),bytes(target),0):raise OSError(ctypes.get_errno(),'APFS clone failed',str(source))
        if (source.stat().st_dev,source.stat().st_ino)==(target.stat().st_dev,target.stat().st_ino):raise ValueError('Hardlink is forbidden')
        target.chmod(0o600);count+=1;total+=obj['size']
        if time.monotonic()-last>10:
            p={'phase':'cloning','objects':count,'logical_unique_bytes':total};save(root/'progress.json',p);print(json.dumps(p),flush=True);last=time.monotonic()
    changed=[]
    for e in m['entries']:
        if e['type']=='file':
            s=Path(e['source']).stat()
            if (s.st_size,s.st_mtime_ns)!=(e['size'],e['mtime_ns']):changed.append(e['path'])
    save(root/'source_stability.json',{'passed':not changed,'changed_paths':changed,'data_objects_are_distinct_inodes':True})
    if changed:raise ValueError('Sources changed since scan; no completion claim')
    print(json.dumps({'cloned_objects':count,'logical_unique_bytes':total,'hardlinks':False}),flush=True)


def verify(root):
    m=json.loads((root/'source_manifest.json').read_text());store=root/'objects';actual=set(p.name for p in store.iterdir())
    if actual!=set(m['objects']):raise ValueError('Object set mismatch')
    count=0;total=0;last=time.monotonic()
    for sha,obj in sorted(m['objects'].items()):
        path=store/sha
        if path.is_symlink() or not path.is_file() or path.stat().st_size!=obj['size'] or digest(path)!=sha:raise ValueError('Object mismatch: '+sha)
        count+=1;total+=obj['size']
        if time.monotonic()-last>10:print(json.dumps({'phase':'verifying','objects':count,'unique_bytes':total}),flush=True);last=time.monotonic()
    for e in m['entries']:
        safe_path(Path('/unused_restore_root'),e['path'])
        if e['type']=='file' and (e['sha256'] not in actual or e['size']!=m['objects'][e['sha256']]['size']):raise ValueError('Invalid mapping')
    proof={'format':'content_addressed_folder_v1','manifest_sha256':digest(root/'source_manifest.json'),'source_files':m['files'],'source_bytes':m['source_bytes'],'verified_unique_objects':count,'verified_unique_bytes':total,'all_file_mappings_verified':True,'mismatches':[],'verified_unix':time.time(),'no_original_source_required_to_restore':True,'shared_volume_local_backup':True,'simulation_started':False}
    save(root/'backup.verify.json',proof);print(json.dumps(proof),flush=True)


def restore(root,dest,prefixes):
    if dest.exists():raise FileExistsError('Destination must be a NEW directory')
    m=json.loads((root/'source_manifest.json').read_text());dest.mkdir(parents=True)
    entries=[e for e in m['entries'] if not prefixes or any(e['path']==p or e['path'].startswith(p.rstrip('/')+'/') for p in prefixes)]
    count=0
    for e in entries:
        if e['type']!='file':continue
        source=root/'objects'/e['sha256'];target=safe_path(dest,e['path']);target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source,target)
        if digest(target)!=e['sha256']:raise ValueError('Restore SHA mismatch: '+e['path'])
        target.chmod(e['mode']);os.utime(target,ns=(e['mtime_ns'],e['mtime_ns']));count+=1
    for e in entries:
        target=safe_path(dest,e['path'])
        if e['type']=='directory':target.mkdir(parents=True,exist_ok=True);target.chmod(e['mode'])
        elif e['type']=='symlink':target.parent.mkdir(parents=True,exist_ok=True);target.symlink_to(e['target'])
    save(dest/'RESTORE_RECEIPT.json',{'source_backup':str(root),'selected_prefixes':prefixes,'files_restored':count,'all_selected_files_verified':True,'simulation_started':False})
    print(json.dumps({'files_restored':count,'destination':str(dest)}),flush=True)


def export_tar(root,target):
    m=json.loads((root/'source_manifest.json').read_text());pending=Path(str(target)+'.pending')
    if target.exists() or pending.exists():raise FileExistsError(target)
    if shutil.disk_usage(target.parent).free<m['unique_bytes']+8*1024**3:raise OSError('Target needs unique_bytes plus 8GiB reserve')
    with tarfile.open(pending,'w',format=tarfile.PAX_FORMAT) as tar:
        tar.add(root/'source_manifest.json',arcname='SOURCE_MANIFEST.json',recursive=False)
        for sha,obj in sorted(m['objects'].items()):
            reader=CheckedReader(root/'objects'/sha);info=tarfile.TarInfo('objects/'+sha);info.size=obj['size'];info.mode=0o600
            try:tar.addfile(info,reader)
            finally:reader.close()
            if reader.hash.hexdigest()!=sha:raise ValueError('Object mismatch')
    pending.replace(target);target.chmod(0o600)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['create','verify','restore','export-tar']);p.add_argument('--backup',type=Path,required=True);p.add_argument('--destination',type=Path);p.add_argument('--prefix',action='append');a=p.parse_args();root=a.backup.resolve()
    if a.command=='create':clone_objects(root)
    elif a.command=='verify':verify(root)
    elif a.command=='restore':
        if a.destination is None:p.error('--destination required')
        restore(root,a.destination.resolve(),a.prefix)
    else:
        if a.destination is None:p.error('--destination required')
        export_tar(root,a.destination.resolve())


if __name__=='__main__':main()
