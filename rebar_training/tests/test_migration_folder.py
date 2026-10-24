import argparse
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import migration_backup as b
import migration_folder as f


@unittest.skipUnless(sys.platform=='darwin','APFS creation is Mac-only; restoration is portable')
class MigrationFolder(unittest.TestCase):
    def test_source_mutation_and_deletion_do_not_change_backup(self):
        with tempfile.TemporaryDirectory(dir=str(Path.home()/'Develop/backups')) as td,contextlib.redirect_stdout(io.StringIO()):
            r=Path(td);src=r/'source';src.mkdir();data=b'original policy fixture'*200
            (src/'one').write_bytes(data);(src/'duplicate').write_bytes(data);(src/'link').symlink_to('one')
            roots=r/'roots.json';roots.write_text(json.dumps({'workspace':str(src)}));out=r/'backup'
            b.scan(argparse.Namespace(output=out,roots=roots));f.clone_objects(out)
            obj=out/'objects'/b.digest(src/'one');self.assertNotEqual(obj.stat().st_ino,(src/'one').stat().st_ino)
            (src/'one').write_bytes(b'new content');(src/'duplicate').unlink();f.verify(out)
            dest=r/'restored';f.restore(out,dest,None)
            self.assertEqual((dest/'workspace/one').read_bytes(),data)
            self.assertEqual((dest/'workspace/duplicate').read_bytes(),data)
            self.assertEqual((dest/'workspace/link').readlink(),Path('one'))
            with self.assertRaises(FileExistsError):f.restore(out,dest,None)

    def test_corruption_is_rejected(self):
        with tempfile.TemporaryDirectory(dir=str(Path.home()/'Develop/backups')) as td,contextlib.redirect_stdout(io.StringIO()):
            r=Path(td);src=r/'source';src.mkdir();(src/'file').write_bytes(b'original')
            roots=r/'roots.json';roots.write_text(json.dumps({'workspace':str(src)}));out=r/'backup'
            b.scan(argparse.Namespace(output=out,roots=roots));f.clone_objects(out)
            next((out/'objects').iterdir()).write_bytes(b'corrupt')
            with self.assertRaises(ValueError):f.verify(out)


if __name__=='__main__':unittest.main()
