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


class MigrationBackup(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.source=self.root/'source';self.source.mkdir();self.out=self.root/'backup'
        (self.source/'one').write_bytes(b'actual data'*500)
        (self.source/'duplicate').write_bytes((self.source/'one').read_bytes())
        (self.source/'one').chmod(0o755);(self.source/'link').symlink_to('one');(self.source/'empty').mkdir()
        roots=self.root/'roots.json';roots.write_text(json.dumps({'workspace':str(self.source)}))
        self.args=argparse.Namespace(output=self.out,roots=roots)
        self.capture=contextlib.redirect_stdout(io.StringIO());self.capture.__enter__();self.addCleanup(self.capture.__exit__,None,None,None)
        b.scan(self.args)

    def test_full_roundtrip_preserves_bytes_links_empty_directory_and_mode(self):
        b.build(self.args);archive=self.out/'rebar_migration.tar';b.verify(argparse.Namespace(archive=archive))
        dest=self.root/'restore';b.restore(argparse.Namespace(archive=archive,destination=dest,prefix=None))
        self.assertEqual((dest/'workspace/one').read_bytes(),(self.source/'one').read_bytes())
        self.assertEqual((dest/'workspace/link').readlink(),Path('one'))
        self.assertTrue((dest/'workspace/empty').is_dir());self.assertTrue((dest/'workspace/one').stat().st_mode&0o111)
        self.assertEqual(len(json.loads((self.out/'source_manifest.json').read_text())['objects']),1)

    def test_changed_source_cannot_become_completed_archive(self):
        (self.source/'one').write_bytes(b'changed')
        with self.assertRaises((RuntimeError,FileNotFoundError)):b.build(self.args)
        self.assertFalse((self.out/'rebar_migration.tar').exists())

    def test_corrupted_object_is_detected(self):
        b.build(self.args);archive=self.out/'rebar_migration.tar';data=archive.read_bytes()
        archive.write_bytes(data.replace(b'actual data',b'broken data',1))
        with self.assertRaises(ValueError):b.verify(argparse.Namespace(archive=archive))

    def test_restore_refuses_existing_destination_and_path_escape(self):
        with self.assertRaises(FileExistsError):b.restore(argparse.Namespace(archive=Path('unused'),destination=self.source,prefix=None))
        with self.assertRaises(ValueError):b.safe_path(self.root,'../outside')


if __name__=='__main__':unittest.main()
