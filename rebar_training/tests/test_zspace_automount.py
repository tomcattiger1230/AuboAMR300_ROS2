"""Pure fstab/credential-format tests. No root, mounts, credentials or services."""
import importlib.util
from pathlib import Path
import unittest

_spec = importlib.util.spec_from_file_location('zspace_automount_installer', Path(__file__).parents[1]/'scripts/configure_zspace_automount.py')
installer = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(installer)


class ZspaceAutomountPlanTests(unittest.TestCase):
    def test_idempotent_and_preserves_other_entries(self):
        line = installer.fstab_line('//host/share', {})
        original = '# existing\nUUID=abc / ext4 defaults 0 1\n'
        once = installer.updated_fstab(original, line)
        self.assertTrue(once.startswith(original))
        self.assertEqual(installer.updated_fstab(once, line), once)
        self.assertEqual(once.count('/mnt/zspace cifs'), 1)

    def test_conflicting_share_is_rejected(self):
        with self.assertRaises(ValueError):
            installer.updated_fstab('//other/share /mnt/zspace cifs defaults 0 0\n', installer.fstab_line('//host/share', {}))

    def test_duplicate_target_is_rejected(self):
        with self.assertRaises(ValueError):
            installer.updated_fstab('//h/s /mnt/zspace cifs a 0 0\n//h/s /mnt/zspace cifs b 0 0\n', installer.fstab_line('//h/s', {}))

    def test_fstab_never_embeds_credentials(self):
        line = installer.fstab_line('//host/share', {'password': 'test-secret', 'username': 'test-account'})
        self.assertNotIn('test-secret', line)
        self.assertNotIn('test-account', line)
        self.assertIn('credentials=/etc/samba/zspace.credentials', line)
        self.assertIn('_netdev,nofail,x-systemd.automount', line)

    def test_credential_line_injection_rejected(self):
        with self.assertRaises(ValueError):
            installer.credential_bytes('u', 'p\nusername=other')
        self.assertEqual(installer.credential_bytes('u', 'comma,password'), b'username=u\npassword=comma,password\n')

    def test_existing_permissions_preserved(self):
        line = installer.fstab_line('//host/share', {'uid': '1000', 'gid': '1000', 'file_mode': '0750', 'dir_mode': '0750'})
        self.assertIn('uid=1000,gid=1000,file_mode=0750,dir_mode=0750', line)

    def test_option_injection_rejected(self):
        with self.assertRaises(ValueError):
            installer.fstab_line('//host/share', {'vers': '3.1.1,password=other'})

    def test_space_escaping(self):
        self.assertTrue(installer.fstab_line('//host/space share', {}).startswith('//host/space\\040share '))
