"""Reject conflicting/unsupported outputs before any future advisory consumer."""
import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
SPEC=importlib.util.spec_from_file_location('split_replay',Path(__file__).parents[1]/'scripts/cosmos_reasoner_split_replay.py')
replay=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(replay)


class SplitReplayContracts(unittest.TestCase):
    def answer(self):
        return {'base_motion':'moving','base_motion_evidence':'Base shifts relative to table.',
                'rod_support':'vehicle_tray','rod_support_evidence':'Rod rests across vehicle fixtures.',
                'gripper_contact':'visible_open_or_separated','view_change':False,'summary':'Visible transport.'}

    def test_wrist_cannot_claim_global_base_motion(self):
        with self.assertRaisesRegex(ValueError,'Wrist view'):
            replay.validate_answer(self.answer(),'wrist')
        a=self.answer();a['base_motion']='not_observable'
        self.assertEqual(replay.validate_answer(a,'wrist'),a)

    def test_open_fingers_cannot_be_accepted_as_gripper_support(self):
        a=self.answer();a['rod_support']='gripper'
        with self.assertRaisesRegex(ValueError,'contradicts'):
            replay.validate_answer(a,'overview')

    def test_nearby_or_occluded_gripper_is_not_positive_support_evidence(self):
        a=self.answer();a['rod_support']='gripper';a['gripper_contact']='not_clear'
        with self.assertRaisesRegex(ValueError,'closed-finger'):
            replay.validate_answer(a,'overview')

    def test_unknown_labels_and_non_boolean_view_change_are_rejected(self):
        a=self.answer();a['rod_support']='secure_grasp'
        with self.assertRaises(ValueError):replay.validate_answer(a,'overview')
        a=self.answer();a['view_change']='false'
        with self.assertRaises(ValueError):replay.validate_answer(a,'overview')

    def test_schema_restricts_wrist_scope_without_forcing_grasp_or_motion_labels(self):
        wrist=replay.schema_for('wrist');overview=replay.schema_for('overview')
        self.assertEqual(wrist['properties']['base_motion']['enum'],['not_observable'])
        self.assertIn('uncertain',overview['properties']['base_motion']['enum'])
        self.assertEqual(set(overview['properties']['rod_support']['enum']),replay.SUPPORT)
        self.assertFalse(overview['additionalProperties'])

    def test_fusion_never_uses_wrist_to_override_global_motion(self):
        a=self.answer();b=self.answer();b['base_motion']='not_observable'
        result=replay.fuse_pair({'schema_valid':True,'answer':a},{'schema_valid':True,'answer':b})
        self.assertEqual(result['base_motion'],'moving')
        self.assertFalse(result['positive_grasp_authorized'])

    def test_cross_view_support_conflict_requires_review(self):
        a=self.answer();b=self.answer();b.update(rod_support='gripper',gripper_contact='visible_closed_around_rod')
        result=replay.fuse_pair({'schema_valid':True,'answer':a},{'schema_valid':True,'answer':b})
        self.assertTrue(result['needs_review']);self.assertEqual(result['rod_support'],'uncertain')

    def test_one_view_alone_cannot_authorize_grasp(self):
        a=self.answer();a.update(rod_support='gripper',gripper_contact='visible_closed_around_rod')
        b=self.answer();b['rod_support']='uncertain'
        result=replay.fuse_pair({'schema_valid':True,'answer':a},{'schema_valid':True,'answer':b})
        self.assertTrue(result['needs_review']);self.assertEqual(result['rod_support'],'uncertain')

    def test_both_views_have_uncertainty_and_no_metric_success_target(self):
        for view in ['overview','wrist']:
            p=replay.prompt_for({'view':view,'source_start_s':180,'source_end_s':200})
            self.assertIn('use uncertain',p)
            self.assertNotIn('1.253',p)
            self.assertNotIn('navigate',p)


if __name__=='__main__':unittest.main()
