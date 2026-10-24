"""Final advisory review gates must retain uncertainty and source disagreement."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from summarize_cosmos_split_replay import require_review


class AssessmentReviewTests(unittest.TestCase):
    def test_uncertain_support_requires_review_even_when_views_agree(self):
        d={'needs_review':False,'rod_support':'uncertain','model_motion_suggestion':'stationary',
           'pipeline_motion_assessment':'stationary','observed_movement_alert':False}
        self.assertTrue(require_review(d)['needs_review'])

    def test_model_stationary_cannot_hide_measured_movement_alert(self):
        d={'needs_review':False,'rod_support':'vehicle_tray','model_motion_suggestion':'stationary',
           'pipeline_motion_assessment':'moving','observed_movement_alert':True}
        r=require_review(d)
        self.assertTrue(r['motion_source_disagreement']);self.assertTrue(r['needs_review'])
        self.assertTrue(r['observed_movement_alert'])


if __name__=='__main__':unittest.main()
