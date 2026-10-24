"""Image-only tracking regressions: vehicle motion, arm-like motion, camera motion."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
SPEC=importlib.util.spec_from_file_location('visual_motion',Path(__file__).parents[1]/'scripts/cosmos_visual_motion.py')
motion=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(motion)


class MotionEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import cv2,numpy,av
        except ImportError:raise unittest.SkipTest('Run in isolated Cosmos environment for image tracking tests')

    def evaluate(self,mode):
        import numpy as np,cv2,av
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fixture.mp4'
            with av.open(str(path),'w') as c:
                s=c.add_stream('libx264',rate=4);s.width=320;s.height=180;s.pix_fmt='yuv420p'
                for i in range(12):
                    frame=np.full((180,320,3),190,np.uint8)
                    for y in range(12,175,30):
                        for x in range(12,310,35):cv2.rectangle(frame,(x,y),(x+5,y+5),(80,80,80),-1)
                    dx=i*4 if mode in ['vehicle','moving_occluded'] else 0
                    cv2.rectangle(frame,(50+dx,75),(140+dx,135),(245,245,245),-1)
                    if not(mode in ['occluded','moving_occluded'] and i>6):
                        for x in [60+dx,105+dx]:
                            cv2.rectangle(frame,(x,80),(x+24,101),(130,215,240),-1)
                            for k in [4,12,20]:cv2.rectangle(frame,(x+k,80),(x+k+1,94),(230,245,250),-1)
                    # White moving arm-shaped patch is not the cyan vehicle fixture.
                    cv2.rectangle(frame,(185+i*3,40),(198+i*3,70),(250,250,250),-1)
                    if mode=='camera':frame=cv2.warpAffine(frame,np.float32([[1,0,i*2],[0,1,0]]),(320,180),borderValue=(190,190,190))
                    f=av.VideoFrame.from_ndarray(frame,format='rgb24')
                    for packet in s.encode(f):c.mux(packet)
                for packet in s.encode():c.mux(packet)
            return motion.analyze(path,Path(directory)/'result.json')

    def test_vehicle_translation_detected_against_fixed_scene(self):
        r=self.evaluate('vehicle');self.assertEqual(r['decision'],'moving')
        self.assertAlmostEqual(r['median_end_displacement_xy_px'][0],44,delta=3)

    def test_moving_white_arm_does_not_move_cyan_vehicle_anchor(self):
        self.assertEqual(self.evaluate('stationary')['decision'],'stationary')

    def test_camera_translation_is_not_claimed_as_vehicle_motion(self):
        r=self.evaluate('camera');self.assertTrue(r['camera_changed_or_unstable'])
        self.assertEqual(r['decision'],'uncertain')

    def test_verified_motion_before_later_occlusion_is_retained(self):
        r=self.evaluate('moving_occluded');self.assertEqual(r['decision'],'moving')
        self.assertFalse(r['end_anchor_visible'])

    def test_lost_or_occluded_anchor_does_not_become_stationary(self):
        self.assertEqual(self.evaluate('occluded')['decision'],'uncertain')


if __name__=='__main__':unittest.main()
