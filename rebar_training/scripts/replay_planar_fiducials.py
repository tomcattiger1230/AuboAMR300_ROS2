"""Replay recorded Isaac fiducial PNG+FK using an explicit level-floor prior.

No simulator, Cosmos or policy is loaded. Keeps rejected frames and raw SE(3).
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from rebar_training.fiducial_localization import (
    CameraCalibration, MarkerAnchor, PlanarWristBoardEstimator, inverse)


def replay(path):
    d=json.loads(path.read_text());construction=d['construction']
    original=d['samples'][0]['observation']
    c=CameraCalibration(original['camera_id'],original['calibration_id'],original['hand_eye_id'],
        tuple(construction['image_size']),construction['K'],(0.,)*5,construction['fixed_mount'],
        original['base_frame_id'],original['flange_frame_id'],original['optical_frame_id'],original['clock_id'])
    fixture_marker=inverse(construction['world_fixture'])@np.array(construction['world_marker'])
    anchors=[]
    for spec in construction['markers']:
        off=np.eye(4);off[0,3]=spec['centre_x_m']
        anchors.append(MarkerAnchor('machine_1','DICT_5X5_100',spec['id'],spec['length_m'],fixture_marker@off,'scene_survey'))
    height=d['reset_perturbation']['desired_fixture_from_base'][2][3]
    estimator=PlanarWristBoardEstimator(c,anchors,fixture_from_base_z_m=height)
    context=dict(camera_id=c.camera_id,calibration_id=c.calibration_id,hand_eye_id=c.hand_eye_id,
        fk_base_frame_id=c.base_frame_id,fk_flange_frame_id=c.flange_frame_id,
        optical_frame_id=c.optical_frame_id,clock_id=c.clock_id)
    rows=[]
    for s in d['samples']:
        image=path.parent/s['image'];row=dict(step=s['step'],image=s['image'])
        try:
            obs=estimator.estimate(cv2.imread(str(image),0),machine_id='machine_1',frame_id=f"replay_{s['step']}",
                image_stamp_s=s['stamp_s'],fk_stamp_s=s['stamp_s'],now_s=s['stamp_s'],
                base_from_flange=s['robot_input_base_from_mount'],**context)
            truth=inverse(s['evaluation_only']['base_from_fixture_truth'])
            row.update(accepted=True,observation=obs,evaluation_only_base_error_m=float(
                np.linalg.norm(np.array(obs['fixture_from_base'])[:3,3]-truth[:3,3])))
        except ValueError as exc:row.update(accepted=False,error=str(exc))
        rows.append(row)
    return dict(scope='same_recorded_frames_offline_planar_replay',source_report=str(path),
        known_height_prior_m=height,prior_source='declared scene survey before initial offset',
        known_xy_yaw_truth_supplied=False,simulation_started=False,policy_loaded=False,cosmos_used=False,
        rows=rows,all_frames_passed=all(r['accepted'] for r in rows),closed_loop_passed=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();result=replay(args.input)
    with args.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
    for row in result['rows']:print(row['step'],row['accepted'],row.get('error'))
