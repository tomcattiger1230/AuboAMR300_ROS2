"""Independent image/FK replay and physical stage review; no simulator launch."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .fiducial_localization import CameraCalibration,inverse,rotation_distance
from .machine_fiducial import CharucoFixtureEstimator,station_from_board
from .workcells import WorkcellLayout,FROZEN_POLICY_SHA256


def review(directory,layout):
    directory=Path(directory);report=json.loads((directory/'report.json').read_text())
    failures=[];replays=[]
    def require(value,message):
        if not value:failures.append(message)
    require(not report['errors'],'runner exception')
    require(report['initialized_on_tray'] and not report['end_to_end_source_pickup'],
            'stage scope must explicitly exclude source pickup')
    require(report['training'] is False and report['cosmos_used'] is False,'unexpected training/Cosmos')
    before=report['policy_optimizer_before'];after=report['policy_optimizer_after']
    require(before==after and before['sha256']==FROZEN_POLICY_SHA256 and
            before['model_optimizer_tensors']==52 and before['counters']==dict(updates=466,actor_updates=170,policy_steps=36602),
            'frozen model/optimizer identity mismatch')
    require('forbidden_pose_write_attempts' in report and not report['forbidden_pose_write_attempts'],
            'missing/failed live root-pose write guard')
    mounts=report.get('mounts',[])
    require(len(mounts)==3 and all(m.get('rigid_bracket') and m['collision_backing'] and
            m['board_prim'].startswith(m['parent_prim']+'/') and
            m['attached_frame_prim']==m['parent_prim']+'/TopBeam' for m in mounts),
            'board is not a physically supported fixed machine CAD child')
    station=layout.station(report['machine_id']);profile=layout.profiles[station.profile_id]
    world_station=np.eye(4);world_station[:3,:3]=Rotation.from_euler('z',station.pose_map.yaw).as_matrix()
    world_station[:3,3]=station.pose_map.values()[:3]
    world_fixture=world_station.copy();world_fixture[:3,3]+=world_station[:3,:3]@profile.fixture_goal
    for name in report['fid_rows_files']:
        row=json.loads((directory/name).read_text());path=directory/row['image_file']
        require(hashlib.sha256(path.read_bytes()).hexdigest()==row['image_sha256'],'raw image SHA mismatch: '+name)
        if row.get('reason') in ('observation motion not settled','wrist still settling'):continue
        image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
        c=CameraCalibration('wrist_mono','mounted_charuco_K','usd_fixed_wrist_mount',
            (image.shape[1],image.shape[0]),np.array(row['K']),(0.,)*5,np.array(row['fixed_mount']),
            'control_base','mono_camera_link','wrist_optical','sim_clock')
        estimator=CharucoFixtureEstimator(c,layout,report['machine_id'])
        output=dict(file=name,stamp_s=row['stamp_s'],frame_id='fid_'+str(row['frame']))
        try:
            obs=estimator.estimate(image,machine_id=report['machine_id'],frame_id='replay_'+str(row['frame']),
                image_stamp_s=row['stamp_s'],fk_stamp_s=row['stamp_s'],now_s=row['stamp_s'],
                base_from_flange=np.array(row['base_from_mount']),camera_id=c.camera_id,
                calibration_id=c.calibration_id,hand_eye_id=c.hand_eye_id,fk_base_frame_id=c.base_frame_id,
                fk_flange_frame_id=c.flange_frame_id,optical_frame_id=c.optical_frame_id,clock_id=c.clock_id)
            estimated=np.array(row['base_pose_odom'])@np.array(obs['base_from_fixture'])
            camera=np.array(row['base_pose_odom'])@np.array(row['base_from_mount'])@np.array(row['fixed_mount'])
            metadata=np.array(row['camera_world_pose_for_evaluation'])
            truth=inverse(camera)@world_station@station_from_board()
            projected=cv2.projectPoints(estimator.points,cv2.Rodrigues(truth[:3,:3])[0],truth[:3,3],np.array(c.K),None)[0].reshape(-1,2)
            output.update(valid=True,fixture_origin_error_m=float(np.linalg.norm(estimated[:3,3]-world_fixture[:3,3])),
                fixture_orientation_error_deg=math.degrees(rotation_distance(estimated,world_fixture)),
                estimated_fixture_odom=estimated.tolist(),pnp_rms_px=obs['candidates'][0]['rms_px'],
                fixture_from_base=obs['fixture_from_base'],
                camera_metadata_translation_error_m=float(np.linalg.norm(camera[:3,3]-metadata[:3,3])),
                camera_metadata_rotation_error_deg=math.degrees(rotation_distance(camera,metadata)),
                surveyed_projection_rms_px=float(np.sqrt(np.mean(np.sum((projected-obs['corners_px'])**2,axis=1)))))
        except ValueError as exc:output.update(valid=False,error=str(exc))
        replays.append(output)
    stable=[e for e in report.get('fid_events',[]) if e['event']=='fresh_board_pose_stable']
    require(bool(stable),'no stable mounted-board observation')
    require(all(r['valid'] for r in replays) and len(replays)>=3,'settled raw image replay failed/incomplete')
    valid=[r for r in replays if r['valid']]
    require(bool(valid) and max(r['fixture_origin_error_m'] for r in valid)<=.003,
            'independent surveyed fixture-origin error exceeds 3mm or missing')
    require(bool(valid) and all(r['camera_metadata_translation_error_m']<.0001 and
            r['camera_metadata_rotation_error_deg']<.01 for r in valid),'render camera/FK metadata mismatch')
    lookup={r['frame_id']:r for r in valid}
    for event in stable:
        evidence=[lookup.get(fid) for fid in event['evidence_frame_ids']]
        require(len(evidence)==3 and all(evidence),'stable session raw frames missing')
        if len(evidence)!=3 or not all(evidence):continue
        require(round((evidence[-1]['stamp_s']-evidence[0]['stamp_s'])*1e9)<=200_000_000,
                'stable session evidence expired')
        poses=[np.array(r['fixture_from_base']) for r in evidence]
        require(all(np.linalg.norm(a[:3,3]-b[:3,3])<=.003 and rotation_distance(a,b)<=math.radians(.5)
                    for a in poses for b in poses),'independent multi-frame stability gate')
    peaks=report.get('peak_contact_loads_N',{})
    require(bool(peaks) and max(peaks.get('arm_all',{}).values(),default=math.inf)<.5,'arm contact peak not below .5 N')
    native=report.get('native_results',[])
    for r in native:require(r['success'],'native stage failure: '+str(r.get('failure')))
    transfer=report['requested_until']=='transfer'
    if transfer:
        require(len(native)==1 and native[0]['success'],'one continuous physical transfer not completed')
        require(report.get('fid_verified') and report.get('fid_corrections',0)>0,'correction/reobservation not completed')
        names=[e['event'] for e in report.get('fid_events',[])]
        require(all(n in names for n in ['start_guarded_correction','correction_stopped','final_pose_confirmed_after_return']),
                'missing correction/stop/final confirmation evidence')
        if native:
            r=native[0]
            require(r.get('actual_handoff') and r['machine_error_m']<=.003,'physical handoff 3mm gate')
            require(min(r['jaw_forces_N'])>=.5 and max(map(abs,r['finger_positions']))<.003
                    and max(r['raw_finger_forces_N'])<.5,'four real jaws / open unloaded fingers')
            require(r['tcp_clearance_m']>=.06 and r['steel_linear_speed_m_s']<=.04
                    and r['steel_angular_speed_rad_s']<=.2,'retreat / steel velocity gate')
            require(abs(r['bar_axis_for_evaluation'][2])>=math.cos(math.radians(3)),'physical vertical steel axis')
    return dict(schema='mounted_fiducial_independent_stage_review_v1',run_id=report['run_id'],
        report_sha256=hashlib.sha256((directory/'report.json').read_bytes()).hexdigest(),
        scope='initialized-tray arrival/observation/correction/transfer; source pickup excluded',
        passed=not failures,transfer_test=transfer,full_mission_acceptance=False,failures=failures,
        stable_observation_sessions=len(stable),raw_image_replays=replays)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    p.add_argument('--layout',type=Path,required=True);args=p.parse_args()
    result=review(args.directory,WorkcellLayout.load(args.layout))
    target=args.directory/'independent_review.json';target.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='raw_image_replays'},indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':main()
