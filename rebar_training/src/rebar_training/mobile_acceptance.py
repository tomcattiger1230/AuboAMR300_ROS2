"""Independent, fail-closed evaluation gates for the mobile residual curriculum."""
import argparse
import json
import math
from pathlib import Path


def wilson_lower(successes,count,z=1.959963984540054):
    if not count:return 0.
    p=successes/count;den=1+z*z/count
    return (p+z*z/(2*count)-z*math.sqrt(p*(1-p)/count+z*z/(4*count*count)))/den


def acceptance(reports,phase='full',randomization='expanded'):
    if phase not in ('tray','dock_clamp','full'):raise ValueError('Unknown curriculum phase')
    errors=[];seeds=set();fingerprints=set();sources=set();detectors=set();assets=set();results=[]
    for report in reports:
        if not report.get('completed') or report.get('residual_training') or report.get('regression'):
            errors.append('incomplete, training or regression run cannot be independent evaluation')
        if not report.get('privileged_control_guard') or report.get('steel_truth_control') is not False:
            errors.append('missing camera-only runtime guard')
        if not report.get('residual_policy') or not report.get('residual_checkpoint_sha256'):
            errors.append('no deployed residual checkpoint fingerprint')
        if report.get('residual_total_updates',0)<1 or report.get('residual_trained_steps',0)<1:
            errors.append('residual policy has no actual training updates')
        if report.get('residual_actor_updates',0)<1 or report.get('residual_reward_contract')!='mobile_residual_potential_v2':
            errors.append('residual actor has not trained with the corrected reward contract')
        fingerprints.add(report.get('residual_checkpoint_sha256'))
        for key,bucket in (('source_sha256',sources),('detector_sha256',detectors),('asset_manifest_sha256',assets)):
            value=report.get(key)
            if not isinstance(value,str) or len(value)!=64:errors.append('missing '+key)
            bucket.add(value)
        if report.get('randomization',{}).get('name')!=randomization:
            errors.append('randomization scope mismatch')
        seed=report.get('seed')
        if type(seed) is not int or seed in seeds:errors.append('missing or repeated evaluation seed')
        seeds.add(seed)
        rows=report.get('results',[])
        if len(rows)!=report.get('requested_episodes') or len(rows)!=report.get('episodes'):
            errors.append('missing or extra episode receipts')
        if phase=='full' and (not report.get('end_to_end') or report.get('steel_initialized_on_tray_at_reset')):
            errors.append('reset-only curriculum cannot count as a continuous full mission')
        if phase=='dock_clamp' and not report.get('initialized_at_dock'):
            errors.append('dock skill initialization mismatch')
        for row in rows:
            if row.get('until')!=('tray' if phase=='tray' else 'clamp'):
                errors.append('episode termination task mismatch')
            if row.get('rebar_pose_writes_after_reset') is not False:
                errors.append('unverified steel physics')
            if row.get('success'):
                force=row.get('unsafe_force_N',float('inf'));error=row.get('machine_error_m',float('inf'))
                if row.get('failure') or row.get('timeout') or not math.isfinite(force) or force>250:
                    errors.append('success contradicts physical safety receipt')
                if phase!='tray' and (not row.get('actual_handoff') or not math.isfinite(error) or error>.003):
                    errors.append('success contradicts independent clamp acceptance')
                if phase=='tray' and not row.get('actual_tray'):
                    errors.append('success contradicts independent tray acceptance')
                if phase=='full':
                    reached={v.get('to_stage') for v in row.get('mission_transitions',[])}
                    required={'tray_release','tray_retreat','observe_tray','navigate','dock',
                              'pickup_tray','tray_lift_high','loading'}
                    if not required<=reached or not row.get('tray_verified'):
                        errors.append('missing continuous pickup, transport or clamp phase evidence')
            results.append(row)
    if len(fingerprints)!=1:errors.append('different checkpoints cannot share a stability gate')
    if any(len(bucket)!=1 for bucket in (sources,detectors,assets)):
        errors.append('different controllers, detectors or assets cannot share a stability gate')
    n=len(results);successes=sum(r.get('success') is True for r in results)
    minimum=64 if phase=='full' else 8
    required_lower=.90 if phase=='full' else .60
    lower=wilson_lower(successes,n)
    enough=n>=minimum and len(seeds)>=2
    passed=not errors and enough and successes/max(1,n)>=.95 and lower>=required_lower
    return dict(passed=passed,phase=phase,randomization=randomization,
        episodes=n,successes=successes,success_rate=successes/max(1,n),wilson_95_lower=lower,
        minimum_episodes=minimum,minimum_distinct_seeds=2,distinct_seeds=sorted(s for s in seeds if type(s)is int),
        required_wilson_95_lower=required_lower,
        checkpoint_sha256=next(iter(fingerprints)) if len(fingerprints)==1 else None,
        evidence_errors=sorted(set(errors)),sample_requirement_met=enough,
        failures=[dict(episode=r.get('episode'),stage=r.get('stage'),reason=r.get('failure'),
                       timeout=r.get('timeout')) for r in results if not r.get('success')],
        scope='bounded manipulation residual + deployed visual servo, phase supervisor and odometry; not standalone RL')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('reports',type=Path,nargs='+');p.add_argument('--phase',choices=('tray','dock_clamp','full'),default='full')
    p.add_argument('--randomization',default='expanded');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=acceptance([json.loads(f.read_text()) for f in a.reports],a.phase,a.randomization)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
