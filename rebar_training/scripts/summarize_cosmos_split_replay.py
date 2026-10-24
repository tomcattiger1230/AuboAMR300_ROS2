"""Summarize saved offline responses; never supply reference labels to the model."""
from pathlib import Path
import argparse
import collections
import json
import statistics
from cosmos_reasoner_split_replay import baseline, fuse_pair, now, save, sha256


def parsed(response):
    try:
        value=json.loads(response['final_output'])
        return value if isinstance(value,dict) else {}
    except (ValueError,KeyError):return {}


def require_review(assessment):
    """Keep uncertainty and source disagreement visible to advisory consumers."""
    model=assessment['model_motion_suggestion']
    measured=assessment['pipeline_motion_assessment']
    assessment['motion_source_disagreement']=model in {'moving','stationary'} and model!=measured
    assessment['needs_review']=bool(assessment['needs_review']
        or assessment['rod_support']=='uncertain'
        or model in {'uncertain','malformed','not_observable'}
        or measured=='uncertain' or assessment['motion_source_disagreement'])
    if assessment['rod_support']=='uncertain':
        assessment['reason']='Support unresolved; neither a grasp nor absence of grasp is inferred'
    return assessment


def summarize(root):
    manifest=baseline(root);plan=json.loads((root/'evaluation_plan.json').read_text())
    responses={}
    for clip in manifest['clips']:
        response=json.loads((root/'responses'/(clip['id']+'.json')).read_text())
        assert response['id']==clip['id'] and response['clip_sha256']==clip['sha256']
        assert sha256(root/clip['path'])==clip['sha256']
        for f in response['keyframes']:assert sha256(root/f['path'])==f['sha256']
        responses[clip['id']]=response
    assert len(responses)==32 and len({r['identity'] for r in responses.values()})==1
    pairs=[]
    for index in range(16):
        a=responses[f'overview_{index:03d}'];b=responses[f'wrist_{index:03d}']
        motion_path=root/'visual_motion'/f'overview_{index:03d}.json'
        motion=json.loads(motion_path.read_text())
        assert motion['input_sha256']==a['clip_sha256']
        assert a['image_derived_motion_evidence']['sha256']==sha256(motion_path)
        fused=fuse_pair(a,b)
        fused.update(pipeline_motion_assessment=motion['decision'],
            pipeline_motion_source='independent image-only feature tracking, not Cosmos detection',
            model_motion_suggestion=parsed(a).get('base_motion','malformed'),
            observed_movement_alert=motion['decision']=='moving')
        require_review(fused)
        pairs.append({'index':index,'start_s':a['source_start_s'],'end_s':a['source_end_s'],
            'raw_overview':parsed(a),'raw_wrist':parsed(b),
            'overview_schema_valid':a['schema_valid'],'wrist_schema_valid':b['schema_valid'],
            'independent_visual_motion':motion,'fused':fused})
    def motion(indices):
        return dict(collections.Counter(pairs[i]['raw_overview'].get('base_motion','malformed') for i in indices))
    support=[]
    for i in plan['non_gripper_vehicle_support_indices']:
        p=pairs[i];a=p['raw_overview'];b=p['raw_wrist']
        support.append({'index':i,'overview_support':a.get('rod_support','malformed'),
            'wrist_support':b.get('rod_support','malformed'),
            'either_view_gripper_claim':'gripper' in [a.get('rod_support'),b.get('rod_support')],
            'overview_correct_tray':a.get('rod_support')=='vehicle_tray',
            'wrist_correct_tray':b.get('rod_support')=='vehicle_tray',
            'overview_schema_valid':p['overview_schema_valid'],'wrist_schema_valid':p['wrist_schema_valid'],
            'fused':p['fused']})
    latencies=[r['latency_s'] for r in responses.values()]
    summary={'at':now(),'complete_responses':len(responses),'schema_and_scope_valid':sum(r['schema_valid'] for r in responses.values()),
        'identity':next(iter(responses.values()))['identity'],'evaluation_plan_sha256':sha256(root/'evaluation_plan.json'),
        'input_strategy':'Three overview keyframes plus 31 original high-resolution wrist snapshots in 16 intervals; overview model also receives independent image-derived tracking measurements',
        'motion_positive_cases':plan['motion_positive_indices'],'motion_positive_raw_counts':motion(plan['motion_positive_indices']),
        'motion_negative_cases':plan['motion_negative_indices'],'motion_negative_raw_counts':motion(plan['motion_negative_indices']),
        'independent_tracker_positive_counts':dict(collections.Counter(pairs[i]['independent_visual_motion']['decision'] for i in plan['motion_positive_indices'])),
        'independent_tracker_negative_counts':dict(collections.Counter(pairs[i]['independent_visual_motion']['decision'] for i in plan['motion_negative_indices'])),
        'independent_tracker_transition_counts':dict(collections.Counter(pairs[i]['independent_visual_motion']['decision'] for i in plan['motion_transition_excluded_indices'])),
        'support_cases':support,'non_gripper_windows':len(support),
        'windows_with_any_raw_false_gripper_claim':sum(p['either_view_gripper_claim'] for p in support),
        'windows_correct_tray_in_overview':sum(p['overview_correct_tray'] for p in support),
        'windows_correct_tray_in_wrist':sum(p['wrist_correct_tray'] for p in support),
        'uncertainty_is_not_correct_support_identification':True,
        'total_response_latency_s':sum(latencies),'median_response_latency_s':statistics.median(latencies),
        'format_constraints_are_not_semantic_accuracy':True,'wrist_motion_scope_is_forced_not_model_accuracy':True,
        'general_semantic_acceptance':False,'online_control_ready':False,'physical_acceptance':False,
        'new_independent_episodes':0,'simulation_started':False,'training_started':False}
    save(root/'fused_pairs.json',pairs);save(root/'results_summary.json',summary)
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    print(json.dumps(summarize(p.parse_args().output.resolve()),ensure_ascii=False,indent=2))
