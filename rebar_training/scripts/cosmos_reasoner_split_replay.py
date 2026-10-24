"""Separate-view, higher-resolution OFFLINE Cosmos replay. No simulator or policy imports."""
from pathlib import Path
import argparse
import hashlib
import json
import time

from cosmos_reasoner_replay import MODEL_ID, MODEL_REVISION, VIDEO_SHA, POLICY_SHA, sha256, save, now, intervals
from cosmos_generation_audit import GenerationScoreAudit, GenerationNumericsError, generation_receipt, validate_local_prefill

# Original recorder pixels: wrist pasted at (876,118), size 384x231.
# Use 230 rows for a legal even-sized H.264 frame; no wrist upsampling.
VIEWS = {
    'overview': {'filter': 'crop=1280:630:0:90,drawbox=x=876:y=0:w=384:h=259:color=0xdcdcdc:t=fill',
                 'pixel_budget_per_frame': 768*384, 'description': 'Fixed external overview; top-right rectangle masks the removed inset.'},
    'wrist': {'filter': 'crop=384:230:876:118',
              'pixel_budget_per_frame': 384*230, 'description': 'Only the recorded wrist monochrome inset, without the overview.'},
}
BASE = {'moving', 'stationary', 'uncertain', 'not_observable'}
SUPPORT = {'gripper', 'vehicle_tray', 'source_support', 'machine', 'transition', 'uncertain'}
CONTACT = {'visible_closed_around_rod', 'visible_open_or_separated', 'not_clear'}
KEYS = {'base_motion', 'base_motion_evidence', 'rod_support', 'rod_support_evidence',
        'gripper_contact', 'view_change', 'summary'}


def baseline(root):
    m = json.loads((root/'manifest.json').read_text())
    candidates=[(m['source_video'],m['frozen_policy'])]
    candidates.extend((v['source_video'],v['frozen_policy']) for v in m.get('host_bindings',{}).values())
    bound=next(((v,p) for v,p in candidates if Path(v).is_file() and Path(p).is_file()),None)
    assert bound, 'No saved immutable input binding on this host'
    assert sha256(bound[0]) == m['source_video_sha256'] == VIDEO_SHA
    assert sha256(bound[1]) == m['frozen_policy_sha256'] == POLICY_SHA
    return m


def prepare(args):
    import subprocess
    r = args.output.resolve()
    assert not r.exists(), 'Use a new output directory; preserve earlier evidence'
    assert sha256(args.video) == VIDEO_SHA and sha256(args.policy) == POLICY_SHA
    r.mkdir(parents=True); (r/'clips').mkdir(); (r/'responses').mkdir()
    clips = []
    for index, (start, end) in enumerate(intervals(316.4, 20)):
        for view, spec in VIEWS.items():
            name = f'{view}_{index:03d}'
            p = r/'clips'/f'{name}.mp4'
            subprocess.run([args.ffmpeg, '-v', 'error', '-n', '-ss', str(start), '-i', str(args.video),
                '-t', str(end-start), '-vf', spec['filter']+',fps=4', '-an', '-c:v', 'libx264',
                '-crf', '16', '-pix_fmt', 'yuv420p', str(p)], check=True, timeout=60)
            clips.append({'id': name, 'index': index, 'view': view, 'source_start_s': start,
                'source_end_s': end, 'path': str(p.relative_to(r)), 'sha256': sha256(p),
                'pixel_budget_per_frame': spec['pixel_budget_per_frame']})
    save(r/'manifest.json', {'schema': 'cosmos_split_replay_v2', 'at': now(),
        'source_video': str(args.video.resolve()), 'source_video_sha256': VIDEO_SHA,
        'frozen_policy': str(args.policy.resolve()), 'frozen_policy_sha256': POLICY_SHA,
        'source_duration_seconds':316.4, 'sampling_fps':4, 'views':VIEWS,
        'overview_missing_pixels_cannot_be_recovered':True, 'wrist_native_recorded_pixels':[384,231],
        'wrist_extracted_pixels':[384,230], 'no_wrist_detail_created_by_upscaling':True,
        'top_phase_overlay_excluded':True, 'native_report_or_metrics_given_to_model':False,
        'clips':clips, 'model_id':MODEL_ID, 'model_revision':MODEL_REVISION,
        'training':False,'simulation':False,'control_output_connected':False})
    baseline(r)
    print(json.dumps({'prepared':len(clips),'output':str(r)}),flush=True)


def prompt_for(clip, motion=None):
    view = clip['view']
    scope = ('These are ordered first, middle and last frames from the same interval. '
             'Track the WHITE WHEELED VEHICLE carrying the arm, not the stationary large machine pedestal. '
             'Compare the vehicle position relative to fixed tables and the large machine '
             'at the beginning, middle and end. Describe its location in the first and last image. '
             'Arm motion is not vehicle movement. '
             'A change in camera viewpoint is not evidence of base motion; use uncertain if ambiguous. '
             'The plain top-right rectangle is missing pixels, not an object. '
             if view == 'overview' else
             'This moving wrist camera does NOT show global base position reliably. '
             'Set base_motion to not_observable. Camera motion does not prove base motion. ')
    measurement=''
    if motion is not None:
        fields=['camera_changed_or_unstable','initial_anchor_tracks','surviving_anchor_tracks',
                'max_median_excursion_px','median_end_displacement_xy_px','background_end_shift_xy_px',
                'observed_peak_vector_spread_px','affine_inlier_ratio','observed_peak_relative_s',
                'observed_peak_track_count','observed_peak_displacement_xy_px','end_anchor_visible']
        measurement=(' Independent image-only measurement from this same clip (not simulator poses '
            'or a native task label): cyan fixtures on the white vehicle were tracked through the '
            '4fps frames with forward/backward optical flow checks and static-scene registration. '
            +json.dumps({k:motion[k] for k in fields if k in motion},sort_keys=True)+
            '. Values are pixels, not metres. Consider reliable fixture displacement relative to '
            'the scene when judging the vehicle. State this separate measurement as your evidence '
            'if you use it; do not claim you independently measured it. If tracks or camera are '
            'unreliable, keep the motion uncertain. Verified movement observed before later '
            'occlusion is evidence of movement during the window, but not of the final position. ')
    return (
        f'Inspect this {view} recording of a robotics simulation, {clip["source_end_s"]-clip["source_start_s"]:.1f} seconds. '
        + scope + measurement +
        'Identify the rod support from visible evidence: gripper fingers enclosing the rod, '
        'rod resting across fixtures on the vehicle, supports on a separate table, or machine jaws. '
        'Proximity alone does not establish a grasp. An open or separated gripper does not support the rod. '
        'If the support is hidden, use uncertain. Do not invent secure grasp, force, task success or commands. '
        'For wrist view, do not identify an off-screen support from prior knowledge. '
        'Answer in English with one concise JSON object, exactly these keys. '
        'Use one short sentence per text field, at most 25 words; no claims about task success. '
        'Choose base_motion from moving, stationary, uncertain, not_observable; '
        'rod_support from gripper, vehicle_tray, source_support, machine, transition, uncertain; '
        'gripper_contact from visible_closed_around_rod, visible_open_or_separated, not_clear. '
        'Evidence must describe visible spatial or temporal changes, not repeat instructions. '
        'Template: {"base_motion":"uncertain","base_motion_evidence":"Visible positions compared.",'
        '"rod_support":"uncertain","rod_support_evidence":"Visible support or occlusion.",'
        '"gripper_contact":"not_clear","view_change":false,"summary":"Concise observation."}'
    )


def validate_answer(a, view):
    if not isinstance(a,dict) or set(a)!=KEYS:
        raise ValueError('Incorrect exact keys')
    if a['base_motion'] not in BASE or a['rod_support'] not in SUPPORT or a['gripper_contact'] not in CONTACT:
        raise ValueError('Invalid classification')
    for k in ['base_motion_evidence','rod_support_evidence','summary']:
        if not isinstance(a[k],str) or not a[k].strip():raise ValueError('Missing visible evidence')
    if type(a['view_change']) is not bool:raise ValueError('Invalid view_change')
    if view=='wrist' and a['base_motion']!='not_observable':
        raise ValueError('Wrist view cannot establish global base motion')
    if a['rod_support']=='gripper' and a['gripper_contact']!='visible_closed_around_rod':
        raise ValueError('Gripper support contradicts missing visible closed-finger evidence')
    return a


def schema_for(view):
    props = {k: {'type':'string','maxLength':160} for k in sorted(KEYS)}
    props['base_motion']={'type':'string','enum':['not_observable'] if view=='wrist' else sorted(BASE)}
    props['rod_support']={'type':'string','enum':sorted(SUPPORT)}
    props['gripper_contact']={'type':'string','enum':sorted(CONTACT)}
    props['view_change']={'type':'boolean'}
    return {'type':'object','properties':props,'required':sorted(KEYS),'additionalProperties':False}


def constraint_token_data(tokenizer):
    """LMFE core adapter; its 0.11.3 HF helper imports a removed HF5 alias.

    Uses LMFE's public tokenizer-data/core API without patching installed modules.
    Build once for the whole batch, rather than rebuilding the vocabulary per clip.
    """
    from lmformatenforcer.tokenenforcer import TokenEnforcerTokenizerData
    prefix=tokenizer.encode('0')[-1];special=set(tokenizer.all_special_ids);tokens=[]
    for index in range(len(tokenizer)):
        if index in special:continue
        text=tokenizer.decode([prefix,index])[1:]
        tokens.append((index,text,len(text)>len(tokenizer.decode([index]))))
    return TokenEnforcerTokenizerData(tokens,lambda ids:tokenizer.decode(ids).rstrip('\ufffd'),
                                     tokenizer.eos_token_id,False,len(tokenizer))


def constraint_prefix(token_data, schema):
    from lmformatenforcer import JsonSchemaParser
    from lmformatenforcer.tokenenforcer import TokenEnforcer
    enforcer=TokenEnforcer(token_data,JsonSchemaParser(schema))
    return lambda batch_id, sent:enforcer.get_allowed_tokens(sent.tolist()).allowed_tokens


def fuse_pair(overview, wrist):
    """Conservative advisory fusion; abstention is never physical acceptance."""
    a=overview.get('answer') if overview.get('schema_valid') else None
    b=wrist.get('answer') if wrist.get('schema_valid') else None
    result={'base_motion':a['base_motion'] if a else 'uncertain',
            'base_motion_source':'overview_only','rod_support':'uncertain',
            'needs_review':not(a and b),'positive_grasp_authorized':False,
            'advisory_only':True,'physical_acceptance':False}
    if not(a and b):
        result['reason']='At least one model response was rejected'
        return result
    sa,sb=a['rod_support'],b['rod_support']
    if sa!=sb and 'uncertain' not in (sa,sb):
        result.update(needs_review=True,reason='Views disagree on support')
    elif 'gripper' in (sa,sb):
        if sa==sb=='gripper' and all(x['gripper_contact']=='visible_closed_around_rod' for x in [a,b]):
            result.update(rod_support='gripper_visually_suggested',reason='Both views suggest closed fingers; secure grasp still unproved')
        else:
            result.update(needs_review=True,reason='Positive grasp withheld without agreement and visible closed fingers in both views')
    else:
        result.update(rod_support=sa if sa!='uncertain' else sb,
                      reason='Non-gripper support suggestion; no force or success claim')
    return result


def model_inputs(processor, video, prompt, clip, root):
    import av
    from PIL import Image
    keydir=root/'keyframes';keydir.mkdir(exist_ok=True)
    images=[];content=[];receipts=[]
    if clip.get('raw_wrist_frames'):
        for raw in clip['raw_wrist_frames']:
            path=root/raw['path'];assert sha256(path)==raw['sha256']
            images.append(Image.open(path).convert('RGB'));relative=raw['source_time_s']-clip['source_start_s']
            receipts.append(dict(raw,relative_time_s=relative,input_kind='original_full_resolution_wrist_camera'))
            content.extend([{'type':'text','text':f'Raw wrist frame at {relative:.2f} seconds: '},{'type':'image'}])
    else:
        with av.open(str(video)) as c:
            stream=c.streams.video[0];assert abs(float(stream.average_rate)-4)<.001
            frames=[f.to_image() for f in c.decode(stream)]
        for index in [0,len(frames)//2,len(frames)-1]:
            frame=frames[index];target=keydir/f"{clip['id']}_{index:03d}.png"
            frame.save(target);images.append(frame)
            receipts.append({'relative_time_s':index/4,'source_time_s':clip['source_start_s']+index/4,
                             'path':str(target.relative_to(root)),'sha256':sha256(target)})
            content.extend([{'type':'text','text':f'Frame at {index/4:.2f} seconds: '},{'type':'image'}])
    content.append({'type':'text','text':prompt})
    text=processor.apply_chat_template([{'role':'user','content':content}],tokenize=False,
        add_generation_prompt=True,enable_thinking=False)
    # A clear global view and native wrist pixels; no fabricated wrist detail.
    max_pixels=1024*512 if clip['view']=='overview' else 1024*615
    inputs=processor(text=[text],images=images,return_tensors='pt',
        images_kwargs={'size':{'shortest_edge':64*64,'longest_edge':max_pixels}})
    return inputs,receipts


def infer(args):
    import torch,transformers
    r=args.output.resolve();m=baseline(r);mp=args.model_path.resolve()
    mm=json.loads((mp/'download_manifest.json').read_text());assert mm['revision']==MODEL_REVISION
    for i in mm['files']:assert sha256(mp/i['name'])==i['actual_sha256'],i['name']
    processor=transformers.AutoProcessor.from_pretrained(str(mp),local_files_only=True)
    device='cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else None
    assert device, 'A local GPU backend is required'
    # Local MPS BF16+SDPA produced all-NaN decode logits in the controlled replay.
    # Preserve CUDA's existing dtype; MPS uses the independently checked FP32 path.
    dtype=torch.float32 if device=='mps' else torch.bfloat16
    model=transformers.AutoModelForImageTextToText.from_pretrained(str(mp),dtype=dtype,
        attn_implementation='sdpa',local_files_only=True).to(device).eval()
    torch.set_grad_enabled(False);torch.manual_seed(0)
    token_data=constraint_token_data(processor.tokenizer)
    config={'script_sha256':sha256(Path(__file__)),'shared_utilities_sha256':sha256(Path(__file__).with_name('cosmos_reasoner_replay.py')),
        'model_revision':MODEL_REVISION,'thinking':False,'max_new_tokens':args.max_tokens,
        'input_strategy':'three_overview_frames_and_original_highres_wrist_snapshots','format_enforcer':'lm-format-enforcer==0.11.3',
        'device':device,'dtype':str(dtype),'attention':'sdpa','sampling':False,'manifest_sha256':sha256(r/'manifest.json'),
        'generation_audit_sha256':sha256(Path(__file__).with_name('cosmos_generation_audit.py')),
        'visual_motion_manifest_sha256':sha256(r/'visual_motion_manifest.json'),
        'execution_order':'focus_first_then_remaining'}
    identity=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()
    execution={'at':now(),'config':config,'identity':identity,'torch':torch.__version__,
        'transformers':transformers.__version__,'gpu':torch.cuda.get_device_name(0) if device=='cuda' else 'Apple M1 Max / MPS','training':False,
        'optimizer_loaded':False,'simulation_started':False,'control_output_connected':False}
    # Each invocation gets its own receipt; never overwrite the smoke history.
    receipts=r/'executions';receipts.mkdir(exist_ok=True)
    save(receipts/(str(time.time_ns())+'.json'),execution)
    priority={v:i for i,v in enumerate([10,8,9,11,6,7])}
    ordered=sorted(m['clips'],key=lambda c:(priority.get(c['index'],100+c['index']),c['view']))
    for clip in ordered:
        if args.only and clip['id'] not in args.only:continue
        p=r/clip['path'];assert sha256(p)==clip['sha256']
        motion=None;motion_ref=None
        if clip['view']=='overview':
            motion_path=r/'visual_motion'/(clip['id']+'.json')
            expected=json.loads((r/'visual_motion_manifest.json').read_text())['files'][clip['id']]
            assert sha256(motion_path)==expected
            motion=json.loads(motion_path.read_text());assert motion['input_sha256']==clip['sha256']
            motion_ref={'path':str(motion_path.relative_to(r)),'sha256':expected}
        prompt=prompt_for(clip,motion)
        prompt_sha=hashlib.sha256(prompt.encode()).hexdigest();target=r/'responses'/(clip['id']+'.json')
        if target.exists():
            previous=json.loads(target.read_text());assert previous['identity']==identity
            assert previous['clip_sha256']==clip['sha256'] and previous['prompt_sha256']==prompt_sha
            continue
        started=time.monotonic();x,keyframes=model_inputs(processor,p,prompt,clip,r)
        grid=x['image_grid_thw'].tolist();tokens=x['input_ids'].shape[-1]
        if device=='mps':validate_local_prefill(grid,tokens)
        assert tokens+args.max_tokens<model.config.text_config.max_position_embeddings
        x=x.to(model.device);x={k:(v.to(dtype) if torch.is_tensor(v) and v.is_floating_point() else v) for k,v in x.items()}
        if device=='cuda':torch.cuda.reset_peak_memory_stats()
        schema=schema_for(clip['view'])
        prefix=constraint_prefix(token_data,schema)
        audit=GenerationScoreAudit(tokens,allow_masked=True)
        numerical_error=None;g=None;ids=[]
        try:
            with torch.inference_mode():g=model.generate(**x,do_sample=False,max_new_tokens=args.max_tokens,
                prefix_allowed_tokens_fn=prefix,logits_processor=[audit])
            ids=g[0,tokens:].tolist()
        except GenerationNumericsError as exc:numerical_error=str(exc)
        generation=generation_receipt(ids,processor.tokenizer,False,args.max_tokens,time.monotonic()-started,float('inf'))
        if numerical_error:generation['generation_status']='runtime_error'
        raw=generation['decoded'].strip();final=generation['final_answer']
        answer,error=None,None
        try:
            if numerical_error:raise ValueError(numerical_error)
            if not generation['answer_complete']:raise ValueError('Generation incomplete; not semantic uncertainty')
            answer=validate_answer(json.loads(final),clip['view'])
        except (ValueError,TypeError,KeyError) as exc:error=str(exc)
        out={'at':now(),'id':clip['id'],'index':clip['index'],'view':clip['view'],
            'source_start_s':clip['source_start_s'],'source_end_s':clip['source_end_s'],
            'clip_sha256':clip['sha256'],'identity':identity,'prompt':prompt,'prompt_sha256':prompt_sha,
            'raw_output':raw,'final_output':final,'answer':answer,'schema_valid':error is None,
            'image_derived_motion_evidence':motion_ref,'native_pose_or_result_given_to_model':False,
            'validation_error':error,'image_grid_thw':grid,'keyframes':keyframes,'output_schema':schema,'processed_frame_pixels':[[v[1]*16,v[2]*16] for v in grid],
            'input_tokens':tokens,'output_tokens':len(ids),'latency_s':time.monotonic()-started,
            'generation':generation,'score_audit':audit.steps,'numerical_error':numerical_error,
            'peak_cuda_memory_bytes':torch.cuda.max_memory_allocated() if device=='cuda' else None,
            'device_allocated_after_generate_bytes':torch.mps.current_allocated_memory() if device=='mps' else torch.cuda.memory_allocated(),'advisory_only':True,'physical_acceptance':False}
        save(target,out);baseline(r)
        print(json.dumps({k:out[k] for k in ['id','schema_valid','latency_s','input_tokens','output_tokens','processed_frame_pixels','peak_cuda_memory_bytes']}),flush=True)
        del x,g
    out=[json.loads(p.read_text()) for p in (r/'responses').glob('*.json')]
    save(r/'progress.json',{'at':now(),'expected':len(m['clips']),'completed':len(out),
        'schema_valid':sum(v['schema_valid'] for v in out),'complete':len(out)==len(m['clips']),
        'frozen_inputs_unchanged':True,'control_output_connected':False})


def main():
    p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='command',required=True)
    a=s.add_parser('prepare');a.add_argument('--video',type=Path,required=True);a.add_argument('--policy',type=Path,required=True)
    a.add_argument('--output',type=Path,required=True);a.add_argument('--ffmpeg',default='ffmpeg')
    a=s.add_parser('infer');a.add_argument('--output',type=Path,required=True);a.add_argument('--model-path',type=Path,required=True)
    a.add_argument('--only',nargs='+');a.add_argument('--max-tokens',type=int,default=2048)
    args=p.parse_args();(prepare if args.command=='prepare' else infer)(args)


if __name__=='__main__':main()
