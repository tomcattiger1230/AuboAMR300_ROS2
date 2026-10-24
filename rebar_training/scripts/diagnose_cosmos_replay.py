"""Controlled, image-only Cosmos integration ablations; never imports control code."""
import argparse
import json
import time
from pathlib import Path

from cosmos_reasoner_replay import MODEL_REVISION, POLICY_SHA, VIDEO_SHA, now, save, sha256
from cosmos_reasoner_split_replay import constraint_prefix, constraint_token_data, prompt_for, schema_for
from cosmos_generation_audit import generation_receipt, GenerationScoreAudit, GenerationNumericsError


def cases():
    wrist = 'Are the two large white gripper fingers in the foreground visibly closed around the dark vertical rod, or separated from it? Describe the visible gap or contact in one sentence.'
    motion = 'Compare the white mobile robot chassis carrying the arm in these ordered images. Does the chassis translate or rotate relative to the fixed gray machine pedestal? Describe its position and orientation in each image, then answer moving, stationary, or uncertain.'
    label = wrist + ' Return JSON with exactly one field contact: closed, separated, or uncertain.'
    contact_schema = {'type': 'object', 'properties': {'contact': {'type': 'string', 'enum': ['closed', 'separated', 'uncertain']}}, 'required': ['contact'], 'additionalProperties': False}
    old = {'view': 'wrist', 'source_start_s': 200, 'source_end_s': 220}
    return [
        dict(id='wrist_old_free', view='wrist', index=10, prompt=prompt_for(old), thinking=False),
        dict(id='wrist_old_constrained', view='wrist', index=10, prompt=prompt_for(old), thinking=False, schema=schema_for('wrist')),
        dict(id='wrist_short_free', view='wrist', index=10, prompt=wrist, thinking=False),
        dict(id='wrist_short_json_free', view='wrist', index=10, prompt=label, thinking=False),
        dict(id='wrist_short_json_constrained', view='wrist', index=10, prompt=label, thinking=False, schema=contact_schema),
        dict(id='wrist_short_thinking', view='wrist', index=10, prompt=wrist, thinking=True),
        dict(id='overview_moving_free', view='overview', index=10, prompt=motion, thinking=False),
        dict(id='overview_moving_thinking', view='overview', index=10, prompt=motion, thinking=True),
        dict(id='overview_moving_roi', view='overview', index=10, prompt=motion, thinking=False, roi=[600,90,1010,390]),
        dict(id='overview_static_free', view='overview', index=6, prompt=motion, thinking=False),
        dict(id='overview_duplicate_control', view='overview', index=10, prompt=motion, thinking=False, duplicate=True),
        dict(id='wrist_blank_control', view='wrist', index=10, prompt=wrist, thinking=False, blank=True),
    ]


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--only',nargs='+')
    ap.add_argument('--cases',type=Path,help='Explicit saved ablation plan; preserve all outcomes')
    ap.add_argument('--dtype',choices=['bfloat16','float32'],default='bfloat16')
    ap.add_argument('--attention',choices=['sdpa','eager'],default='sdpa')
    args=ap.parse_args()
    import torch
    import transformers
    from PIL import Image
    base=Path(__file__).resolve().parents[2];r=args.output.resolve();s=args.source.resolve()
    r.mkdir(parents=True,exist_ok=True);(r/'responses').mkdir(exist_ok=True);(r/'inputs').mkdir(exist_ok=True)
    mp=base/'.cosmos_reasoner_replay/models'/MODEL_REVISION
    acceptance=base/'rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022'
    frozen={name:sha256(acceptance/name) for name in ['full_policy_demo.mp4','frozen_policy.pt']}
    assert frozen=={'full_policy_demo.mp4':VIDEO_SHA,'frozen_policy.pt':POLICY_SHA}
    mm=json.loads((mp/'download_manifest.json').read_text())
    for f in mm['files']:assert sha256(mp/f['name'])==f['actual_sha256']
    dtype=getattr(torch,args.dtype);device='mps'
    p=transformers.AutoProcessor.from_pretrained(str(mp),local_files_only=True)
    model,loading=transformers.AutoModelForImageTextToText.from_pretrained(str(mp),dtype=dtype,
        attn_implementation=args.attention,local_files_only=True,output_loading_info=True)
    save(r/'loading_info.json',json.loads(json.dumps(loading,default=lambda v:sorted(v) if isinstance(v,set) else str(v))))
    assert not loading.get('missing_keys') and not loading.get('mismatched_keys') and not loading.get('error_msgs')
    model.to(device).eval();torch.set_grad_enabled(False);torch.manual_seed(0)
    manifest=json.loads((s/'manifest.json').read_text())
    config={'at':now(),'script_sha256':sha256(Path(__file__)),'model_revision':MODEL_REVISION,
            'class':type(model).__name__,'dtype':args.dtype,'device':device,'attention':args.attention,
            'torch':torch.__version__,'transformers':transformers.__version__,
            'generation_config':model.generation_config.to_dict(),'frozen_before':frozen,
            'policy_loaded':False,'optimizer_loaded':False,'simulation_started':False,
            'tracker_or_native_answer_labels_supplied':False,'total_parameters':sum(v.numel() for v in model.parameters())}
    save(r/'execution.json',config)
    tokdata=constraint_token_data(p.tokenizer)
    plan=json.loads(args.cases.read_text()) if args.cases else cases()
    save(r/'case_plan.json',plan)
    for case in plan:
        if args.only and case['id'] not in args.only:continue
        target=r/'responses'/(case['id']+'.json')
        assert not target.exists(),f'Preserve previous result: {target}'
        clip=next(c for c in manifest['clips'] if c['view']==case['view'] and c['index']==case['index'])
        images=[];refs=[]
        if case['view']=='wrist':
            for f in clip['raw_wrist_frames']:
                path=s/f['path'];assert sha256(path)==f['sha256'];images.append(Image.open(path).convert('RGB'));refs.append(f)
        else:
            decoded=None
            for n in case.get('keyframe_indices',[0,40,79]):
                path=s/'keyframes'/f"overview_{case['index']:03d}_{n:03d}.png"
                if not path.exists():
                    import av
                    if decoded is None:
                        with av.open(str(s/clip['path'])) as c:decoded=[v.to_image() for v in c.decode(video=0)]
                    path=r/'inputs'/f"overview_{case['index']:03d}_{n:03d}.png";decoded[n].save(path)
                images.append(Image.open(path).convert('RGB'));refs.append({'path':str(path),'sha256':sha256(path),'relative_time_s':n/4})
        if case.get('roi'):images=[im.crop(case['roi']) for im in images]
        if case.get('duplicate'):images=[images[0].copy() for _ in images]
        if case.get('blank'):images=[Image.new('RGB',im.size,(128,128,128)) for im in images]
        if case.get('montage'):
            selected=[images[0],images[-1]]
            montage=Image.new('RGB',(sum(im.width for im in selected),max(im.height for im in selected)),(128,128,128))
            left=0
            for im in selected:montage.paste(im,(left,0));left+=im.width
            images=[montage]
        content=[];used=[]
        for n,im in enumerate(images):
            path=r/'inputs'/f"{case['id']}_{n}.png";im.save(path);used.append({'path':str(path),'sha256':sha256(path)})
            heading=(f"Raw wrist frame at {refs[n]['source_time_s']-clip['source_start_s']:.2f} seconds: "
                     if case.get('legacy_heading') else f'Image {n+1} in temporal order: ')
            content.extend([{'type':'text','text':heading},{'type':'image'}])
        content.append({'type':'text','text':case['prompt']})
        text=p.apply_chat_template([{'role':'user','content':content}],tokenize=False,add_generation_prompt=True,enable_thinking=case['thinking'])
        if case.get('video'):
            import av,numpy as np
            assert sha256(s/clip['path'])==clip['sha256']
            with av.open(str(s/clip['path'])) as c:frames=[v.to_image() for v in c.decode(video=0)]
            if case.get('roi'):frames=[im.crop(case['roi']) for im in frames]
            original_count=len(frames);indices=list(range(original_count))
            if case.get('video_limit'):
                indices=np.linspace(0,original_count-1,case['video_limit']).round().astype(int).tolist()
                frames=[frames[i] for i in indices]
            arr=np.stack([np.array(im.convert('RGB')) for im in frames])
            metadata={'total_num_frames':original_count,'fps':4,'frames_indices':indices,
                      'duration':original_count/4,'height':arr.shape[1],'width':arr.shape[2],'video_backend':'pyav_explicit'}
            text=p.apply_chat_template([{'role':'user','content':[{'type':'video'},{'type':'text','text':case['prompt']}]}],tokenize=False,add_generation_prompt=True,enable_thinking=case['thinking'])
            x=p(text=[text],videos=[arr],return_tensors='pt',videos_kwargs={'video_metadata':[metadata],'do_sample_frames':False,'size':{'shortest_edge':4096,'longest_edge':len(arr)*410*300}}).to(device)
            used=[{'path':str(s/clip['path']),'sha256':clip['sha256'],'metadata':metadata,'roi':case.get('roi')}]
        else:
            x=p(text=[text],images=images,return_tensors='pt',images_kwargs={'size':{'shortest_edge':4096,'longest_edge':1024*615}}).to(device)
        x={k:(v.to(dtype) if v.is_floating_point() else v) for k,v in x.items()}
        n=x['input_ids'].shape[-1];kw={}
        grid=x.get('image_grid_thw',x.get('video_grid_thw'))
        patches=int(grid.prod(-1).sum().item())
        save(r/'input_preflight.json',{'id':case['id'],'grid':grid.tolist(),'vision_patches':patches,'language_tokens':n})
        assert patches<=8192 and n<=3500,'Conservative local MPS prefill budget exceeded; no model inference started'
        if case.get('schema'):kw['prefix_allowed_tokens_fn']=constraint_prefix(tokdata,case['schema'])
        maximum=4096 if case['thinking'] else 512;deadline=180 if case['thinking'] else 90
        audit=GenerationScoreAudit(n,allow_masked=bool(case.get('schema')))
        started=time.monotonic();error=None;g=None;ids=[]
        try:
            with torch.inference_mode():g=model.generate(**x,do_sample=False,max_new_tokens=maximum,max_time=deadline,logits_processor=[audit],**kw)
            ids=g[0,n:].tolist()
        except GenerationNumericsError as exc:error=str(exc)
        elapsed=time.monotonic()-started
        out=dict(case,at=now(),source_refs=refs,actual_images=used,prompt_rendered=text,
                 image_grid_thw=x.get('image_grid_thw',x.get('video_grid_thw')).tolist(),input_tokens=n,
                 latency_s=elapsed,max_new_tokens=maximum,deadline_s=deadline,
                 numerical_error=error,score_audit=audit.steps,
                 **generation_receipt(ids,p.tokenizer,case['thinking'],maximum,elapsed,deadline))
        if error:out['generation_status']='runtime_error'
        save(target,out);print(json.dumps({k:out[k] for k in ['id','latency_s','generated_tokens','answer_complete','final_answer','token_frequency']}),flush=True)
        del x,g
    frozen_after={name:sha256(acceptance/name) for name in frozen};assert frozen==frozen_after
    save(r/'frozen_after.json',{'at':now(),'sha256':frozen_after,'unchanged':True})


if __name__=='__main__':main()
