"""Record raw next-token finiteness across local inference backend configurations."""
import argparse
import json
import time
from pathlib import Path
from cosmos_reasoner_replay import MODEL_REVISION, now, save
from diagnose_cosmos_replay import generation_receipt


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--generation-only',action='store_true',help='Match ordinary client calls without a separate prefill probe')
    args=ap.parse_args();r=args.output.resolve();r.mkdir(parents=True,exist_ok=False)
    import torch,transformers
    from PIL import Image
    torch.set_grad_enabled(False);torch.manual_seed(0)
    base=Path(__file__).resolve().parents[2];mp=base/'.cosmos_reasoner_replay/models'/MODEL_REVISION
    src=r.parent/'backend_comparison/comparison.json';receipt=json.loads(src.read_text())
    p=transformers.AutoProcessor.from_pretrained(str(mp),local_files_only=True)
    images=[Image.open(v['path']).convert('RGB') for v in receipt['input_refs']]
    content=[]
    for n in range(len(images)):content.extend([{'type':'text','text':f'Image {n+1} in temporal order: '},{'type':'image'}])
    content.append({'type':'text','text':receipt['prompt']})
    text=p.apply_chat_template([{'role':'user','content':content}],tokenize=False,add_generation_prompt=True,enable_thinking=False)
    xx=p(text=[text],images=images,return_tensors='pt',images_kwargs={'size':{'shortest_edge':4096,'longest_edge':1024*615}})
    n=xx['input_ids'].shape[-1]
    model=transformers.AutoModelForImageTextToText.from_pretrained(str(mp),dtype=torch.bfloat16,attn_implementation='sdpa',local_files_only=True).to('mps').eval()
    plan=[('bf16_sdpa_1',torch.bfloat16,'sdpa',True),('bf16_sdpa_2',torch.bfloat16,'sdpa',True),('bf16_no_cache',torch.bfloat16,'sdpa',False),('bf16_eager',torch.bfloat16,'eager',True),('fp32_sdpa_1',torch.float32,'sdpa',True),('fp32_sdpa_2',torch.float32,'sdpa',True)]
    if args.generation_only:
        plan=[(f'bf16_generate_{i}',torch.bfloat16,'sdpa',True) for i in range(1,4)]+[('fp32_generate',torch.float32,'sdpa',True)]
    for name,dtype,attn,cache in plan:
        model.to(dtype=dtype);model.set_attn_implementation(attn)
        x={k:(v.to(device='mps',dtype=dtype) if v.is_floating_point() else v.to('mps')) for k,v in xx.items()}
        steps=[]
        class Audit(transformers.LogitsProcessor):
            def __call__(self,input_ids,scores):
                finite=torch.isfinite(scores);bad=int((~finite).sum().item())
                record={'step':len(steps),'nonfinite':bad,'nan':int(torch.isnan(scores).sum().item()),'argmax':int(scores[0].argmax().item()),'generated_prefix':input_ids[0,n:].tolist()}
                steps.append(record)
                if bad:raise FloatingPointError('Non-finite raw generation scores; inference rejected')
                return scores
        t=time.monotonic();error=None;ids=[]
        try:
            with torch.inference_mode():
                # Match the earlier backend comparison's explicit prefill before generation.
                first=None;prefill_finite=None
                if not args.generation_only:
                    first=model(**x,logits_to_keep=1,use_cache=False).logits
                    prefill_finite=bool(torch.isfinite(first).all().item())
                g=model.generate(**x,do_sample=False,max_new_tokens=32,max_time=90,use_cache=cache,logits_processor=[Audit()])
            ids=g[0,n:].tolist()
        except FloatingPointError as exc:error=str(exc)
        elapsed=time.monotonic()-t
        out={'at':now(),'name':name,'dtype':str(dtype),'attention':attn,'use_cache':cache,'separate_prefill_probe':not args.generation_only,'prefill_finite':prefill_finite,'error':error,'steps':steps,'latency_s':elapsed,**generation_receipt(ids,p.tokenizer,False,32,elapsed,90)}
        save(r/(name+'.json'),out);print(json.dumps({k:out[k] for k in ['name','error','final_answer','latency_s']}),flush=True)
        del x,first
    save(r/'scope.json',{'at':now(),'inputs':receipt['input_refs'],'prompt':receipt['prompt'],'simulation_started':False,'optimizer_loaded':False,'policy_loaded':False})


if __name__=='__main__':main()
