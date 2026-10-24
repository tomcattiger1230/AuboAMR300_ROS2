"""Compare identical original-image requests on MPS BF16 and CPU FP32, offline."""
import argparse
import json
import time
from pathlib import Path

from cosmos_reasoner_replay import MODEL_REVISION, now, save, sha256
from diagnose_cosmos_replay import generation_receipt


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();r=args.output.resolve();r.mkdir(exist_ok=False,parents=True)
    import torch,transformers
    from PIL import Image
    torch.set_num_threads(8);torch.set_grad_enabled(False)
    base=Path(__file__).resolve().parents[2];mp=base/'.cosmos_reasoner_replay/models'/MODEL_REVISION
    s=base/'rebar_training/artifacts/cosmos_reasoner_split_replay_20261024'
    p=transformers.AutoProcessor.from_pretrained(str(mp),local_files_only=True)
    prompt='Are the two large white gripper fingers in the foreground visibly closed around the dark vertical rod, or separated from it? Describe the visible gap or contact in one sentence. Return JSON with exactly one field contact: closed, separated, or uncertain.'
    paths=[s/'raw_wrist'/f'wrist_step{step}.png' for step in [6000,6300]]
    images=[Image.open(path).convert('RGB') for path in paths]
    content=[]
    for n in range(len(images)):content.extend([{'type':'text','text':f'Image {n+1} in temporal order: '},{'type':'image'}])
    content.append({'type':'text','text':prompt})
    text=p.apply_chat_template([{'role':'user','content':content}],tokenize=False,add_generation_prompt=True,enable_thinking=False)
    x=p(text=[text],images=images,return_tensors='pt',images_kwargs={'size':{'shortest_edge':4096,'longest_edge':1024*615}})
    n=x['input_ids'].shape[-1];logits=[];out=[]
    model=transformers.AutoModelForImageTextToText.from_pretrained(str(mp),dtype=torch.bfloat16,attn_implementation='sdpa',local_files_only=True).eval()
    for name,device,dtype in [('mps_bf16','mps',torch.bfloat16),('cpu_fp32','cpu',torch.float32)]:
        model.to(device=device,dtype=dtype)
        xx={k:(v.to(device=device,dtype=dtype) if v.is_floating_point() else v.to(device)) for k,v in x.items()}
        t=time.monotonic()
        with torch.inference_mode():
            a=model(**xx,logits_to_keep=1,use_cache=False).logits[0,-1].float().cpu()
            g=model.generate(**xx,do_sample=False,max_new_tokens=32,max_time=180)
        elapsed=time.monotonic()-t;logits.append(a);ids=g[0,n:].tolist()
        item=dict(name=name,latency_s=elapsed,top10_logits=[{'id':int(i),'text':p.tokenizer.decode([int(i)]),'logit':float(a[i])} for i in a.topk(10).indices],**generation_receipt(ids,p.tokenizer,False,32,elapsed,180))
        out.append(item);save(r/(name+'.json'),item);print(json.dumps({'name':name,'elapsed':elapsed,'final':item['final_answer']}),flush=True)
        del xx,g
        if device=='mps':torch.mps.empty_cache()
    save(r/'comparison.json',{'at':now(),'prompt':prompt,'input_refs':[{'path':str(path),'sha256':sha256(path)} for path in paths],
        'input_grid':x['image_grid_thw'].tolist(),'input_tokens':n,
        'first_token_logit_cosine':float(torch.nn.functional.cosine_similarity(logits[0],logits[1],dim=0)),
        'first_token_max_abs_difference':float((logits[0]-logits[1]).abs().max()),
        'same_generated_ids':out[0]['token_ids']==out[1]['token_ids'],'same_first_token':int(logits[0].argmax())==int(logits[1].argmax()),
        'simulation_started':False,'optimizer_loaded':False,'policy_loaded':False})


if __name__=='__main__':main()
