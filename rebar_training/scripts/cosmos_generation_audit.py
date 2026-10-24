"""Generation integrity checks, independent of semantic classifications."""
import collections


class GenerationNumericsError(FloatingPointError):
    pass


def validate_local_prefill(grid, text_tokens):
    """Conservative bound for this Mac's diagnosed FP32 replay path."""
    patches=sum(t*h*w for t,h,w in grid)
    if patches>8192 or text_tokens>3500:
        raise ValueError('Local MPS prefill budget exceeded; split or explicitly sample input before inference')
    return {'vision_patches':patches,'language_tokens':text_tokens}


class GenerationScoreAudit:
    """Reject NaN/+inf and empty distributions; permit schema-masked -inf."""
    def __init__(self, input_tokens, allow_masked=False):
        self.input_tokens=input_tokens
        self.allow_masked=allow_masked
        self.steps=[]

    def __call__(self, input_ids, scores):
        nan=int(scores.isnan().sum().item())
        positive_inf=int(scores.isposinf().sum().item())
        finite=int(scores.isfinite().sum().item())
        invalid=nan or positive_inf or finite==0 or (not self.allow_masked and finite!=scores.numel())
        item={'step':len(self.steps),'nan':nan,'positive_inf':positive_inf,'finite':finite}
        if invalid:item['generated_prefix']=input_ids[0,self.input_tokens:].tolist()
        self.steps.append(item)
        if invalid:raise GenerationNumericsError('Invalid generation scores; no semantic answer accepted')
        return scores


def generation_receipt(ids, tokenizer, thinking, maximum, elapsed, deadline):
    raw=tokenizer.decode(ids,skip_special_tokens=False)
    clean=tokenizer.decode(ids,skip_special_tokens=True)
    eos=bool(ids and ids[-1]==getattr(tokenizer,'eos_token_id',11))
    unknown=ids.count(getattr(tokenizer,'unk_token_id',0))
    closed='</think>' in clean
    complete=eos and not unknown and (not thinking or closed)
    return {'token_ids':ids,'raw_with_special_tokens':raw,'decoded':clean,
            'generated_tokens':len(ids),'eos_reached':eos,'unknown_tokens':unknown,
            'limit_reached':len(ids)>=maximum,
            'time_limit_reached':not eos and elapsed>=deadline,
            'thinking_closed':closed,'answer_complete':complete,
            'generation_status':'complete' if complete else 'incomplete',
            'final_answer':clean.rsplit('</think>',1)[-1].strip() if complete else None,
            'token_frequency':collections.Counter(ids).most_common(10)}
