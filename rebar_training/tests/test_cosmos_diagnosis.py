import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from diagnose_cosmos_replay import generation_receipt
from cosmos_generation_audit import GenerationScoreAudit, GenerationNumericsError, validate_local_prefill


class Tokens:
    def decode(self, ids, skip_special_tokens):
        return ''.join({1:'reasoning', 2:'</think>answer', 11:'' if skip_special_tokens else '<|im_end|>'}[i] for i in ids)


class CompletionContract(unittest.TestCase):
    def test_unfinished_thinking_is_not_an_answer(self):
        r=generation_receipt([1], Tokens(), True, 1, 1, 90)
        self.assertFalse(r['answer_complete'])
        self.assertIsNone(r['final_answer'])
        self.assertTrue(r['limit_reached'])

    def test_eos_without_thinking_close_is_incomplete(self):
        r=generation_receipt([1,11], Tokens(), True, 4096, 1, 90)
        self.assertFalse(r['answer_complete'])

    def test_completed_answer_and_raw_control_token_are_preserved(self):
        r=generation_receipt([1,2,11], Tokens(), True, 4096, 1, 90)
        self.assertEqual(r['final_answer'],'answer')
        self.assertTrue(r['answer_complete'])
        self.assertIn('<|im_end|>',r['raw_with_special_tokens'])

    def test_time_limit_is_not_success(self):
        r=generation_receipt([1], Tokens(), False, 512, 91, 90)
        self.assertTrue(r['time_limit_reached'])
        self.assertIsNone(r['final_answer'])


class NumericalContract(unittest.TestCase):
    def test_observed_80_frame_resource_failure_is_rejected_before_inference(self):
        with self.assertRaises(ValueError):validate_local_prefill([[80,18,26]],9500)

    def test_bounded_same_interval_video_is_within_local_budget(self):
        self.assertEqual(validate_local_prefill([[12,18,26]],1570)['vision_patches'],5616)

    def test_all_nan_is_rejected_before_argmax_can_emit_unknown(self):
        import torch
        audit=GenerationScoreAudit(2)
        with self.assertRaises(GenerationNumericsError):
            audit(torch.tensor([[1,2,3]]),torch.full((1,5),float('nan')))
        self.assertEqual(audit.steps[-1]['nan'],5)

    def test_schema_masked_negative_infinity_is_legal(self):
        import torch
        audit=GenerationScoreAudit(2,allow_masked=True)
        scores=torch.tensor([[float('-inf'),1.]])
        self.assertIs(audit(torch.tensor([[1,2]]),scores),scores)

    def test_empty_or_positive_infinite_distribution_is_rejected(self):
        import torch
        for values in [[float('-inf')]*3,[1.,float('inf')]]:
            with self.assertRaises(GenerationNumericsError):
                GenerationScoreAudit(2,allow_masked=True)(torch.tensor([[1,2]]),torch.tensor([values]))


if __name__=='__main__':unittest.main()
