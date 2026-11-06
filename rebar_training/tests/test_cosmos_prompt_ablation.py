"""Keep runtime, output contracts and semantic scoring distinct."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from cosmos_prompt_ablation import strict_motion


def test_no_coordinate_arrays_or_prose_are_coerced_into_json_labels():
    for text in ('[{"translation_2d":[0,0]}]', 'No.', '{"translation":"no","extra":1}',
                 '{"translation":"no"', '{"translation":"stationary"}'):
        assert strict_motion(text, 'translation', 'single_json') is None


def test_explicit_uncertainty_is_a_valid_contract_but_not_the_negative_class():
    assert strict_motion('{"translation":"uncertain"}', 'translation', 'single_json') == 'uncertain'


def test_prose_accepts_an_explicit_prefix_without_inventing_hidden_motion():
    assert strict_motion('No. Only the arm moves.', 'translation', 'single_prose') == 'no'
    assert strict_motion('The base might be moving.', 'translation', 'single_prose') is None


def test_baseline_requires_both_motion_fields_and_evidence():
    assert strict_motion('{"translation":"yes","rotation":"no","visible_evidence":"floor reference"}',
                         'translation', 'baseline') == 'yes'
    assert strict_motion('{"translation":"yes","rotation":"no"}', 'translation', 'baseline') is None
