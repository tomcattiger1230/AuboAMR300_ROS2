import numpy as np
import pytest
import torch
from rebar_training.randomization import profile, perturb_image
from rebar_training.reset import ResetSampler
from rebar_training.spec import GRASP_JOINTS


def test_expanded_reset_is_reproducible_supported_and_within_declared_bounds():
    b=profile('expanded')
    batch=ResetSampler(551042).sample('loading',4096,'expanded')
    repeat=ResetSampler(551042).sample('loading',4096,'expanded')
    torch.testing.assert_close(batch.bar_pose,repeat.bar_pose)
    torch.testing.assert_close(batch.bar_pose[:,:2],batch.station_pose[:,:2])
    torch.testing.assert_close(batch.bar_pose[:,3:],batch.station_pose[:,3:])
    assert (batch.bar_pose[:,2]==.522).all()
    delta=batch.bar_pose[:,:2]-torch.tensor([-1.15,0.])
    assert float(delta.abs().max())<=b.source_xy_m+1e-7
    assert float(delta.abs().max())>.055
    assert float((batch.joints[:,:6]-torch.tensor(GRASP_JOINTS)).abs().max())<=b.arm_joints_rad+1e-6
    assert (torch.linalg.vector_norm(batch.bar_pose[:,3:],dim=1)-1).abs().max()<1e-6
    with pytest.raises(ValueError,match='Unknown randomization'):
        ResetSampler(1).sample('loading',1,'typo')


def test_image_perturbation_does_not_wrap_or_change_shape_and_baseline_is_identical():
    mono=np.array([[0,255],[128,64]],dtype=np.uint8)
    original=mono.copy()
    assert perturb_image(mono,1.,0.,0.,np.random.default_rng(1)) is mono
    result=perturb_image(mono,1.2,8.,2.,np.random.default_rng(7))
    np.testing.assert_array_equal(mono,original)
    assert result.dtype==np.uint8 and result.shape==mono.shape
    assert result[0,1]==255 and result[0,0]<16
