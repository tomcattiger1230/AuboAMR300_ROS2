import torch
import pytest
from rebar_training.mobile_residual import (ResidualPPO,FIELDS,OBS_SIZE,gae,masked_policy_loss,
    pack_observation,progress_potential,residual_reward,DISCOUNT,PHASES,precision_residual_gain)


def precision_observation(error,stage='loading',route=7):
    fields={name:[0.]*width for name,width in FIELDS}
    fields['phase'][PHASES.index(stage)]=1.;fields['route'][route]=1.
    # TCP is 8 mm from the visually estimated bar; goal-bar error is 'error'.
    fields['bar_minus_tcp']=[0.,-.008,0.]
    fields['machine_goal_minus_tcp']=[-error/5,-.008/5,0.]
    return pack_observation(fields)


def test_precision_envelope_removes_residual_equilibrium_without_relaxing_alignment():
    from types import SimpleNamespace
    from rebar_training.spec import TaskSpec
    from rebar_training.vision.progress import VisualProgress
    spec=TaskSpec(task='loading');goal=torch.tensor(spec.insert_goal).reshape(1,3)
    def rollout(taper):
        logic=VisualProgress(spec,1,'cpu');logic.pick_complete[:]=True;logic.phase[:]=1
        logic.alignment_tolerance=.0008;error=.001
        for _ in range(90):
            gain=precision_residual_gain(precision_observation(error)) if taper else 1.
            # Feedback gain is 4/s; a fixed 4 mm/s residual equilibrates at 1 mm.
            # This is a servo mathematics regression, not a physical simulation.
            error+=(-4*error+.004*gain)/30
            bar=goal+torch.tensor([[error,0.,0.]])
            state=SimpleNamespace(bar=bar,tcp=bar+torch.tensor([[0.,.008,0.]]),
                finger_forces=torch.full((1,2),50.),fingers=torch.full((1,2),.016),
                linear_velocity=torch.zeros(1,3),angular_velocity=torch.zeros(1,3),
                bar_axis=torch.tensor([[0.,0.,1.]]),jaw_forces=torch.zeros(1,4))
            logic.advance(state,torch.zeros(1,7),torch.tensor([True]))
        return error,int(logic.phase[0]),logic.alignment_tolerance
    old_error,old_phase,_=rollout(False);new_error,new_phase,tolerance=rollout(True)
    assert old_error==pytest.approx(.001) and old_phase==1
    assert abs(new_error)<tolerance==.0008 and new_phase==2


@pytest.mark.parametrize('stage,route',[('pickup',7),('pickup_tray',7),('loading',6),('loading',8),('loading',9),('tray_release',7)])
def test_precision_envelope_preserves_other_manipulation_and_release_commands(stage,route):
    assert precision_residual_gain(precision_observation(.001,stage,route))==1.


def test_precision_envelope_preserves_far_actions_and_measures_goal_minus_bar():
    assert precision_residual_gain(precision_observation(.03))==1.
    assert precision_residual_gain(precision_observation(0.))<1e-12
    fields={name:[0.]*width for name,width in FIELDS}
    fields['phase'][PHASES.index('loading')]=1.;fields['route'][7]=1.
    assert precision_residual_gain(pack_observation(fields))==0.
    assert precision_residual_gain(precision_observation(.001))==pytest.approx(.0025,rel=1e-5)
    x=precision_observation(.001);x[0]=float('nan')
    with pytest.raises(ValueError,match='observation'):precision_residual_gain(x)


def test_override_samples_have_zero_policy_gradient_and_do_not_contaminate_active_loss():
    logratio=torch.tensor([.1,20.],requires_grad=True)
    loss=masked_policy_loss(logratio,torch.tensor([1.,-100.]),torch.tensor([True,False]))
    loss.backward()
    assert logratio.grad[0]!=0 and logratio.grad[1]==0
    assert float(loss.detach())==pytest.approx(-torch.exp(torch.tensor(.1)).item())


def test_advantages_stop_at_resets_and_terminal_value_never_leaks():
    advantage,_=gae(torch.tensor([1.,2.]),torch.zeros(2),torch.tensor([999.,0.]),
                    torch.tensor([True,False]),gamma=.99,lam=.95)
    torch.testing.assert_close(advantage,torch.tensor([1.,2.]))


def test_residual_resume_keeps_contract_scale_optimizer_and_actual_updates(tmp_path):
    agent=ResidualPPO(seed=8)
    obs=[0.]*OBS_SIZE
    assert agent.act(obs)[0].abs().max()==0
    before=agent.model.actor[-1].weight.detach().clone()
    for i in range(32):
        action,transition=agent.act(obs,True)
        agent.record(transition,float(action[0,0]),obs,i==31,True)
    receipt=agent.update()
    assert receipt['active_steps']==32 and receipt['update']==1
    assert not torch.equal(before,agent.model.actor[-1].weight)
    file=tmp_path/'policy.pt';agent.save(file)
    loaded=ResidualPPO(checkpoint=file)
    torch.testing.assert_close(agent.act(obs)[0],loaded.act(obs)[0])
    assert loaded.updates==1 and loaded.optimizer.state_dict()['state']
    assert loaded.actor_updates==1
    with pytest.raises(ValueError,match='scale'):
        ResidualPPO(checkpoint=file,scale=.1)


def test_observation_contract_rejects_missing_and_nonfinite_values():
    fields={name:[0.]*width for name,width in FIELDS}
    assert len(pack_observation(fields))==OBS_SIZE
    fields['bar_axis'][0]=float('nan')
    with pytest.raises(ValueError,match='bar_axis'):pack_observation(fields)


def test_discounted_shaping_cannot_farm_progress_or_leak_the_next_reset():
    def discounted_return(potentials):
        return sum(DISCOUNT**i*residual_reward(potentials[i],potentials[i+1],
            done=i==len(potentials)-2) for i in range(len(potentials)-1))
    # Same sparse outcome and horizon: wandering through high-progress states
    # cannot beat staying still. The terminal next value belongs to a reset.
    idle=discounted_return([.1,.1,.1,.1,.9])
    farming=discounted_return([.1,.9,.2,.95,.3])
    assert idle==pytest.approx(farming,abs=1e-9)
    assert residual_reward(.8,.0,done=True)==residual_reward(.8,1.,done=True)
    assert residual_reward(.8,1.,done=True)<-40
    assert residual_reward(.8,.0,done=True,success=True)>0


def test_visual_potential_is_bounded_and_not_defined_from_unobserved_steel():
    from rebar_training.mobile_residual import PHASES
    assert progress_potential((('pickup',0),100.),False)==0
    for stage in PHASES:
        for route in range(11):
            for error in (0.,.01,100.):
                assert 0<=progress_potential(((stage,route),error))<=1
    with pytest.raises(ValueError,match='metric'):progress_potential((('pickup',0),float('nan')))


def test_critic_only_update_cannot_change_actor_or_its_adam_momentum():
    import copy
    agent=ResidualPPO(seed=42);obs=[.1]*OBS_SIZE
    for i in range(32):
        action,transition=agent.act(obs,True)
        agent.record(transition,float(action[0,0]),obs,True,True)
    agent.update()
    parameters=[*agent.model.actor.parameters(),agent.model.log_std]
    before=[p.detach().clone() for p in parameters]
    optimizer_before=[copy.deepcopy(agent.optimizer.state[p]) for p in parameters]
    critic_before=[p.detach().clone() for p in agent.model.critic.parameters()]
    for _ in range(32):
        _,transition=agent.act(obs,True)
        agent.record(transition,10.,obs,True,False)
    update=agent.update()
    assert update['active_steps']==0 and agent.actor_updates==1
    for p,value,optimizer in zip(parameters,before,optimizer_before):
        torch.testing.assert_close(p,value,rtol=0,atol=0)
        for key,state in optimizer.items():torch.testing.assert_close(agent.optimizer.state[p][key],state,rtol=0,atol=0)
    assert any(not torch.equal(p,value) for p,value in zip(agent.model.critic.parameters(),critic_before))
