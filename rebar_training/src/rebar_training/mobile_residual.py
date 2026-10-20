"""Bounded residual PPO for visual manipulation, with an explicit deployment contract.

The RGB servo, phase supervisor, gripper sequencing and odometry navigation are
still part of the deployed controller. This is not a standalone whole-mission
actor. Observation/acquisition and perception-loss holds are excluded from the
policy gradient, rather than pretending their overridden actions were executed.
"""
from pathlib import Path
import math
import torch
from torch import nn

CONTRACT = 'mobile_visual_manipulation_residual_v1'
REWARD_CONTRACT = 'mobile_residual_potential_v2'
DISCOUNT = .9995
PHASES = ('pickup','tray_raise','tray_rotate','tray_via_side','tray_via_front',
          'tray_above','tray_down','tray_release','tray_retreat','observe_tray',
          'navigate','dock','docked','pickup_tray','tray_lift_high','loading')
FIELDS = (('q',6),('dq',6),('tcp',3),('tcp_quaternion',4),('bar_minus_tcp',3),
          ('bar_axis',3),('bar_velocity',3),('bar_angular_velocity',3),
          ('fingers',2),('finger_load',2),('jaw_load',4),('tray_load',1),
          ('machine_goal_minus_tcp',3),('image_age',1),('valid',1),
          ('phase',len(PHASES)),('route',11),('base_xy',2),('base_velocity',2))
OBS_SIZE=sum(width for _,width in FIELDS)
ACTION_SIZE=6
PRECISION_ENVELOPE='visual_precision_quadratic_20mm_v1'


def precision_residual_gain(observation):
    """Let measured visual servo error dominate the final insertion alignment.

    A constant learned joint-velocity offset otherwise creates a nonzero servo
    equilibrium, so deterministic evaluation can miss the unchanged 0.8 mm
    control interlock even when noisy training occasionally crosses it. Only
    loading route 7 tapers the residual; farther than 20 mm it is unchanged.
    Both vectors below are already camera/robot measurements in the actor input.
    """
    if len(observation)!=OBS_SIZE or not all(math.isfinite(float(v)) for v in observation):
        raise ValueError('Invalid visual residual observation')
    offset=0;fields={}
    for name,width in FIELDS:
        if name in ('phase','route','machine_goal_minus_tcp','bar_minus_tcp'):
            fields[name]=observation[offset:offset+width]
        offset+=width
    if fields['phase'][PHASES.index('loading')]<.5 or fields['route'][7]<.5:return 1.
    error=[5*float(g)-float(b) for g,b in zip(fields['machine_goal_minus_tcp'],fields['bar_minus_tcp'])]
    return min(1.,sum(v*v for v in error)/(.020**2))


def progress_potential(metric,observed=True):
    """Bounded visual task potential; not an extra per-stage success reward."""
    if not observed:return 0.
    (stage,route),error=metric
    if stage not in PHASES or not 0<=route<=10 or not math.isfinite(error) or error<0:
        raise ValueError('Invalid visual progress metric')
    route_progress=route/10 if stage in ('pickup','pickup_tray','loading') else 0.
    return (PHASES.index(stage)+.5*route_progress+.5*math.exp(-error/.1))/(len(PHASES)+1)


def residual_reward(potential,next_potential,*,done=False,success=False,effort=0.,gamma=DISCOUNT):
    """Potential shaping telescopes, including failures and automatic resets.

    Terminal potential is zero regardless of the next episode's observation.
    The same gamma must be used for PPO's value targets and the shaping term.
    """
    if not all(math.isfinite(x) for x in (potential,next_potential,effort,gamma)):
        raise ValueError('Nonfinite residual reward input')
    if not (0<=potential<=1 and 0<=next_potential<=1 and effort>=0 and 0<gamma<1):
        raise ValueError('Residual reward outside declared bounds')
    if success and not done:raise ValueError('Success requires a terminal receipt')
    terminal=20. if done and success else -40. if done else 0.
    return -.003-.001*effort+terminal+5*(gamma*(0. if done else next_potential)-potential)


def pack_observation(fields):
    """Fail on missing/nonfinite measurements; never silently insert GT values."""
    import numpy as np
    result=[]
    if set(fields)!={name for name,_ in FIELDS}:
        raise ValueError('Residual observation fields do not match the versioned contract')
    for name,width in FIELDS:
        value=np.asarray(fields[name],dtype=np.float32).reshape(-1)
        if value.size!=width or not np.isfinite(value).all():
            raise ValueError('Invalid residual observation: '+name)
        result.extend(value.clip(-10,10).tolist())
    return result


class ResidualActorCritic(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor=nn.Sequential(nn.Linear(OBS_SIZE,128),nn.Tanh(),nn.Linear(128,128),nn.Tanh(),nn.Linear(128,ACTION_SIZE))
        self.critic=nn.Sequential(nn.Linear(OBS_SIZE,128),nn.Tanh(),nn.Linear(128,128),nn.Tanh(),nn.Linear(128,1))
        # Begin at the physically verified visual servo. PPO learns corrections.
        nn.init.zeros_(self.actor[-1].weight);nn.init.zeros_(self.actor[-1].bias)
        self.log_std=nn.Parameter(torch.full((ACTION_SIZE,),-1.6))

    def distribution(self,obs):
        return torch.distributions.Normal(self.actor(obs),self.log_std.clamp(-4.,-.7).exp())

    def forward(self,obs):
        return self.actor(obs).tanh()


def gae(rewards,values,next_values,dones,gamma=.995,lam=.95):
    advantage=torch.zeros_like(rewards);tail=torch.zeros_like(rewards[0])
    for t in reversed(range(len(rewards))):
        live=1-dones[t].float()
        delta=rewards[t]+gamma*next_values[t]*live-values[t]
        tail=delta+gamma*lam*live*tail;advantage[t]=tail
    return advantage,advantage+values


def masked_policy_loss(log_ratio,advantage,active,clip=.2):
    selected=active.bool()
    if not selected.any():return log_ratio.sum()*0
    ratio=log_ratio[selected].exp()
    return -torch.minimum(ratio*advantage[selected],ratio.clamp(1-clip,1+clip)*advantage[selected]).mean()


class ResidualPPO:
    def __init__(self,device='cpu',scale=.08,seed=42,checkpoint=None):
        if not 0<scale<=.15:raise ValueError('Residual joint-command scale must be in (0,.15]')
        torch.manual_seed(seed)
        self.device=device;self.scale=scale
        self.reward_contract=REWARD_CONTRACT;self.gamma=DISCOUNT
        self.model=ResidualActorCritic().to(device)
        self.optimizer=torch.optim.Adam(self.model.parameters(),lr=1e-4)
        self.buffer=[];self.updates=0;self.policy_steps=0;self.history=[]
        self.training_contexts=[]
        self.actor_updates=0
        if checkpoint:
            saved=torch.load(checkpoint,map_location=device,weights_only=False)
            if saved.get('observation_contract')!=CONTRACT or saved.get('fields')!=list(FIELDS):
                raise ValueError('Checkpoint does not match the residual visual observation contract')
            if saved.get('residual_scale')!=scale:
                raise ValueError('Residual scale must match the evaluated/trained checkpoint')
            if saved.get('reward_contract')!=REWARD_CONTRACT or saved.get('gamma')!=DISCOUNT:
                raise ValueError('Checkpoint reward contract differs; start a fresh residual curriculum')
            self.model.load_state_dict(saved['model']);self.optimizer.load_state_dict(saved['optimizer'])
            self.updates=saved['updates'];self.policy_steps=saved['policy_steps']
            self.history=saved.get('history',[])
            self.training_contexts=saved.get('training_contexts',[])
            self.actor_updates=saved.get('actor_updates',0)

    def tensor(self,obs):
        return torch.as_tensor(obs,dtype=torch.float32,device=self.device).reshape(1,OBS_SIZE)

    @torch.no_grad()
    def act(self,obs,learn=False):
        x=self.tensor(obs);distribution=self.model.distribution(x)
        latent=distribution.sample() if learn else distribution.mean
        value=self.model.critic(x).squeeze(-1)
        # PPO ratios use the same untransformed normal sample on both sides;
        # tanh's fixed Jacobian cancels in that ratio.
        logp=distribution.log_prob(latent).sum(-1)
        return latent.tanh(),(x,latent,logp,value)

    def record(self,transition,reward,next_obs,done,active):
        x,latent,logp,value=transition
        with torch.no_grad():nv=self.model.critic(self.tensor(next_obs)).squeeze(-1)
        self.buffer.append((x,latent,logp,value,torch.tensor([reward],device=self.device),nv,
                            torch.tensor([done],device=self.device),torch.tensor([active],device=self.device)))
        self.policy_steps+=int(active)

    @torch.enable_grad()
    def update(self):
        if not self.buffer:return None
        obs,latent,old_logp,values,rewards,next_values,dones,active=[torch.cat(v) for v in zip(*self.buffer)]
        advantages,returns=gae(rewards,values,next_values,dones,gamma=DISCOUNT)
        eligible=active.bool();n=int(eligible.sum())
        if n:
            center=advantages[eligible].mean();spread=advantages[eligible].std(unbiased=False).clamp_min(1e-5)
            advantages=(advantages-center)/spread
        losses=[];kls=[]
        for _ in range(4):
            for ids in torch.randperm(len(obs),device=self.device).split(256):
                critic_loss=(self.model.critic(obs[ids]).squeeze(-1)-returns[ids]).square().mean()
                selected=ids[eligible[ids]]
                loss=.5*critic_loss
                if len(selected):
                    dist=self.model.distribution(obs[selected]);logp=dist.log_prob(latent[selected]).sum(-1)
                    log_ratio=logp-old_logp[selected]
                    actor_loss=masked_policy_loss(log_ratio,advantages[selected],active[selected])
                    entropy=dist.entropy().sum(-1).mean()
                    anchor=self.model.actor(obs[selected]).square().mean()
                    loss=loss+actor_loss-.001*entropy+.02*anchor
                # None gradients also prevent Adam momentum from changing an
                # actor when the entire minibatch was externally overridden.
                self.optimizer.zero_grad(set_to_none=True);loss.backward()
                nn.utils.clip_grad_norm_(self.model.critic.parameters(),.5)
                nn.utils.clip_grad_norm_([*self.model.actor.parameters(),self.model.log_std],.5)
                self.optimizer.step()
                losses.append(float(loss.detach()))
                if len(selected):kls.append(float((old_logp[selected]-logp).mean().detach()))
            if kls and abs(kls[-1])>.02:break
        self.updates+=1;self.buffer=[]
        self.actor_updates+=int(n>0)
        row=dict(update=self.updates,active_steps=n,policy_steps=self.policy_steps,
                 actor_updates=self.actor_updates,
                 loss=sum(losses)/len(losses),approx_kl=sum(kls)/max(1,len(kls)))
        if not all(math.isfinite(v) for v in row.values()):raise RuntimeError('Nonfinite PPO update')
        self.history.append(row)
        return row

    def save(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        temporary=path.with_suffix('.tmp')
        torch.save(dict(observation_contract=CONTRACT,fields=list(FIELDS),residual_scale=self.scale,
            reward_contract=REWARD_CONTRACT,gamma=DISCOUNT,actor_updates=self.actor_updates,
            model=self.model.state_dict(),optimizer=self.optimizer.state_dict(),updates=self.updates,
            policy_steps=self.policy_steps,history=self.history,
            training_contexts=self.training_contexts,
            controller='bounded arm residual PPO over camera-only servo; scripted observation/gripper/odometry',
            standalone_full_mission_policy=False,steel_truth_policy_input=False),temporary)
        temporary.replace(path)
