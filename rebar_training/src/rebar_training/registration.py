"""Lazy Gym registration, callable through Isaac Lab's external_callback."""


def register_tasks():
    import gymnasium as gym

    for task, cfg in (("Grasp", "GraspEnvCfg"), ("Insert", "InsertEnvCfg"), ("Loading", "LoadingEnvCfg")):
        task_id = f"Aubo-Rebar-{task}-Direct-v0"
        if task_id not in gym.registry:
            gym.register(
                id=task_id,
                entry_point="rebar_training.isaac_env:RebarEnv",
                disable_env_checker=True,
                kwargs={
                    "env_cfg_entry_point": f"rebar_training.isaac_cfg:{cfg}",
                    "rsl_rl_cfg_entry_point": "rebar_training.isaac_cfg:" + ("LoadingPPORunnerCfg" if task == "Loading" else "RebarPPORunnerCfg"),
                },
            )
    task_id = 'Aubo-Rebar-Visual-Loading-Direct-v0'
    if task_id not in gym.registry:
        gym.register(id=task_id, entry_point='rebar_training.visual_env:VisualRebarEnv', disable_env_checker=True,
                     kwargs={'env_cfg_entry_point': 'rebar_training.isaac_cfg:VisualLoadingEnvCfg',
                             'rsl_rl_cfg_entry_point': 'rebar_training.isaac_cfg:VisualPPORunnerCfg'})
    task_id='Aubo-Rebar-Mobile-Visual-Direct-v0'
    if task_id not in gym.registry:
        gym.register(id=task_id,entry_point='rebar_training.mobile_env:MobileVisualEnv',disable_env_checker=True,
            kwargs={'env_cfg_entry_point':'rebar_training.isaac_cfg:MobileVisualEnvCfg'})
    return None
