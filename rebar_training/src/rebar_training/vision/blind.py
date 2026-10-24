"""Runtime tripwire against privileged steel/station/reward state in control."""
from contextlib import contextmanager


class ForbiddenTruth:
    def __init__(self, label):
        object.__setattr__(self, '_label', label)

    def __getattr__(self, name):
        raise RuntimeError(f'Privileged truth access in visual control: {self._label}.{name}')

    def __call__(self, *args, **kwargs):
        raise RuntimeError(f'Privileged truth access in visual control: {self._label}()')


@contextmanager
def blinded_control(env):
    """Block both direct assets and the scene lookup while control executes.

    Reset, rendering backend physics, labels, reward and evaluator run outside
    this context. The controller cannot inspect steel state or reward phases.
    """
    names = ('bar', 'station', 'logic', 'sampler', 'visual_truth_for_evaluation',
             '_last_result', '_filtered_force')
    attributes = {name: getattr(env,name) for name in names if hasattr(env,name)}
    objects = env.scene.rigid_objects
    assets = {name:objects[name] for name in ('rebar','station') if name in objects}
    try:
        for name in attributes:
            setattr(env,name,ForbiddenTruth(name))
        for name in assets:
            objects[name] = ForbiddenTruth('scene.'+name)
        yield
    finally:
        for name,value in attributes.items():
            setattr(env,name,value)
        for name,value in assets.items():
            objects[name] = value
