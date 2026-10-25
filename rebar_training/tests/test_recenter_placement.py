import ast
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np


def test_recenter_requires_extra_planar_margin_before_release():
    src=Path(__file__).parents[1]/'src/rebar_training/workcell_sim_env.py';tree=ast.parse(src.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='WorkcellControl')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='settled_at')
    scope={'np':np,'math':math};exec(compile(ast.Module(body=[method],type_ignores=[]),str(src),'exec'),scope)
    q=np.array([0.,1.,0.,0.]);c=SimpleNamespace(valid=[True],packets=[SimpleNamespace(quaternion_xyzw=q)],
        observers=[SimpleNamespace(position=np.array([.2927,.003,.715]))],stage='tray_down',stage_dwell=0,transport_resume=None)
    for _ in range(9):passed=scope['settled_at'](c,np.array([.2927,0.,.717]),q)
    assert passed  # Original 4mm behavior is preserved outside recovery.
    c.transport_resume={'index':0};c.stage_dwell=0
    for _ in range(20):assert not scope['settled_at'](c,np.array([.2927,0.,.717]),q)
    c.observers[0].position[1]=.001
    for _ in range(9):passed=scope['settled_at'](c,np.array([.2927,0.,.717]),q)
    assert passed
