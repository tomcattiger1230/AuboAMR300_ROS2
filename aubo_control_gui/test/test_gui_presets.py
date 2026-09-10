import math
from types import SimpleNamespace
from aubo_control_gui.app import MainWindow
from aubo_control_gui.presets import PresetStore

def test_run_preset_dispatches_updated_saved_values(tmp_path):
    store=PresetStore(tmp_path/'positions.json')
    store.set('放置位',[1,2,3,4,5,6])
    store.set('放置位',[6,5,4,3,2,1])
    calls=[]
    window=SimpleNamespace(
        presets=store,actual=[0.]*6,velocity=SimpleNamespace(value=lambda:.2),
        acceleration=SimpleNamespace(value=lambda:.2),log=lambda *args:None,
        bridge=SimpleNamespace(node=SimpleNamespace(fresh=lambda:True),
            execute_with_fallback=lambda *args:calls.append(args)))
    MainWindow.run_preset(window,'放置位')
    assert calls[0][1]==[math.radians(v) for v in [6,5,4,3,2,1]]
    window.bridge.node.fresh=lambda:False
    MainWindow.run_preset(window,'放置位')
    assert len(calls)==1
