from aubo_control_gui.presets import PresetStore
def test_presets_survive_restart(tmp_path):
    path=tmp_path/"positions.json"; store=PresetStore(path); store.set("放置位",[1,2,3,4,5,6]); assert PresetStore(path).get("放置位")==[1,2,3,4,5,6]

def test_corrupt_presets_are_not_motion_targets(tmp_path):
    import pytest
    path=tmp_path/'positions.json'
    path.write_text('{"放置位":[NaN,0,0,0,0,0],"存储位 1":[1,2,3,4,5,6]}')
    store=PresetStore(path)
    assert store.get('放置位') is None
    assert store.get('存储位 1')==[1,2,3,4,5,6]
    with pytest.raises(ValueError):store.set('放置位',[float('inf')]*6)
