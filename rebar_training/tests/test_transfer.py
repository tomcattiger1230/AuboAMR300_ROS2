import hashlib
import importlib.util
import json
from pathlib import Path
import pytest


def test_transfer_includes_new_mobile_rack_and_rejects_changed_dependencies(tmp_path):
    spec=importlib.util.spec_from_file_location('build_transfer',Path(__file__).parents[1]/'scripts/build_transfer.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rack=tmp_path/'seer_description/urdf/rack.usda';rack.parent.mkdir(parents=True);rack.write_text('rack geometry')
    folder=tmp_path/'rebar_training/artifacts/assets_mobile';folder.mkdir(parents=True)
    (folder/'manifest.json').write_text(json.dumps(dict(source_sha256={},
        rack_source=str(rack.relative_to(tmp_path)),rack_sha256=hashlib.sha256(rack.read_bytes()).hexdigest())))
    assert module.source_assets(tmp_path)==[rack]
    rack.write_text('changed geometry')
    with pytest.raises(ValueError,match='Source asset changed'):module.source_assets(tmp_path)
