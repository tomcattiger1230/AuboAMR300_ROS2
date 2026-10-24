"""Exercise actual adapters without SSH, a GPU, or either agent runtime."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

MODULE = Path(__file__).resolve().parents[1]
BRIDGE = MODULE / 'scripts/spark_debug_bridge.py'


def invoke(payload, *args):
    result = subprocess.run([sys.executable, str(BRIDGE), *args],
                            input=json.dumps(payload), text=True, capture_output=True, timeout=10)
    return result.returncode, json.loads(result.stdout)


def test_dry_run_and_dedicated_key_requirement():
    code, value = invoke({'operation': 'status'})
    assert code == 0 and value['dry_run'] and value['training_paused']
    code, value = invoke({'operation': 'status'}, '--execute-readonly')
    assert code == 1 and value['ok'] is False and 'identity_file' in value['error']


@pytest.mark.parametrize('payload', [
    {'operation': 'train'}, {'operation': 'status', 'command': 'uname -a'},
    {'operation': 'status', 'target': 'other@host'},
])
def test_model_cannot_select_command_or_host(payload):
    code, value = invoke(payload)
    assert code == 1 and value['ok'] is False and value['training_paused']


def test_admin_configuration_pins_key_without_enabling_network(tmp_path):
    config = tmp_path / 'config.json'
    config.write_text(json.dumps({'identity_file': str(tmp_path / 'dedicated-key'), 'execute_readonly': False}))
    code, value = invoke({'operation': 'status'}, '--config', str(config))
    assert code == 0 and value['dry_run']
    assert str(tmp_path / 'dedicated-key') in value['argv']
    assert 'IdentitiesOnly=yes' in value['argv'] and 'IdentityAgent=none' in value['argv']
    assert value['argv'][1:3] == ['-F', '/dev/null']
    config.write_text('{"execute_readonly": "false"}')
    code, value = invoke({'operation': 'status'}, '--config', str(config))
    assert code == 1 and value['ok'] is False


def test_hermes_and_openclaw_handlers_use_fixed_bridge(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    folder = tmp_path / '.local/share/rebar-debug'
    folder.mkdir(parents=True)
    shutil.copyfile(BRIDGE, folder / BRIDGE.name)
    config = tmp_path / '.config/rebar-debug/config.json'
    config.parent.mkdir(parents=True)
    config.write_text('{"execute_readonly": false}')
    plugin = MODULE / 'deploy/hermes-rebar/__init__.py'
    spec = importlib.util.spec_from_file_location('rebar_hermes_test', plugin)
    hermes = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hermes)
    calls = []

    class Context:
        def register_tool(self, **kwargs):
            calls.append(kwargs)

    hermes.register(Context())
    assert calls[0]['name'] == 'rebar_readonly_debug'
    assert json.loads(hermes.handle({'operation': 'status'}, task_id='offline'))['dry_run']
    assert json.loads(hermes.handle({'operation': 'train'}))['ok'] is False
    if not shutil.which('node'):
        pytest.skip('Node unavailable; Hermes adapter checked')
    runner = MODULE / 'deploy/openclaw-rebar/runner.mjs'
    script = f"import {{runBridge}} from {json.dumps(runner.as_uri())}; console.log(JSON.stringify(await runBridge({{operation:process.argv[1]}})));"
    for operation in ['status', 'train']:
        result = subprocess.run(['node', '--input-type=module', '-e', script, operation],
                                capture_output=True, text=True, check=True, timeout=10)
        value = json.loads(result.stdout)
        assert value.get('dry_run') if operation == 'status' else value.get('ok') is False


def test_packaged_schemas_match():
    original = json.loads((MODULE / 'config/feishu_debug_tools.json').read_text())
    for name in ['hermes-rebar', 'openclaw-rebar']:
        assert json.loads((MODULE / 'deploy' / name / 'tool.json').read_text()) == original
