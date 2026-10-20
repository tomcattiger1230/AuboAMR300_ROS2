"""Hermes native tool plugin. No network or process launch at registration."""
import json
from pathlib import Path
import subprocess


def handle(args: dict, **kwargs) -> str:
    try:
        base = Path.home()
        result = subprocess.run(
            ['/usr/bin/python3', str(base / '.local/share/rebar-debug/spark_debug_bridge.py'),
             '--config', str(base / '.config/rebar-debug/config.json')],
            input=json.dumps(args, ensure_ascii=False), capture_output=True, text=True, timeout=20,
        )
        response = json.loads(result.stdout)
        if not isinstance(response, dict):
            raise ValueError('Bridge returned a non-object response')
        if result.returncode and response.get('ok') is not False:
            raise ValueError('Bridge process failed')
        return json.dumps(response, ensure_ascii=False)
    except Exception as exc:
        return json.dumps({'ok': False, 'training_paused': True, 'error': str(exc)}, ensure_ascii=False)


def register(ctx):
    schema = json.loads((Path(__file__).parent / 'tool.json').read_text())['tool']
    ctx.register_tool(name=schema['name'], toolset='rebar_debug', schema=schema, handler=handle)
