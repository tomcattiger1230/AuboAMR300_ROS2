import { execFile } from 'node:child_process';
import { homedir } from 'node:os';
import { join } from 'node:path';

export function runBridge(params) {
  return new Promise((resolve) => {
    const fail = (error) => resolve({ ok: false, training_paused: true, error: String(error) });
    const child = execFile('/usr/bin/python3', [
      join(homedir(), '.local/share/rebar-debug/spark_debug_bridge.py'),
      '--config', join(homedir(), '.config/rebar-debug/config.json'),
    ], { timeout: 20000, maxBuffer: 512 * 1024 }, (error, stdout, stderr) => {
      try {
        const value = JSON.parse(stdout);
        if (!value || typeof value !== 'object' || Array.isArray(value)) throw Error('Non-object response');
        if (error && value.ok !== false) throw Error('Bridge process failed');
        resolve(value);
      } catch (parseError) {
        fail(error ? `${error.message}: ${stderr.slice(-1000)}` : parseError.message);
      }
    });
    child.stdin.on('error', () => {}); // Callback handles process failures, including EPIPE.
    child.stdin.end(JSON.stringify(params));
  });
}
