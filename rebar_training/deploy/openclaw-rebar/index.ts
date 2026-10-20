import { definePluginEntry } from 'openclaw/plugin-sdk/plugin-entry';
import { readFileSync } from 'node:fs';
import { runBridge } from './runner.mjs';

export default definePluginEntry({
  id: 'rebar-debug',
  name: 'Rebar read-only diagnostics',
  description: 'Read saved rebar reports and logs; training stays paused.',
  register(api) {
    const schema = JSON.parse(readFileSync(new URL('./tool.json', import.meta.url), 'utf8')).tool;
    api.registerTool({
      ...schema,
      async execute(_id, params) {
        const details = await runBridge(params);
        return { content: [{ type: 'text', text: JSON.stringify(details) }], details,
          isError: details.ok === false };
      },
    }, { optional: true });
  },
});
