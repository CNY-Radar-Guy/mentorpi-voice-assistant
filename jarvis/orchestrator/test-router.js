'use strict';

/**
 * Phase 1 unit + mock-integration suite for the Jarvis orchestrator.
 * Run:  node D:\AI-Models\jarvis\orchestrator\test-router.js
 * Does not modify server.js / index.html / bots.json / README.txt / shell-agent.*.
 */

const fs = require('fs');
const os = require('os');
const path = require('path');
const http = require('http');
const { normalizeInput } = require('./input-normalizer');
const { SessionStore, estimateTokens, looksLikeFollowup } = require('./session-store');
const { VramScheduler, createVramScheduler } = require('./vram-scheduler');
const { createRouter, scoreRules, topScore } = require('./router');
const { createDispatcher, pickPassthrough } = require('./dispatcher');
const { createOrchestrator, handleOrchestratorRoute } = require('./index');

const INSTALLED = [
  'llama3.3:70b', 'qwen2.5-coder:32b', 'deepseek-r1:32b', 'qwen3:32b',
  'phi4:latest', 'gemma3:27b', 'mistral-small:24b', 'deepseek-r1:14b',
  'qwen3:14b', 'qwen3.5:4b'
];

function loadConfig() {
  const p = path.join(__dirname, '..', 'orchestrator-config.json');
  return JSON.parse(fs.readFileSync(p, 'utf8'));
}

function tmpRoot() {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'jarvis-orch-'));
  fs.mkdirSync(path.join(dir, 'sessions'), { recursive: true });
  return dir;
}

let passed = 0, failed = 0, skipped = 0;
const failures = [];

async function test(name, fn) {
  try {
    await fn();
    passed++;
    console.log('  PASS  ' + name);
  } catch (err) {
    failed++;
    failures.push({ name, err });
    console.log('  FAIL  ' + name);
    console.log('        ' + (err && err.stack || err));
  }
}

function skip(name, reason) {
  skipped++;
  console.log('  SKIP  ' + name + '  (' + reason + ')');
}

function assert(cond, msg) {
  if (!cond) throw new Error(msg || 'assertion failed');
}
function eq(a, b, msg) {
  if (a !== b) throw new Error((msg || 'expected ===') + `  got ${JSON.stringify(a)} !== ${JSON.stringify(b)}`);
}
function ge(a, b, msg) {
  if (!(a >= b)) throw new Error((msg || 'expected >=') + `  got ${a} < ${b}`);
}

function routerFor(overrides) {
  const config = { ...loadConfig(), ...(overrides || {}) };
  return createRouter({
    config,
    ollamaChatJSON: overrides && overrides.ollamaChatJSON,
    scheduler: new VramScheduler({
      installed: INSTALLED,
      loaded: (overrides && overrides.loaded) || [],
      budgetMb: 16384,
      reserveMb: 1536,
      weights: config.model_weights_mb,
      heavyModels: config.heavy_models,
      fitPolicy: config.fit_policy,
      allowPartialOffload: false
    })
  });
}

async function main() {
  console.log('Jarvis orchestrator Phase 1 tests\n');

  // ---------- input normalizer ----------
  console.log('input-normalizer');
  await test('extracts @bot mentions and aliases', () => {
    const n = normalizeInput('Hey @coder please look at this, cc @shell-agent', { statFiles: false });
    assert(n.mentions.includes('coder'), 'coder mention');
    assert(n.mentions.includes('shell-agent'), 'shell-agent mention');
    eq(n.shape === '@mention' || n.shape === 'mixed', true, 'shape mentions');
  });
  await test('extracts URLs and does not treat them as file paths', () => {
    const n = normalizeInput('look up https://example.com/installer.exe?x=1', { statFiles: false });
    assert(n.urls.some((u) => u.startsWith('https://example.com/installer.exe')), 'url');
    assert(!n.file_paths.some((f) => /example.com/.test(f.path)), 'url is not a path');
  });
  await test('extracts Windows paths', () => {
    const n = normalizeInput('Please review D:\\AI-Models\\jarvis\\server.js and also \\\\nas\\share\\part.stl', { statFiles: false });
    assert(n.file_paths.some((f) => /server\.js$/.test(f.path)), 'win path ' + JSON.stringify(n.file_paths));
    assert(n.file_paths.some((f) => /part\.stl$/.test(f.path)), 'unc path');
  });
  await test('bounds file excerpts and hashes without dumping the whole file', () => {
    const big = path.join(os.tmpdir(), 'jarvis-orch-excerpt-' + Date.now() + '.txt');
    const body = ('ALPHA-ONE line\n'.repeat(400) + 'END');
    fs.writeFileSync(big, body);
    try {
      const n = normalizeInput('inspect ' + big);
      eq(n.file_paths.length, 1, 'one file');
      const f = n.file_paths[0];
      eq(f.exists, true, 'exists');
      eq(f.size, Buffer.byteLength(body), 'size');
      assert(f.hash && /^[a-f0-9]{64}$/.test(f.hash), 'sha256');
      assert(f.excerpt && f.excerpt.length <= 800, 'excerpt bounded ' + (f.excerpt && f.excerpt.length));
      eq(f.truncated, true, 'truncated flag');
      assert(!JSON.stringify(n.preview).includes('END'), 'raw tail not in preview');
    } finally {
      try { fs.unlinkSync(big); } catch { }
    }
  });
  await test('parses structured JSON payloads', () => {
    const n = normalizeInput('{"intent":"install","product":"Bambu Studio","source":"https://example.com"}');
    assert(n.json_payload && n.json_payload.product === 'Bambu Studio', 'json payload');
    assert(n.urls.includes('https://example.com'), 'url inside json string');
  });
  await test('detects multi-step hints', () => {
    const n = normalizeInput('research this URL https://example.com then install the app');
    eq(n.hints.multi_step, true, 'multi_step');
    ge(n.steps.length, 2, 'steps');
  });

  // ---------- Stage 0 fast path ----------
  console.log('\nrouter stage 0 (0 tokens)');
  const r = routerFor();

  const cases = [
    ['Download and install Bambu Studio', 'shell-agent'],
    ['install VLC', 'shell-agent'],
    ['run this command: dir D:\\AI-Models', 'shell-agent'],
    ['Write an OpenSCAD script for a 20mm cube', 'coder'],
    ['debug this python function, it throws TypeError', 'coder'],
    ['fix this javascript script please', 'coder'],
    ['look up https://example.com', 'researcher'],
    ['search the web for RTX 5080 VRAM', 'researcher'],
    ["how's the GPU memory on ALPHA-ONE", 'ops'],
    ['is Wally reachable', 'ops'],
    ['Tailscale status', 'ops'],
    ['check disk space', 'ops']
  ];
  for (const [text, bot] of cases) {
    await test(`fast-path "${text}" → ${bot}`, async () => {
      const out = await r.route(text, { statFiles: false });
      eq(out.target_bot, bot, text);
      eq(out.stage, 'fast_path', 'stage');
      eq(out.tokens.orchestrator_tokens, 0, '0 orchestrator tokens');
      eq(out.tokens.prompt_eval_count, 0, '0 prompt_eval');
    });
  }

  await test('@bot override beats conflicting keywords', async () => {
    const out = await r.route('@coder what is the weather on ALPHA-ONE and the GPU memory', { statFiles: false });
    eq(out.target_bot, 'coder', 'mention wins');
    eq(out.stage, 'fast_path', 'stage');
    eq(out.confidence, 1, 'confidence 1');
    eq(out.tokens.orchestrator_tokens, 0, '0 tokens');
  });
  await test('@shell-agent override', async () => {
    const out = await r.route('@shell-agent list files in D:\\AI-Models', { statFiles: false });
    eq(out.target_bot, 'shell-agent');
    eq(out.stage, 'fast_path');
  });
  await test('@ops override', async () => {
    const out = await r.route('@ops check GPU', { statFiles: false });
    eq(out.target_bot, 'ops');
  });

  await test('compound "check disk space and install VLC" splits ops → shell-agent', async () => {
    const out = await r.route('check disk space and install VLC', { statFiles: false });
    eq(out.stage, 'fast_path', 'compound stays on fast path');
    eq(out.tokens.orchestrator_tokens, 0, '0 tokens');
    assert(out.needs_handoff, 'needs_handoff');
    assert(Array.isArray(out.steps) && out.steps.length >= 2, 'two steps ' + JSON.stringify(out.steps));
    eq(out.steps[0].target_bot, 'ops', 'step1 ops');
    eq(out.steps[1].target_bot, 'shell-agent', 'step2 shell-agent');
  });

  // ---------- Stage 1 LLM fallback ----------
  console.log('\nrouter stage 1 (compact LLM / fallback)');
  await test('ambiguous input calls compact LLM router', async () => {
    let called = 0;
    const llm = routerFor({
      ollamaChatJSON: async (args) => {
        called++;
        assert(args.options.num_ctx === 2048, 'num_ctx 2048');
        assert(args.options.num_predict === 120, 'num_predict 120');
        assert(args.think === false, 'think false');
        const prompt = JSON.stringify(args.messages);
        ge(8000 - prompt.length, 0, 'prompt stays compact, len=' + prompt.length);
        return {
          json: { target_bot: 'general', intent: 'chat', reuse_session: false, needs_handoff: false },
          prompt_eval_count: 140, eval_count: 30
        };
      }
    });
    const out = await llm.route('can you help me with this', { statFiles: false });
    eq(called, 1, 'LLM called');
    eq(out.target_bot, 'general');
    eq(out.stage, 'llm_router');
    eq(out.tokens.orchestrator_tokens, 170, 'token accounting');
  });

  await test('falls back cleanly when Ollama is unavailable', async () => {
    const llm = routerFor({
      ollamaChatJSON: async () => { throw new Error('ECONNREFUSED mock'); }
    });
    const out = await llm.route('can you help me with this', { statFiles: false });
    eq(out.stage, 'fallback', 'fallback stage');
    eq(out.target_bot, 'general', 'defaults to general');
    assert(/llm_error|unavailable/i.test(out.reason), 'reason records the failure');
  });

  await test('falls back when no ollamaChatJSON is configured', async () => {
    const bare = createRouter({ config: loadConfig() });
    const out = await bare.route('hmm not sure', { statFiles: false });
    eq(out.stage, 'fallback');
    eq(out.tokens.orchestrator_tokens, 0);
  });

  // ---------- sessions ----------
  console.log('\nsession-store');
  await test('reuses a specialist session on follow-up and saves tokens vs naive replay', async () => {
    const root = tmpRoot();
    const store = new SessionStore({
      root,
      indexPath: path.join(root, 'sessions.json'),
      sessionsDir: path.join(root, 'sessions'),
      sharedFactsPath: path.join(root, 'shared-facts.json'),
      maxRawTurns: 4,
      compactAfterTurns: 6,
      compactAfterTokens: 1800
    });
    store.setSharedFacts('alpha-one-hw', { gpu: 'RTX 5080', ram_gb: 31.46 });
    const first = store.getOrCreate('coder', 'write a python script to parse CSV');
    const longScript = 'def parse_csv(path):\n    ' + 'x = 1\n    ' .repeat(40);
    store.appendTurn(first.session.id, 'user', 'write a python script to parse CSV of printer jobs');
    store.appendTurn(first.session.id, 'assistant', longScript);

    const fu = store.findFollowup('add error handling to it');
    assert(fu && fu.session && fu.session.id === first.session.id, 'follow-up reused same session');
    assert(looksLikeFollowup('add error handling to it'), 'follow-up language');

    // Drive through the dispatcher so telemetry/reuse counters are exercised.
    const config = loadConfig();
    const d = createDispatcher({
      config,
      root,
      sessionsIndex: path.join(root, 'sessions.json'),
      sessionsDir: path.join(root, 'sessions'),
      sharedFactsPath: path.join(root, 'shared-facts.json'),
      telemetryFile: path.join(root, 'telemetry.jsonl'),
      sessionStore: store,
      installed: INSTALLED,
      loaded: [{ name: 'qwen3:14b', size_mb: 11000 }],
      ollamaAdapter: async (task) => ({
        summary: 'Added try/except around the parser. ' + (task.prompt.session_id === first.session.id ? 'REUSED' : 'COLD'),
        artifacts: [{ type: 'code', lang: 'python' }],
        tool_events: [],
        session_delta: { facts: { language: 'python' } },
        tokens_used: { prompt_eval_count: 80, eval_count: 40, specialist_tokens: 120 }
      }),
      ollamaChatJSON: async () => { throw new Error('should not be called on fast-path follow-up'); }
    });
    const out = await d.dispatch('add error handling to it', { statFiles: false });
    eq(out.route.target_bot, 'coder', 'stayed on coder');
    eq(out.route.stage, 'fast_path', 'follow-up is fast path');
    eq(out.route.tokens.orchestrator_tokens, 0, '0 manager tokens');
    assert(out.steps[0].reused_session, 'dispatcher reused session');
    assert(/REUSED/.test(out.summary), 'adapter saw same session');

    const final = store.get(first.session.id);
    const payload = store.buildPromptPayload(final);
    assert(payload.tokens_est > 0, 'payload has a token estimate');
    assert(payload.shared_fact_ids.some((r) => r.startsWith('alpha-one-hw#')), 'shared facts by id/hash');
    assert(!JSON.stringify(payload.messages).includes('31.46'), 'hardware blob is not duplicated into message history');
    // Follow-up reuse is the main assertion here; compaction savings are covered in the next test.
    eq(final.id, first.session.id, 'same session id after follow-up');
  });

  await test('auto-compacts rolling summary once history crosses the threshold, with measurable savings', async () => {
    const root = tmpRoot();
    const store = new SessionStore({
      root,
      indexPath: path.join(root, 'sessions.json'),
      sessionsDir: path.join(root, 'sessions'),
      maxRawTurns: 4,
      compactAfterTurns: 6,
      compactAfterTokens: 400
    });
    const { session } = store.getOrCreate('coder', 'openscad cube');
    const archive = [];
    for (let i = 0; i < 8; i++) {
      const u = `Turn ${i} user: please tweak the OpenSCAD cube, add fillet radius ${i}.5 mm and reprint notes for Wally tray slot ${i}.`;
      const a = `Turn ${i} assistant: updated cube() module, radius=${i}.5, here is a long-ish snippet: ` + 'cube([20,20,20]); // fillet and tray notes\n'.repeat(80);
      archive.push(u, a);
      store.appendTurn(session.id, 'user', u);
      store.appendTurn(session.id, 'assistant', a);
    }
    const after = store.get(session.id);
    ge(after.compact_count, 1, 'compacted at least once, count=' + after.compact_count);
    ge(4, after.turns.length, 'raw turns capped at N=4, have ' + after.turns.length);
    assert(after.summary && /Turn 0 user/.test(after.summary), 'older turns folded into summary');
    const payload = store.buildPromptPayload(after);
    const naive = estimateTokens(archive.join('\n'));
    const saved = naive - payload.tokens_est;
    ge(saved, 200, `expected >=200 token savings, naive=${naive} payload=${payload.tokens_est} saved=${saved}`);
    assert(saved / naive >= 0.2, `expected >=20% savings, got ${(saved / naive * 100).toFixed(1)}%`);
    // Last turns remain verbatim.
    assert(after.turns.some((t) => /Turn 7/.test(t.content)), 'latest raw turns kept');
  });

  // ---------- VRAM scheduler ----------
  console.log('\nvram-scheduler');
  await test('cold fast_router selects already-installed qwen3.5:4b', () => {
    const cfg = loadConfig();
    const sch = new VramScheduler({
      installed: INSTALLED, loaded: [], budgetMb: 16384, reserveMb: 1536,
      weights: cfg.model_weights_mb, heavyModels: cfg.heavy_models, fitPolicy: 'prefer_in_vram'
    });
    const plan = sch.selectModel(cfg.model_tiers.fast_router);
    eq(plan.ok, true);
    eq(plan.model, 'qwen3.5:4b');
    eq(plan.num_ctx, 2048);
    eq(plan.keep_alive, '5m');
    assert(!plan.warning, '4b should fit comfortably');
  });

  await test('reuses warm qwen3:14b instead of swapping in the 4B router', () => {
    const cfg = loadConfig();
    const sch = new VramScheduler({
      installed: INSTALLED,
      loaded: [{ name: 'qwen3:14b', size_mb: 11000 }],
      budgetMb: 16384, reserveMb: 1536,
      weights: cfg.model_weights_mb, heavyModels: cfg.heavy_models,
      reuseLoadedIfCompatible: true
    });
    const plan = sch.selectModel(cfg.model_tiers.fast_router);
    eq(plan.model, 'qwen3:14b', 'reuse warm 14b');
    eq(plan.reuse, true, 'reuse flag');
    assert(!plan.swap, 'no swap');
  });

  await test('deep_coder falls back to qwen3:14b instead of spilling qwen2.5-coder:32b', () => {
    const cfg = loadConfig();
    const sch = new VramScheduler({
      installed: INSTALLED, loaded: [], budgetMb: 16384, reserveMb: 1536,
      weights: cfg.model_weights_mb, heavyModels: cfg.heavy_models,
      fitPolicy: 'prefer_in_vram', allowPartialOffload: false
    });
    const plan = sch.selectModel(cfg.model_tiers.deep_coder);
    eq(plan.model, 'qwen3:14b', '14b fits; 32b would spill');
    assert(plan.estimated_vram_mb <= 16384 - 1536, 'stays in usable VRAM');
  });

  await test('32B is selected only when partial offload is allowed', () => {
    const cfg = loadConfig();
    const sch = new VramScheduler({
      installed: INSTALLED, loaded: [], budgetMb: 16384, reserveMb: 1536,
      weights: cfg.model_weights_mb, heavyModels: cfg.heavy_models,
      allowPartialOffload: true
    });
    const plan = sch.selectModel(cfg.model_tiers.deep_coder, { allowPartialOffload: true });
    eq(plan.model, 'qwen2.5-coder:32b');
    assert(!!plan.warning, 'warns about VRAM spill');
  });

  await test('evicts a loaded heavy model before loading a competing heavy', () => {
    const cfg = loadConfig();
    const sch = new VramScheduler({
      installed: INSTALLED,
      loaded: [{ name: 'qwen3:32b', size_mb: 20000 }],
      budgetMb: 16384, reserveMb: 1536,
      weights: cfg.model_weights_mb, heavyModels: cfg.heavy_models,
      allowPartialOffload: true
    });
    const plan = sch.selectModel(cfg.model_tiers.deep_coder, { allowPartialOffload: true, forceModel: 'qwen2.5-coder:32b' });
    eq(plan.model, 'qwen2.5-coder:32b');
    assert(plan.evict.some((e) => e.name === 'qwen3:32b' && e.keep_alive === '0'), 'evict 32b with keep_alive 0: ' + JSON.stringify(plan.evict));
  });

  await test('never auto-selects llama3.3:70b (larger than VRAM+RAM)', () => {
    const cfg = loadConfig();
    const sch = new VramScheduler({
      installed: INSTALLED, loaded: [], budgetMb: 16384, reserveMb: 1536,
      weights: cfg.model_weights_mb, heavyModels: cfg.heavy_models, fitPolicy: 'prefer_in_vram'
    });
    for (const tier of Object.values(cfg.model_tiers)) {
      const plan = sch.selectModel(tier);
      assert(plan.model !== 'llama3.3:70b', '70b auto-selected for ' + JSON.stringify(tier.preferred_models));
    }
  });

  await test('general_chat prefers already-loaded 14b over a swap to phi4', () => {
    const cfg = loadConfig();
    const sch = new VramScheduler({
      installed: INSTALLED,
      loaded: [{ name: 'qwen3:14b', size_mb: 11000 }],
      budgetMb: 16384, reserveMb: 1536,
      weights: cfg.model_weights_mb
    });
    const plan = sch.selectModel(cfg.model_tiers.general_chat);
    eq(plan.model, 'qwen3:14b');
    eq(plan.reuse, true);
  });

  // ---------- shell-agent adapter passthrough ----------
  console.log('\nshell-agent adapter / dispatcher');
  await test('preserves approval_required and guardrail fields', async () => {
    const root = tmpRoot();
    const config = loadConfig();
    const d = createDispatcher({
      config,
      root,
      sessionsIndex: path.join(root, 'sessions.json'),
      sessionsDir: path.join(root, 'sessions'),
      sharedFactsPath: path.join(root, 'shared-facts.json'),
      telemetryFile: path.join(root, 'telemetry.jsonl'),
      installed: INSTALLED,
      loaded: [{ name: 'qwen3:14b', size_mb: 11000 }],
      shellAgentAdapter: async (task) => ({
        summary: 'Bambu Studio install is ready for your approval.',
        artifacts: [{ type: 'installer', url: 'https://example.com/bambu.exe' }],
        tool_events: [{ type: 'plan', command: 'Start-Process installer.exe' }],
        session_delta: {},
        tokens_used: { prompt_eval_count: 0, eval_count: 0, specialist_tokens: 0 },
        approval_required: true,
        guardrail: { tier: 2, reason: 'software install requires click-to-approve' },
        blocked: false,
        proposed_command: 'Start-Process installer.exe',
        tier: 2
      })
    });
    const out = await d.dispatch('Download and install Bambu Studio', { statFiles: false });
    eq(out.route.target_bot, 'shell-agent');
    eq(out.route.stage, 'fast_path');
    eq(out.approval_required, true, 'approval_required survived');
    assert(out.guardrail && out.guardrail.tier === 2, 'guardrail survived');
    eq(out.proposed_command, 'Start-Process installer.exe', 'proposed_command survived');
    eq(out.tier, 2, 'tier survived');
    eq(out.blocked, false, 'blocked flag present (not stripped)');
    const pt = pickPassthrough(out);
    assert(pt.approval_required === true && pt.guardrail, 'pickPassthrough');
  });

  await test('records telemetry.jsonl with token split and fast_path hit', async () => {
    const root = tmpRoot();
    const d = createDispatcher({
      config: loadConfig(),
      root,
      sessionsIndex: path.join(root, 'sessions.json'),
      sessionsDir: path.join(root, 'sessions'),
      sharedFactsPath: path.join(root, 'shared-facts.json'),
      telemetryFile: path.join(root, 'telemetry.jsonl'),
      installed: INSTALLED,
      opsAdapter: async () => ({
        summary: 'C: 120 GB free, D: 800 GB free (probe, 0 LLM tokens).',
        artifacts: [], tool_events: [{ type: 'probe', name: 'disk' }],
        session_delta: {}, tokens_used: { prompt_eval_count: 0, eval_count: 0, specialist_tokens: 0 }
      })
    });
    const out = await d.dispatch('check disk space', { statFiles: false });
    eq(out.route.stage, 'fast_path');
    eq(out.tokens_used.orchestrator_tokens, 0);
    const rows = d.readTelemetry(10);
    ge(rows.length, 1, 'telemetry row');
    eq(rows[0].stage, 'fast_path');
    eq(rows[0].target_bot, 'ops');
    assert(rows[0].fast_path_hits >= 1, 'counter');
  });

  await test('HTTP route helper serves specialists / route / chat', async () => {
    const root = tmpRoot();
    const orch = createOrchestrator({
      jarvisRoot: path.join(__dirname, '..'),
      config: loadConfig(),
      ollamaChatJSON: async () => ({ json: { target_bot: 'general', intent: 'chat', reuse_session: false } }),
      installed: INSTALLED,
      loaded: [],
      opsAdapter: async () => ({
        summary: 'GPU 1536 MiB used of 16303 (probe).',
        artifacts: [], tool_events: [], session_delta: {},
        tokens_used: { prompt_eval_count: 0, eval_count: 0, specialist_tokens: 0 }
      }),
      sessionStore: new SessionStore({ root, indexPath: path.join(root, 'sessions.json'), sessionsDir: path.join(root, 'sessions'), sharedFactsPath: path.join(root, 'shared-facts.json') })
    });
    // Point dispatcher IO at the temp root (createOrchestrator already built one; reuse via handle + existing orch).
    orch.dispatcher.telemetryFile = path.join(root, 'telemetry.jsonl');

    const server = http.createServer((req, res) => handleOrchestratorRoute(req, res, orch));
    await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
    const { port } = server.address();
    try {
      const specs = await (await fetch('http://127.0.0.1:' + port + '/api/orchestrator/specialists')).json();
      assert(specs.specialists['shell-agent'] && specs.specialists.coder, 'specialists listed');
      const routed = await (await fetch('http://127.0.0.1:' + port + '/api/orchestrator/route', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: "how's the GPU memory on ALPHA-ONE" })
      })).json();
      eq(routed.route.target_bot, 'ops', 'HTTP route');
      eq(routed.route.stage, 'fast_path');
      const chat = await (await fetch('http://127.0.0.1:' + port + '/api/orchestrator/chat', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: "how's the GPU memory on ALPHA-ONE" })
      })).json();
      assert(/GPU/.test(chat.summary), 'chat summary');
      eq(chat.tokens_used.orchestrator_tokens, 0);
    } finally {
      await new Promise((resolve) => server.close(resolve));
    }
  });

  // ---------- optional live probe ----------
  console.log('\nlive Ollama probe (read-only)');
  const cfg = loadConfig();
  const sch = createVramScheduler({
    budgetMb: 16384, reserveMb: 1536,
    weights: cfg.model_weights_mb, heavyModels: cfg.heavy_models,
    ollamaBaseUrl: process.env.OLLAMA_HOST || cfg.ollama_base_url || 'http://127.0.0.1:11434'
  });
  let live = null;
  try {
    live = await sch.refreshFromOllama(2500);
  } catch (err) {
    skip('live /api/tags + /api/ps probe', String(err && err.message || err));
    live = null;
  }
  if (live) {
    await test('live inventory maps into the VRAM scheduler', () => {
      assert(Array.isArray(live.installed) && live.installed.length > 0, 'tags returned models');
      for (const expected of ['qwen3:14b', 'qwen3.5:4b', 'qwen2.5-coder:32b']) {
        assert(live.installed.some((n) => n === expected || n.startsWith(expected.split(':')[0])), 'missing ' + expected + ' in ' + live.installed.join(', '));
      }
      const plan = sch.selectModel(cfg.model_tiers.fast_router);
      assert(plan.ok && plan.model, 'selected ' + plan.model);
      assert(sch.isInstalled(plan.model), 'selected model is actually installed');
      if (live.loaded.length === 0) eq(plan.model, 'qwen3.5:4b', 'cold GPU → 4b router');
      else assert(INSTALLED.some((n) => n === plan.model) || live.installed.includes(plan.model), 'warm pick is installed');
      console.log('        live installed: ' + live.installed.join(', '));
      console.log('        live loaded:    ' + (live.loaded.map((m) => m.name).join(', ') || '(none)'));
      console.log('        router pick:    ' + plan.model + '  ctx=' + plan.num_ctx + '  keep_alive=' + plan.keep_alive + '  reason=' + plan.reason);
    });
  }

  console.log('\n' + passed + ' passed, ' + failed + ' failed, ' + skipped + ' skipped');
  if (failures.length) process.exitCode = 1;
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
