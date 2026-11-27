'use strict';

const fs = require('fs');
const path = require('path');
const { createDispatcher } = require('./dispatcher');
const { createRouter } = require('./router');
const { createSessionStore } = require('./session-store');
const { createVramScheduler } = require('./vram-scheduler');
const { normalizeInput } = require('./input-normalizer');

function readJSON(file, def) {
  try { return JSON.parse(fs.readFileSync(file, 'utf8')); } catch { return def; }
}

function resolveUnder(root, maybe) {
  if (!maybe) return null;
  return path.isAbsolute(maybe) ? maybe : path.join(root, maybe);
}

/**
 * Build a standalone orchestrator. All paths default to the Jarvis root
 * (the parent of this folder) so Phase 2 can mount it with a short call.
 *
 * @param {object} [opts]
 * @param {string} [opts.configPath]
 * @param {string} [opts.botsJsonPath]
 * @param {function} [opts.shellAgentAdapter]
 * @param {string} [opts.ollamaBaseUrl]
 */
function createOrchestrator(opts = {}) {
  const here = __dirname;
  const jarvisRoot = opts.jarvisRoot || path.join(here, '..');
  const configPath = opts.configPath || path.join(jarvisRoot, 'orchestrator-config.json');
  const config = opts.config || readJSON(configPath, {});
  const sess = config.session || {};
  const dispatcher = createDispatcher({
    config,
    root: here,
    sessionsIndex: resolveUnder(jarvisRoot, sess.sessions_index) || path.join(here, 'sessions.json'),
    sessionsDir: resolveUnder(jarvisRoot, sess.sessions_dir) || path.join(here, 'sessions'),
    sharedFactsPath: resolveUnder(jarvisRoot, sess.shared_facts_file) || path.join(here, 'shared-facts.json'),
    telemetryFile: resolveUnder(jarvisRoot, config.telemetry_file) || path.join(here, 'telemetry.jsonl'),
    ollamaBaseUrl: opts.ollamaBaseUrl || config.ollama_base_url || 'http://127.0.0.1:11434',
    shellAgentAdapter: opts.shellAgentAdapter,
    ollamaAdapter: opts.ollamaAdapter,
    opsAdapter: opts.opsAdapter,
    webAdapter: opts.webAdapter,
    ollamaChatJSON: opts.ollamaChatJSON,
    fetchImpl: opts.fetchImpl,
    installed: opts.installed,
    loaded: opts.loaded,
    botsJson: opts.botsJson || readJSON(opts.botsJsonPath || path.join(jarvisRoot, 'bots.json'), null),
    scheduler: opts.scheduler,
    sessionStore: opts.sessionStore,
    router: opts.router
  });

  // Seed shared machine facts once (id/hash only thereafter).
  const host = config.host || {};
  dispatcher.session.setSharedFacts('alpha-one-hw', {
    name: host.name || 'ALPHA-ONE',
    gpu: host.gpu || 'NVIDIA GeForce RTX 5080',
    vram_budget_mb: host.vram_budget_mb || 16384,
    system_ram_mb: host.system_ram_mb || 32215,
    cpu: host.cpu || 'Intel Core Ultra 7 265K',
    cores: host.cores || 20,
    jarvis_root: host.jarvis_root || jarvisRoot,
    ollama: opts.ollamaBaseUrl || config.ollama_base_url || 'http://127.0.0.1:11434'
  });

  return {
    config,
    configPath,
    jarvisRoot,
    dispatcher,
    router: dispatcher.router,
    session: dispatcher.session,
    scheduler: dispatcher.scheduler,
    route: (input, ctx) => dispatcher.routeOnly(input, ctx),
    chat: (input, ctx) => dispatcher.dispatch(input, ctx),
    specialists: () => dispatcher.specialists(),
    sessions: () => dispatcher.session.list(),
    telemetry: (n) => dispatcher.readTelemetry(n)
  };
}

function send(res, code, obj) {
  const b = Buffer.from(JSON.stringify(obj));
  res.writeHead(code, {
    'Content-Type': 'application/json; charset=utf-8',
    'Content-Length': b.length,
    'Cache-Control': 'no-store'
  });
  res.end(b);
}

function readBody(req, limit) {
  const max = limit || 1e6;
  return new Promise((resolve, reject) => {
    const chunks = [];
    let n = 0;
    req.on('data', (c) => {
      n += c.length;
      if (n > max) { reject(new Error('body too large')); try { req.destroy(); } catch { } return; }
      chunks.push(c);
    });
    req.on('end', () => {
      const raw = Buffer.concat(chunks).toString('utf8');
      if (!raw) return resolve({});
      try { resolve(JSON.parse(raw)); } catch { resolve({ text: raw }); }
    });
    req.on('error', reject);
  });
}

function routeName(urlPath) {
  const p = String(urlPath || '').split('?')[0].replace(/\/+$/, '') || '/';
  return p;
}

/**
 * Drop-in HTTP handler for Jarvis's createServer.
 * Mount with: if (urlPath.startsWith('/api/orchestrator/')) return handleOrchestratorRoute(req, res, orchestrator);
 */
async function handleOrchestratorRoute(req, res, orchestrator) {
  const p = routeName(req.url);
  try {
    if (req.method === 'GET' && p === '/api/orchestrator/specialists') {
      const bots = orchestrator.dispatcher.botsJson;
      return send(res, 200, { specialists: orchestrator.specialists(), bots_inventory: bots, config_version: orchestrator.config.version || 1 });
    }
    if (req.method === 'GET' && p === '/api/orchestrator/sessions') {
      return send(res, 200, { sessions: orchestrator.sessions(), last_active: orchestrator.session.lastActive || null });
    }
    if (req.method === 'GET' && p === '/api/orchestrator/telemetry') {
      return send(res, 200, { recent: orchestrator.telemetry(200), counters: orchestrator.dispatcher.counters });
    }
    if (req.method === 'POST' && p === '/api/orchestrator/route') {
      const body = await readBody(req);
      const result = await orchestrator.route(body.text || body.prompt || body, body.ctx || {});
      return send(res, 200, result);
    }
    if (req.method === 'POST' && p === '/api/orchestrator/chat') {
      const body = await readBody(req);
      const result = await orchestrator.chat(body.text || body.prompt || body, body.ctx || {});
      return send(res, 200, result);
    }
    return send(res, 404, { error: 'unknown orchestrator route', path: p });
  } catch (err) {
    return send(res, 500, { error: String(err && err.message || err) });
  }
}

module.exports = {
  createOrchestrator,
  handleOrchestratorRoute,
  createDispatcher,
  createRouter,
  createSessionStore,
  createVramScheduler,
  normalizeInput,
  send,
  readBody
};
