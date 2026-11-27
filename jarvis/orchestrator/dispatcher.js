'use strict';

const fs = require('fs');
const path = require('path');
const { normalizeInput } = require('./input-normalizer');
const { createRouter } = require('./router');
const { createSessionStore, estimateTokens } = require('./session-store');
const { createVramScheduler } = require('./vram-scheduler');

function nowIso() { return new Date().toISOString(); }

function appendJsonl(file, obj) {
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.appendFileSync(file, JSON.stringify(obj) + '\n', 'utf8');
}

function readJsonlTail(file, n) {
  try {
    const lines = fs.readFileSync(file, 'utf8').split(/\n/).filter(Boolean);
    return lines.slice(-(n || 200)).map((l) => { try { return JSON.parse(l); } catch { return { raw: l }; } });
  } catch { return []; }
}

const PASSTHROUGH = [
  'approval_required', 'guardrail', 'blocked', 'proposed_command', 'proposed',
  'tier', 'requires_approval', 'approval', 'risk', 'command', 'args',
  'deny_reason', 'policy', 'confirm_token'
];

function pickPassthrough(src) {
  const out = {};
  if (!src || typeof src !== 'object') return out;
  for (const k of PASSTHROUGH) if (src[k] != null) out[k] = src[k];
  return out;
}

function defaultOllamaAdapter({ ollamaBaseUrl, fetchImpl }) {
  const base = (ollamaBaseUrl || 'http://127.0.0.1:11434').replace(/\/$/, '');
  const f = fetchImpl || (typeof fetch === 'function' ? fetch : null);
  return async function ollamaAdapter(task) {
    if (!f) throw new Error('no fetch for ollama adapter');
    const body = {
      model: task.model,
      messages: task.messages,
      stream: false,
      think: !!task.think,
      keep_alive: task.keep_alive || '5m',
      options: {
        temperature: task.temperature ?? 0.6,
        num_ctx: task.num_ctx || 4096,
        ...(task.num_predict ? { num_predict: task.num_predict } : {})
      }
    };
    if (task.format) body.format = task.format;
    const r = await f(base + '/api/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    if (!r.ok) throw new Error('Ollama HTTP ' + r.status + ': ' + (await r.text()).slice(0, 200));
    const json = await r.json();
    const content = (json.message && json.message.content) || json.response || '';
    return {
      summary: String(content).replace(/<think>[\s\S]*?<\/think>/g, '').trim(),
      artifacts: [],
      tool_events: [],
      session_delta: { assistant: content },
      tokens_used: {
        prompt_eval_count: json.prompt_eval_count || 0,
        eval_count: json.eval_count || 0,
        specialist_tokens: (json.prompt_eval_count || 0) + (json.eval_count || 0)
      },
      raw: json
    };
  };
}

function stubAdapter(name) {
  return async function stub(task) {
    return {
      summary: `[${name} adapter not wired in Phase 1] ${String((task && task.inputs && task.inputs.text) || task.text || '').slice(0, 200)}`,
      artifacts: [],
      tool_events: [{ type: 'adapter_unwired', adapter: name }],
      session_delta: {},
      tokens_used: { prompt_eval_count: 0, eval_count: 0, specialist_tokens: 0 },
      unwired: true
    };
  };
}

class Dispatcher {
  constructor(opts = {}) {
    this.config = opts.config || {};
    this.root = opts.root || process.cwd();
    this.session = opts.sessionStore || createSessionStore({
      root: this.root,
      indexPath: opts.sessionsIndex,
      sessionsDir: opts.sessionsDir,
      sharedFactsPath: opts.sharedFactsPath,
      maxRawTurns: this.config.session && this.config.session.max_raw_turns,
      compactAfterTurns: this.config.session && this.config.session.compact_after_turns,
      compactAfterTokens: this.config.session && this.config.session.compact_after_tokens,
      followupIdleMs: this.config.session && this.config.session.followup_idle_ms
    });
    this.scheduler = opts.scheduler || createVramScheduler({
      budgetMb: this.config.host && this.config.host.vram_budget_mb,
      reserveMb: this.config.host && this.config.host.vram_reserve_mb,
      weights: this.config.model_weights_mb,
      heavyModels: this.config.heavy_models,
      fitPolicy: this.config.fit_policy,
      allowPartialOffload: this.config.allow_partial_offload,
      reuseLoadedIfCompatible: this.config.reuse_loaded_if_compatible,
      ollamaBaseUrl: opts.ollamaBaseUrl || this.config.ollama_base_url,
      fetchImpl: opts.fetchImpl,
      installed: opts.installed,
      loaded: opts.loaded
    });
    this.adapters = {
      'shell-agent': opts.shellAgentAdapter || stubAdapter('shell-agent'),
      ollama: opts.ollamaAdapter || defaultOllamaAdapter({ ollamaBaseUrl: opts.ollamaBaseUrl || this.config.ollama_base_url, fetchImpl: opts.fetchImpl }),
      ops: opts.opsAdapter || stubAdapter('ops'),
      web: opts.webAdapter || stubAdapter('web')
    };
    this.ollamaChatJSON = opts.ollamaChatJSON || this._makeRouterLLM();
    this.router = opts.router || createRouter({
      config: this.config,
      ollamaChatJSON: this.ollamaChatJSON,
      scheduler: this.scheduler
    });
    this.telemetryFile = opts.telemetryFile || path.join(this.root, 'telemetry.jsonl');
    this.botsJson = opts.botsJson || null;
    this.counters = { fast_path_hits: 0, session_reuses: 0, compactions: 0, model_swaps: 0, evictions: 0, llm_router_calls: 0 };
  }

  _makeRouterLLM() {
    const adapter = this.adapters.ollama;
    return async (args) => {
      // Evict first if the scheduler asked for it (keep_alive 0 via a tiny generate is Phase 2;
      // Phase 1 records evict plan and lets the next /api/chat keep_alive do the work, or a hook).
      if (typeof args.onEvict === 'function' && args.evict && args.evict.length) await args.onEvict(args.evict);
      const result = await adapter({
        model: args.model,
        messages: args.messages,
        think: false,
        keep_alive: args.keep_alive,
        temperature: 0,
        num_ctx: 2048,
        num_predict: 120,
        format: args.schema || 'json'
      });
      let json = null;
      try { json = JSON.parse(result.summary); } catch { json = null; }
      return {
        json,
        content: result.summary,
        prompt_eval_count: result.tokens_used && result.tokens_used.prompt_eval_count || 0,
        eval_count: result.tokens_used && result.tokens_used.eval_count || 0
      };
    };
  }

  specialists() {
    return this.config.specialists || {};
  }

  async routeOnly(input, ctx) {
    const follow = this.session.findFollowup(typeof input === 'string' ? input : (input && (input.text || input.prompt)) || '');
    const route = await this.router.route(input, {
      lastSession: follow ? follow.session : (this.session.lastActive && this.session.get(this.session.lastActive.id)),
      sessionPointers: this.session.list().slice(0, 6),
      followupHint: !!(follow && follow.followup),
      ...(ctx || {})
    });
    return { route, follow };
  }

  /**
   * Full orchestration: route → session → VRAM → adapter → compact handoff + telemetry.
   */
  async dispatch(input, ctx = {}) {
    const started = Date.now();
    const normalized = ctx.normalized || normalizeInput(input, { statFiles: ctx.statFiles, ...(ctx.normalizeOpts || {}) });
    const { route, follow } = await this.routeOnly(normalized, ctx);
    if (route.stage === 'fast_path') this.counters.fast_path_hits++;
    if (route.stage === 'llm_router') this.counters.llm_router_calls++;

    const steps = (route.steps && route.steps.length) ? route.steps : [{
      target_bot: route.target_bot,
      intent: route.intent,
      inputs: route.inputs
    }];

    const results = [];
    let lastArtifacts = [];
    for (let i = 0; i < steps.length; i++) {
      const step = steps[i];
      const target = step.target_bot;
      const spec = (this.config.specialists || {})[target] || { id: target, adapter: target === 'shell-agent' ? 'shell-agent' : target === 'ops' ? 'ops' : target === 'researcher' ? 'web' : 'ollama', tier: 'general_chat' };
      const reuseWanted = (i === 0 && route.reuse_session) || (follow && follow.session && follow.session.bot_id === target);
      let session, reused = false, created = false;
      if (reuseWanted) {
        const found = (follow && follow.session && follow.session.bot_id === target)
          ? follow.session
          : this.session.findByBotTopic(target, (follow && follow.session && follow.session.topic_key) || '');
        if (found) { session = found; reused = true; this.counters.session_reuses++; }
      }
      if (!session) {
        const got = this.session.getOrCreate(target, (step.inputs && step.inputs.text) || normalized.text, { reuse: true });
        session = got.session;
        reused = reused || got.reused;
        created = got.created;
        if (got.reused) this.counters.session_reuses++;
      }

      this.session.appendTurn(session.id, 'user', (step.inputs && step.inputs.text) || normalized.text);
      session = this.session.get(session.id);
      const payload = this.session.buildPromptPayload(session, { includeSharedFacts: created });

      const tierName = spec.tier || 'general_chat';
      const tier = (this.config.model_tiers && this.config.model_tiers[tierName]) || { preferred_models: ['qwen3:14b'], num_ctx: 4096, keep_alive: '5m' };
      const wantCtx = step.max_ctx || route.max_ctx || spec.max_ctx || tier.num_ctx || 4096;
      const plan = this.scheduler.selectModel(tier, { num_ctx: wantCtx });
      if (plan.swap) this.counters.model_swaps++;
      if (plan.evict && plan.evict.length) this.counters.evictions += plan.evict.length;

      const adapterName = spec.adapter || 'ollama';
      const adapter = this.adapters[adapterName] || this.adapters[target] || stubAdapter(adapterName);
      const task = {
        target_bot: target,
        intent: step.intent || route.intent,
        inputs: { ...(step.inputs || {}), prior_artifacts: lastArtifacts },
        text: (step.inputs && step.inputs.text) || normalized.text,
        normalized,
        session,
        prompt: payload,
        messages: [
          ...(payload.system_extra ? [{ role: 'system', content: (spec.card || '') + '\n\n' + payload.system_extra }] : [{ role: 'system', content: spec.card || '' }]),
          ...payload.messages
        ],
        model: plan.model,
        num_ctx: plan.num_ctx,
        keep_alive: plan.keep_alive,
        think: plan.think,
        temperature: plan.temperature,
        evict: plan.evict,
        vram: plan
      };

      let raw;
      try {
        raw = await adapter(task);
      } catch (err) {
        raw = {
          summary: 'Specialist ' + target + ' failed: ' + (err && err.message || err),
          artifacts: [],
          tool_events: [{ type: 'error', message: String(err && err.message || err) }],
          session_delta: {},
          tokens_used: { prompt_eval_count: 0, eval_count: 0, specialist_tokens: 0 },
          error: true
        };
      }

      // Never strip shell-agent / adapter guardrail fields.
      const passthrough = pickPassthrough(raw);

      const summary = raw && raw.summary != null ? String(raw.summary) : '';
      const appended = this.session.appendTurn(session.id, 'assistant', summary, { tokens: raw && raw.tokens_used });
      if (appended.compacted) this.counters.compactions++;
      if (raw && raw.session_delta && raw.session_delta.facts) {
        for (const [k, v] of Object.entries(raw.session_delta.facts)) this.session.pinFact(session.id, k, v);
      }

      lastArtifacts = (raw && raw.artifacts) || [];
      results.push({
        target_bot: target,
        intent: task.intent,
        summary,
        artifacts: lastArtifacts,
        tool_events: (raw && raw.tool_events) || [],
        session_delta: (raw && raw.session_delta) || {},
        tokens_used: (raw && raw.tokens_used) || { prompt_eval_count: 0, eval_count: 0, specialist_tokens: 0 },
        session_id: session.id,
        reused_session: reused,
        compacted: !!appended.compacted,
        vram: plan,
        adapter: adapterName,
        ...passthrough
      });
    }

    const specialistTokens = results.reduce((n, r) => n + ((r.tokens_used && (r.tokens_used.specialist_tokens || (r.tokens_used.prompt_eval_count || 0) + (r.tokens_used.eval_count || 0))) || 0), 0);
    const orchestratorTokens = (route.tokens && route.tokens.orchestrator_tokens) || 0;

    const handoff = {
      ok: true,
      summary: results.map((r) => r.summary).filter(Boolean).join('\n'),
      artifacts: results.flatMap((r) => r.artifacts || []),
      tool_events: results.flatMap((r) => r.tool_events || []),
      session_delta: results[results.length - 1] && results[results.length - 1].session_delta || {},
      tokens_used: {
        orchestrator_tokens: orchestratorTokens,
        specialist_tokens: specialistTokens,
        prompt_eval_count: (route.tokens && route.tokens.prompt_eval_count || 0) + results.reduce((n, r) => n + (r.tokens_used && r.tokens_used.prompt_eval_count || 0), 0),
        eval_count: (route.tokens && route.tokens.eval_count || 0) + results.reduce((n, r) => n + (r.tokens_used && r.tokens_used.eval_count || 0), 0)
      },
      route,
      steps: results,
      ...pickPassthrough(results.find((r) => r.approval_required || r.guardrail || r.blocked) || results[0])
    };

    const telem = {
      t: nowIso(),
      ms: Date.now() - started,
      target_bot: route.target_bot,
      intent: route.intent,
      stage: route.stage,
      model: (results[0] && results[0].vram && results[0].vram.model) || route.model,
      num_ctx: results[0] && results[0].vram && results[0].vram.num_ctx,
      keep_alive: results[0] && results[0].vram && results[0].vram.keep_alive,
      orchestrator_tokens: orchestratorTokens,
      specialist_tokens: specialistTokens,
      prompt_eval_count: handoff.tokens_used.prompt_eval_count,
      eval_count: handoff.tokens_used.eval_count,
      fast_path_hits: this.counters.fast_path_hits,
      session_reuses: this.counters.session_reuses,
      compactions: this.counters.compactions,
      model_swaps: this.counters.model_swaps,
      evictions: this.counters.evictions,
      reuse_session: !!route.reuse_session || results.some((r) => r.reused_session),
      approval_required: !!handoff.approval_required
    };
    try { appendJsonl(this.telemetryFile, telem); } catch { /* telemetry must never break dispatch */ }
    handoff.telemetry = telem;
    return handoff;
  }

  readTelemetry(n) { return readJsonlTail(this.telemetryFile, n); }
}

function createDispatcher(opts) { return new Dispatcher(opts); }

module.exports = {
  Dispatcher,
  createDispatcher,
  defaultOllamaAdapter,
  pickPassthrough,
  PASSTHROUGH
};
