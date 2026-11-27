'use strict';

const { normalizeInput, BOT_ALIASES } = require('./input-normalizer');

const DEFAULT_CONFIDENCE = 0.75;

const SPECIALIST_IDS = ['shell-agent', 'coder', 'researcher', 'ops', 'general'];

const RULES = [
  {
    id: 'shell-agent',
    weight: 0.95,
    test: (t, n) => {
      if (/\b(winget|choco|chocolatey|msiexce?c|apt-get|brew install)\b/i.test(t)) return 0.98;
      if (/\b(install|uninstall|reinstall|download and install|set up|setup)\b.{0,80}\b(app|application|software|program|package|studio|tool|driver)\b/i.test(t)) return 0.96;
      if (/\b(download|install|uninstall|upgrade)\b.{0,40}\b([A-Z][\w]+(?:\s+[A-Z][\w]+)*)\b/i.test(t)) return 0.9;
      if (/\b(install|download)\b.+\b(studio|vscode|chrome|bambu|python|node|ollama)\b/i.test(t)) return 0.96;
      if (/\b(run this command|run the command|execute (this|the) command|open powershell|in (a )?powershell|cmd\.exe|from the shell)\b/i.test(t)) return 0.94;
      if (/\b(shell-agent|guarded shell)\b/i.test(t)) return 0.99;
      return 0;
    }
  },
  {
    id: 'coder',
    weight: 0.92,
    test: (t, n) => {
      if (/\b(openscad|scad)\b/i.test(t)) return 0.97;
      if (/\b(stack trace|typeerror|syntaxerror|nullpointer|traceback)\b/i.test(t)) return 0.94;
      if (/\b(debug|refactor|unit test|write (a |an )?(function|script|class|module|program|macro)|fix this (code|script|function|bug)|code review|boilerplate)\b/i.test(t)) return 0.92;
      if (/\b(python|javascript|typescript|powershell|bash|regex|html|css|json schema)\b.+\b(script|function|code|class|module|bug|error|fix|write|generate)\b/i.test(t)) return 0.9;
      if (/\b(write|generate|fix|debug|explain)\b.+\b(code|script|function|class|program)\b/i.test(t)) return 0.9;
      if (n.file_paths.some((f) => /\.(js|ts|py|ps1|scad|c|cpp|h|rs|go|java|sh|css|html|json)$/i.test(f.path || ''))) return 0.88;
      return 0;
    }
  },
  {
    id: 'researcher',
    weight: 0.92,
    test: (t, n) => {
      if (n.urls.length) return 0.95;
      if (/\b(search the web|look up|lookup|google|brave search|browse|from the web|on the web|web search)\b/i.test(t)) return 0.98;
      if (/\b(what'?s the latest|latest news|find (a |an )?url|fetch the page|summarize (this|the) (page|article|url|site))\b/i.test(t)) return 0.9;
      return 0;
    }
  },
  {
    id: 'ops',
    weight: 0.93,
    test: (t) => {
      // Do not steal web-research queries that merely mention hardware as the topic.
      if (/\b(search the web|look up|lookup|brave search|on the web|web search|https?:\/\/)\b/i.test(t)) return 0;
      if (/\b(nvidia-smi|gpu memory|gpu (usage|util|status|temp)|vram (usage|free|used|status))\b/i.test(t)) return 0.97;
      if (/\b(wally|mentorpi|tailscale)\b/i.test(t)) return 0.96;
      if (/\b(disk space|free space|ram usage|memory usage|cpu (load|usage|temp)|system status|how'?s the (gpu|ram|cpu|disk|system|vram)|loaded models|ollama ps)\b/i.test(t)) return 0.94;
      if (/\b(check|show|report|status of)\b.+\b(gpu|vram|ram|disk|cpu|tailscale|wally|ollama|temperature|temps)\b/i.test(t)) return 0.9;
      if (/\b(rtx 5080|vram)\b/i.test(t) && /\b(check|status|usage|free|used|how|alpha-one|this (pc|machine|box))\b/i.test(t)) return 0.9;
      return 0;
    }
  }
];

const FOLLOWUP_RE = /^(and |also |then |ok[,.]? |okay[,.]? |yes[,.]? |please |now |again\b|retry|try again|same |add |fix |change |update |continue|more|go on|what about|how about)/i;

function scoreRules(text, normalized) {
  const scores = { 'shell-agent': 0, coder: 0, researcher: 0, ops: 0, general: 0 };
  for (const rule of RULES) {
    const s = rule.test(text, normalized) || 0;
    if (s > scores[rule.id]) scores[rule.id] = s;
  }
  // Weak general baseline so something always wins at low confidence.
  if (Object.values(scores).every((v) => v === 0)) scores.general = 0.4;
  else scores.general = 0.15;
  return scores;
}

function topScore(scores) {
  let id = 'general', best = -1, second = -1;
  for (const [k, v] of Object.entries(scores)) {
    if (v > best) { second = best; best = v; id = k; }
    else if (v > second) second = v;
  }
  return { id, score: best, margin: best - Math.max(0, second) };
}

function specialistCards(specialists) {
  const list = specialists || {};
  return Object.keys(list).map((id) => {
    const s = list[id];
    return `- ${id}: ${(s && s.card) || id}`;
  }).join('\n');
}

function sessionPointers(pointers) {
  if (!pointers || !pointers.length) return 'none';
  return pointers.map((p) => {
    const age = p.age || p.updated || '';
    const turns = p.turn_count != null ? `${p.turn_count} turns` : '';
    return `${p.bot_id}#${String(p.id || '').slice(-8)} "${p.topic_key || ''}" (${[turns, age].filter(Boolean).join(', ')})`;
  }).join('; ');
}

function partsToSteps(parts) {
  const steps = [];
  for (const part of parts) {
    const n = normalizeInput(part, { statFiles: false });
    const sc = scoreRules(n.text, n);
    const top = topScore(sc);
    if (top.score >= 0.7) steps.push({ target_bot: top.id, intent: inferIntent(top.id, n.text), inputs: { text: n.text }, confidence: top.score });
  }
  const bots = new Set(steps.map((s) => s.target_bot));
  if (steps.length >= 2 && bots.size >= 2) return steps;
  return null;
}

function splitCompound(normalized, scores) {
  const stepsText = (normalized.hints && normalized.hints.steps) || normalized.steps || [];
  const fromHints = stepsText.length ? partsToSteps(stepsText) : null;
  if (fromHints) return fromHints;
  // Also split a single "and" when the two sides clearly belong to different specialists
  // (e.g. "check disk space and install VLC").
  const text = String(normalized.text || '');
  const andParts = text.split(/\s+\band\s+/i).map((s) => s.trim()).filter(Boolean);
  if (andParts.length === 2) return partsToSteps(andParts);
  return null;
}

function inferIntent(bot, text) {
  const t = String(text || '').toLowerCase();
  if (bot === 'shell-agent') {
    if (/\binstall\b/.test(t)) return 'install_software';
    if (/\buninstall\b/.test(t)) return 'uninstall_software';
    if (/\bdownload\b/.test(t)) return 'download';
    return 'shell_command';
  }
  if (bot === 'coder') {
    if (/\bdebug|fix|error|bug\b/.test(t)) return 'debug_code';
    if (/\bopenscad|scad\b/.test(t)) return 'openscad';
    return 'write_code';
  }
  if (bot === 'researcher') return /\bhttps?:\/\//.test(t) ? 'fetch_url' : 'web_search';
  if (bot === 'ops') {
    if (/\bwally|mentorpi\b/.test(t)) return 'wally_status';
    if (/\btailscale\b/.test(t)) return 'tailscale_status';
    if (/\bgpu|vram|nvidia\b/.test(t)) return 'gpu_status';
    if (/\bdisk\b/.test(t)) return 'disk_status';
    return 'system_status';
  }
  return 'chat';
}

function buildRoute({ target, confidence, stage, normalized, intent, reuse_session, needs_handoff, max_ctx, steps, reason, model }) {
  return {
    target_bot: target,
    intent: intent || inferIntent(target, normalized.text),
    inputs: {
      text: normalized.text,
      mentions: normalized.mentions,
      urls: normalized.urls,
      files: (normalized.file_paths || []).map((f) => ({
        path: f.path, exists: f.exists, size: f.size, hash: f.hash, truncated: f.truncated,
        excerpt: f.excerpt ? String(f.excerpt).slice(0, 800) : null
      })),
      json: normalized.json_payload,
      preview: normalized.preview
    },
    reuse_session: !!reuse_session,
    needs_handoff: !!needs_handoff || !!(steps && steps.length > 1),
    max_ctx: max_ctx || null,
    steps: steps || null,
    confidence,
    stage,
    reason: reason || stage,
    model: model || null,
    tokens: { prompt_eval_count: 0, eval_count: 0, orchestrator_tokens: 0 }
  };
}

const ROUTER_SCHEMA = {
  type: 'object',
  properties: {
    target_bot: { type: 'string', enum: SPECIALIST_IDS },
    intent: { type: 'string' },
    inputs: { type: 'object' },
    reuse_session: { type: 'boolean' },
    needs_handoff: { type: 'boolean' },
    max_ctx: { type: 'integer' },
    steps: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          target_bot: { type: 'string', enum: SPECIALIST_IDS },
          intent: { type: 'string' },
          inputs: { type: 'object' }
        }
      }
    }
  },
  required: ['target_bot', 'intent', 'reuse_session']
};

function routerPrompt({ text, specialists, pointers }) {
  return [
    'You are a compact router for Jarvis on ALPHA-ONE. Pick one specialist. Reply with JSON only. No prose.',
    'Specialists:',
    specialistCards(specialists) || [
      '- shell-agent: local commands and software installs (guarded, may need approval)',
      '- coder: code, debug, OpenSCAD, scripts',
      '- researcher: web/URL lookup via Brave',
      '- ops: GPU/RAM/disk, Wally, Tailscale, services (prefer probes)',
      '- general: conversation / fallback'
    ].join('\n'),
    'Active sessions: ' + sessionPointers(pointers),
    'User: ' + String(text || '').slice(0, 800)
  ].join('\n');
}

function extractJson(s) {
  const t = String(s || '').replace(/<think>[\s\S]*?<\/think>/g, '').trim();
  const start = t.indexOf('{');
  const end = t.lastIndexOf('}');
  if (start < 0 || end < start) return null;
  try { return JSON.parse(t.slice(start, end + 1)); } catch { return null; }
}

class Router {
  constructor(opts = {}) {
    this.config = opts.config || {};
    this.specialists = (this.config.specialists) || {};
    this.threshold = (this.config.router && this.config.router.fast_path_confidence) || DEFAULT_CONFIDENCE;
    this.followupConfidence = (this.config.router && this.config.router.followup_confidence) || 0.8;
    this.ollamaChatJSON = opts.ollamaChatJSON || null;
    this.scheduler = opts.scheduler || null;
    this.aliasMap = { ...BOT_ALIASES };
    for (const [id, spec] of Object.entries(this.specialists)) {
      this.aliasMap[id] = id;
      for (const a of spec.aliases || []) this.aliasMap[String(a).toLowerCase()] = id;
    }
  }

  /**
   * @param {string|object} input
   * @param {object} [ctx]  { sessionPointers, lastSession, normalizeOpts }
   */
  async route(input, ctx = {}) {
    const normalized = ctx.normalized || normalizeInput(input, { aliases: this.aliasMap, statFiles: ctx.statFiles, ...(ctx.normalizeOpts || {}) });
    const text = normalized.text;

    // Explicit @bot always wins (Stage 0, confidence 1.0).
    if (normalized.mentions && normalized.mentions.length) {
      const target = normalized.mentions[0];
      return buildRoute({
        target, confidence: 1, stage: 'fast_path', normalized,
        reuse_session: !!(ctx.lastSession && ctx.lastSession.bot_id === target),
        reason: 'explicit_mention'
      });
    }

    // Compound DAG split (still 0 tokens when each part is obvious).
    const compound = splitCompound(normalized, null);
    if (compound) {
      return buildRoute({
        target: compound[0].target_bot,
        confidence: Math.min(...compound.map((s) => s.confidence)),
        stage: 'fast_path',
        normalized,
        intent: 'multi_step',
        needs_handoff: true,
        steps: compound,
        reason: 'compound_fast_path'
      });
    }

    const scores = scoreRules(text, normalized);
    const top = topScore(scores);

    // Session affinity / follow-up.
    if (ctx.lastSession && ctx.lastSession.bot_id) {
      const follow = FOLLOWUP_RE.test(text.trim()) || ctx.followupHint === true;
      if (follow) {
        return buildRoute({
          target: ctx.lastSession.bot_id,
          confidence: this.followupConfidence,
          stage: 'fast_path',
          normalized,
          reuse_session: true,
          reason: 'session_followup'
        });
      }
    }

    if (top.score >= this.threshold) {
      const reuse = !!(ctx.lastSession && ctx.lastSession.bot_id === top.id);
      const maxCtx = (this.specialists[top.id] && this.specialists[top.id].max_ctx) || null;
      return buildRoute({
        target: top.id, confidence: top.score, stage: 'fast_path', normalized,
        reuse_session: reuse, max_ctx: maxCtx, reason: 'deterministic_rule'
      });
    }

    // Stage 1 — compact LLM classifier.
    const llm = await this._llmRoute(normalized, ctx, scores, top);
    return llm;
  }

  async _llmRoute(normalized, ctx, scores, top) {
    const fallback = () => buildRoute({
      target: top.id || 'general',
      confidence: top.score || 0.4,
      stage: 'fallback',
      normalized,
      reuse_session: !!(ctx.lastSession && ctx.lastSession.bot_id === (top.id || 'general')),
      reason: 'llm_unavailable_or_unparseable'
    });

    if (typeof this.ollamaChatJSON !== 'function') return fallback();

    const tier = (this.config.model_tiers && this.config.model_tiers.fast_router) || {
      preferred_models: ['qwen3.5:4b', 'qwen3:14b'], num_ctx: 2048, num_predict: 120, keep_alive: '5m', think: false, temperature: 0
    };
    const plan = this.scheduler ? this.scheduler.selectModel(tier) : { model: (tier.preferred_models || [])[0], num_ctx: 2048, num_predict: 120, keep_alive: '5m', think: false, evict: [] };
    if (!plan || !plan.model) return fallback();

    const messages = [
      { role: 'system', content: 'Return only a compact JSON object that matches the schema. No markdown.' },
      { role: 'user', content: routerPrompt({ text: (normalized.preview && normalized.preview.text) || normalized.text, specialists: this.specialists, pointers: ctx.sessionPointers || [] }) }
    ];

    try {
      const result = await this.ollamaChatJSON({
        model: plan.model,
        messages,
        schema: ROUTER_SCHEMA,
        options: { temperature: tier.temperature ?? 0, num_ctx: 2048, num_predict: 120 },
        think: false,
        keep_alive: plan.keep_alive || '5m',
        evict: plan.evict || []
      });
      const parsed = (result && result.json) || extractJson(result && (result.content || result.message || result));
      if (!parsed || !parsed.target_bot || SPECIALIST_IDS.indexOf(parsed.target_bot) < 0) {
        const fb = fallback();
        fb.tokens = tokenFields(result, 'llm_router_unparseable');
        return fb;
      }
      const route = buildRoute({
        target: parsed.target_bot,
        confidence: 0.7,
        stage: 'llm_router',
        normalized,
        intent: parsed.intent,
        reuse_session: !!parsed.reuse_session,
        needs_handoff: !!parsed.needs_handoff,
        max_ctx: parsed.max_ctx || null,
        steps: parsed.steps || null,
        reason: 'compact_llm_router',
        model: plan.model
      });
      route.tokens = tokenFields(result, 'llm_router');
      route.model = plan.model;
      route.vram = { evict: plan.evict || [], reason: plan.reason, estimated_vram_mb: plan.estimated_vram_mb };
      return route;
    } catch (err) {
      const fb = fallback();
      fb.reason = 'llm_error:' + (err && err.message || err);
      return fb;
    }
  }
}

function tokenFields(result, _stage) {
  const p = (result && (result.prompt_eval_count ?? result.promptEvalCount)) || 0;
  const e = (result && (result.eval_count ?? result.evalCount)) || 0;
  return { prompt_eval_count: p, eval_count: e, orchestrator_tokens: p + e };
}

function createRouter(opts) { return new Router(opts); }

module.exports = {
  Router,
  createRouter,
  scoreRules,
  topScore,
  splitCompound,
  inferIntent,
  routerPrompt,
  extractJson,
  ROUTER_SCHEMA,
  SPECIALIST_IDS,
  RULES
};
