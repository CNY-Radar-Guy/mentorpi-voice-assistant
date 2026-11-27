'use strict';

const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

const DEFAULTS = {
  maxRawTurns: 4,
  compactAfterTurns: 6,
  compactAfterTokens: 1800,
  followupIdleMs: 30 * 60 * 1000
};

function nowIso() { return new Date().toISOString(); }

function writeJSONAtomic(file, obj) {
  fs.mkdirSync(path.dirname(file), { recursive: true });
  const tmp = file + '.tmp';
  fs.writeFileSync(tmp, JSON.stringify(obj, null, 2) + '\n', 'utf8');
  fs.renameSync(tmp, file);
}

function readJSON(file, def) {
  try { return JSON.parse(fs.readFileSync(file, 'utf8')); } catch { return def; }
}

/** Rough token estimate: ~4 chars per token, plus a little overhead. */
function estimateTokens(text) {
  const s = typeof text === 'string' ? text : JSON.stringify(text || '');
  if (!s) return 0;
  return Math.ceil(s.length / 4);
}

function estimateSessionTokens(session) {
  let n = estimateTokens(session.summary || '') + estimateTokens(JSON.stringify(session.pinned_facts || {}));
  for (const turn of session.turns || []) n += estimateTokens(turn.content || '');
  return n;
}

function slugTopic(text) {
  const s = String(text || '')
    .toLowerCase()
    .replace(/https?:\/\/\S+/g, ' ')
    .replace(/@[a-z][\w.-]*/gi, ' ')
    .replace(/[^a-z0-9]+/g, ' ')
    .trim()
    .split(/\s+/)
    .filter((w) => w.length > 2 && !STOP.has(w))
    .slice(0, 8)
    .join('-');
  return s || 'general';
}

const STOP = new Set('the and for with you can please just this that from into about what how why when will would could should have been your our their its a an'.split(' '));

function rid() { return crypto.randomBytes(4).toString('hex'); }

function hashFacts(facts) {
  return crypto.createHash('sha256').update(JSON.stringify(facts || {})).digest('hex').slice(0, 12);
}

function tokenize(s) {
  return new Set(String(s || '').toLowerCase().replace(/[^a-z0-9]+/g, ' ').trim().split(/\s+/).filter((w) => w.length > 2 && !STOP.has(w)));
}

function jaccard(a, b) {
  if (!a.size || !b.size) return 0;
  let inter = 0;
  for (const x of a) if (b.has(x)) inter++;
  return inter / new Set([...a, ...b]).size;
}

function looksLikeFollowup(text) {
  const t = String(text || '').trim().toLowerCase();
  if (!t) return false;
  return /^(and |also |then |ok[,.]? |okay[,.]? |yes[,.]? |no[,.]? |please |now |again\b|retry|try again|same |that |this |it\b|the one|add |fix |change |update |continue|more\b|go on|what about|how about|and also)/i.test(t);
}

class SessionStore {
  constructor(opts = {}) {
    this.root = opts.root || process.cwd();
    this.indexPath = opts.indexPath || path.join(this.root, 'sessions.json');
    this.dir = opts.sessionsDir || path.join(this.root, 'sessions');
    this.sharedFactsPath = opts.sharedFactsPath || path.join(this.root, 'shared-facts.json');
    this.maxRawTurns = opts.maxRawTurns ?? DEFAULTS.maxRawTurns;
    this.compactAfterTurns = opts.compactAfterTurns ?? DEFAULTS.compactAfterTurns;
    this.compactAfterTokens = opts.compactAfterTokens ?? DEFAULTS.compactAfterTokens;
    this.followupIdleMs = opts.followupIdleMs ?? DEFAULTS.followupIdleMs;
    this.lastActive = null; // { id, bot_id, topic_key, updated }
    fs.mkdirSync(this.dir, { recursive: true });
    if (!fs.existsSync(this.indexPath)) writeJSONAtomic(this.indexPath, { sessions: [], last_active: null });
    if (!fs.existsSync(this.sharedFactsPath)) writeJSONAtomic(this.sharedFactsPath, { facts: {} });
  }

  _index() { return readJSON(this.indexPath, { sessions: [], last_active: null }); }

  _saveIndex(idx) { writeJSONAtomic(this.indexPath, idx); }

  _sessionPath(id) { return path.join(this.dir, id + '.json'); }

  _load(id) {
    const s = readJSON(this._sessionPath(id), null);
    return s;
  }

  _save(session) {
    writeJSONAtomic(this._sessionPath(session.id), session);
    const idx = this._index();
    const entry = {
      id: session.id,
      bot_id: session.bot_id,
      topic_key: session.topic_key,
      updated: session.updated,
      turn_count: (session.turns || []).length,
      compact_count: session.compact_count || 0,
      tokens_est: estimateSessionTokens(session)
    };
    const i = idx.sessions.findIndex((x) => x.id === session.id);
    if (i >= 0) idx.sessions[i] = entry; else idx.sessions.push(entry);
    idx.last_active = { id: session.id, bot_id: session.bot_id, topic_key: session.topic_key, updated: session.updated };
    this._saveIndex(idx);
    this.lastActive = idx.last_active;
    return session;
  }

  setSharedFacts(id, facts) {
    const store = readJSON(this.sharedFactsPath, { facts: {} });
    const hash = hashFacts(facts);
    store.facts[id] = { hash, facts, updated: nowIso() };
    writeJSONAtomic(this.sharedFactsPath, store);
    return { id, hash, ref: id + '#' + hash };
  }

  getSharedFacts(id) {
    const store = readJSON(this.sharedFactsPath, { facts: {} });
    return store.facts[id] || null;
  }

  listSharedFactRefs() {
    const store = readJSON(this.sharedFactsPath, { facts: {} });
    return Object.entries(store.facts).map(([id, v]) => ({ id, hash: v.hash, ref: id + '#' + v.hash }));
  }

  create(botId, topicKey, seedText) {
    const session = {
      id: (botId || 'general').replace(/[^a-z0-9_-]+/gi, '') + '-' + rid(),
      bot_id: botId,
      topic_key: topicKey || slugTopic(seedText),
      pinned_facts: {},
      summary: '',
      turns: [],
      compact_count: 0,
      created: nowIso(),
      updated: nowIso()
    };
    return this._save(session);
  }

  get(id) { return this._load(id); }

  list() { return this._index().sessions.slice().sort((a, b) => String(b.updated).localeCompare(String(a.updated))); }

  findByBotTopic(botId, topicKey) {
    const idx = this._index();
    const hits = idx.sessions.filter((s) => s.bot_id === botId && s.topic_key === topicKey);
    hits.sort((a, b) => String(b.updated).localeCompare(String(a.updated)));
    return hits[0] ? this._load(hits[0].id) : null;
  }

  getOrCreate(botId, text, opts = {}) {
    const topicKey = opts.topic_key || slugTopic(text);
    if (opts.reuse !== false) {
      const existing = this.findByBotTopic(botId, topicKey);
      if (existing) return { session: existing, created: false, reused: true, topic_key: topicKey };
    }
    return { session: this.create(botId, topicKey, text), created: true, reused: false, topic_key: topicKey };
  }

  /**
   * Session affinity: reuse the last active specialist when this looks like a follow-up
   * and topic similarity is high enough (or the utterance is clearly anaphoric).
   */
  findFollowup(text, opts = {}) {
    const idx = this._index();
    const last = opts.lastActive || this.lastActive || idx.last_active;
    if (!last || !last.id) return null;
    const idle = Date.now() - Date.parse(last.updated || 0);
    if (Number.isFinite(idle) && idle > this.followupIdleMs) return null;
    if (opts.bot_id && opts.bot_id !== last.bot_id) return null;
    const session = this._load(last.id);
    if (!session) return null;
    const follow = looksLikeFollowup(text);
    const sim = jaccard(tokenize(text), tokenize([session.topic_key, session.summary, ...(session.turns || []).map((t) => t.content)].join(' ')));
    const topicSim = jaccard(tokenize(text), tokenize(session.topic_key.replace(/-/g, ' ')));
    const ok = follow || sim >= 0.18 || topicSim >= 0.34;
    if (!ok) return null;
    return { session, similarity: Math.max(sim, topicSim), followup: follow, idle_ms: idle };
  }

  pinFact(sessionId, key, value) {
    const session = this._load(sessionId);
    if (!session) throw new Error('unknown session ' + sessionId);
    session.pinned_facts = session.pinned_facts || {};
    session.pinned_facts[key] = value;
    session.updated = nowIso();
    return this._save(session);
  }

  appendTurn(sessionId, role, content, meta) {
    const session = this._load(sessionId);
    if (!session) throw new Error('unknown session ' + sessionId);
    session.turns = session.turns || [];
    session.turns.push({ role, content: String(content || ''), at: nowIso(), ...(meta || {}) });
    session.updated = nowIso();
    const compacted = this.compactIfNeeded(session);
    return { session: this._save(compacted.session), compacted: compacted.compacted, tokens_saved: compacted.tokens_saved };
  }

  /**
   * Extractive compaction (no LLM required): fold older turns into a rolling summary,
   * keep only the last maxRawTurns verbatim.
   */
  compactIfNeeded(session, force) {
    const turns = session.turns || [];
    const tokens = estimateSessionTokens(session);
    const overTurns = turns.length > this.compactAfterTurns;
    const overTokens = tokens > this.compactAfterTokens;
    if (!force && !overTurns && !overTokens) {
      return { session, compacted: false, tokens_saved: 0 };
    }
    if (turns.length <= this.maxRawTurns) {
      return { session, compacted: false, tokens_saved: 0 };
    }
    const keep = turns.slice(-this.maxRawTurns);
    const older = turns.slice(0, -this.maxRawTurns);
    const lines = older.map((t) => {
      const clip = String(t.content || '').replace(/\s+/g, ' ').trim().slice(0, 160);
      return `- ${t.role}: ${clip}`;
    });
    const addition = lines.join('\n');
    const prev = session.summary ? session.summary.trim() + '\n' : '';
    const before = tokens;
    session.summary = (prev + addition).trim();
    session.turns = keep;
    session.compact_count = (session.compact_count || 0) + 1;
    session.updated = nowIso();
    const after = estimateSessionTokens(session);
    return { session, compacted: true, tokens_saved: Math.max(0, before - after) };
  }

  /**
   * Minimal prompt payload for a specialist: pinned facts + summary + last N turns
   * + shared-fact refs (not the full shared blob unless requested).
   */
  buildPromptPayload(session, opts = {}) {
    const includeShared = opts.includeSharedFacts === true;
    const refs = this.listSharedFactRefs();
    const known = new Set(Object.keys(session.pinned_facts || {}).filter((k) => k.startsWith('shared:')));
    const shared_fact_ids = refs.map((r) => r.ref);
    let shared_facts = null;
    if (includeShared) {
      const store = readJSON(this.sharedFactsPath, { facts: {} });
      shared_facts = store.facts;
    }
    const messages = (session.turns || []).map((t) => ({ role: t.role, content: t.content }));
    const system_extra_parts = [];
    if (session.pinned_facts && Object.keys(session.pinned_facts).length) {
      system_extra_parts.push('Pinned facts:\n' + Object.entries(session.pinned_facts).map(([k, v]) => `- ${k}: ${typeof v === 'string' ? v : JSON.stringify(v)}`).join('\n'));
    }
    if (session.summary) system_extra_parts.push('Rolling summary:\n' + session.summary);
    if (shared_fact_ids.length) {
      system_extra_parts.push('Shared machine facts (by id/hash; do not echo unless needed): ' + shared_fact_ids.join(', '));
      // Inject full blob only when the session has not yet pinned this hash.
      if (includeShared) {
        for (const ref of refs) {
          if (!known.has('shared:' + ref.ref) && shared_facts[ref.id]) {
            system_extra_parts.push(`Shared facts ${ref.ref}:\n` + JSON.stringify(shared_facts[ref.id].facts));
          }
        }
      }
    }
    const payload = {
      session_id: session.id,
      bot_id: session.bot_id,
      topic_key: session.topic_key,
      system_extra: system_extra_parts.join('\n\n'),
      messages,
      shared_fact_ids,
      tokens_est: estimateTokens(system_extra_parts.join('\n')) + messages.reduce((n, m) => n + estimateTokens(m.content), 0)
    };
    return payload;
  }

  /** Token count of a naive full-history replay (for savings tests / telemetry). */
  naiveReplayTokens(session) {
    const all = [];
    if (session.summary) all.push(session.summary);
    // Reconstruct a worst-case "everything ever said" is not stored; compare against
    // summary+all current turns AND a synthetic full dump if raw_archive exists.
    for (const t of session.turns || []) all.push(t.content);
    if (session._naive_archive) all.push(session._naive_archive);
    return estimateTokens(all.join('\n'));
  }
}

function createSessionStore(opts) { return new SessionStore(opts); }

module.exports = {
  SessionStore,
  createSessionStore,
  estimateTokens,
  estimateSessionTokens,
  slugTopic,
  hashFacts,
  looksLikeFollowup,
  DEFAULTS
};
