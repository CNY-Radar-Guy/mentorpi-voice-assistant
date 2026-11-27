'use strict';

const DEFAULT_WEIGHTS = {
  'llama3.3:70b': 42000,
  'qwen2.5-coder:32b': 19000,
  'deepseek-r1:32b': 19000,
  'qwen3:32b': 20000,
  'gemma3:27b': 17000,
  'mistral-small:24b': 14000,
  'phi4:latest': 9100,
  'phi4': 9100,
  'deepseek-r1:14b': 9000,
  'qwen3:14b': 9300,
  'qwen3.5:4b': 3300
};

function normName(name) {
  return String(name || '').trim();
}

function aliases(name) {
  const n = normName(name);
  const out = new Set([n]);
  if (n.endsWith(':latest')) out.add(n.slice(0, -7));
  else if (!n.includes(':')) out.add(n + ':latest');
  return out;
}

function namesMatch(a, b) {
  const A = aliases(a), B = aliases(b);
  for (const x of A) if (B.has(x)) return true;
  return false;
}

function inList(name, list) {
  return (list || []).some((x) => namesMatch(x, name));
}

class VramScheduler {
  constructor(opts = {}) {
    this.budgetMb = opts.budgetMb ?? 16384;
    this.reserveMb = opts.reserveMb ?? 1536;
    this.weights = { ...DEFAULT_WEIGHTS, ...(opts.weights || {}) };
    this.heavyModels = opts.heavyModels || Object.entries(this.weights).filter(([, mb]) => mb >= 16000).map(([n]) => n);
    this.fitPolicy = opts.fitPolicy || 'prefer_in_vram';
    this.allowPartialOffload = opts.allowPartialOffload === true;
    this.reuseLoadedIfCompatible = opts.reuseLoadedIfCompatible !== false;
    this.ollamaBaseUrl = (opts.ollamaBaseUrl || 'http://127.0.0.1:11434').replace(/\/$/, '');
    this.fetchImpl = opts.fetchImpl || (typeof fetch === 'function' ? fetch : null);
    this.installed = opts.installed || [];
    this.loaded = opts.loaded || [];
  }

  usableMb() { return Math.max(0, this.budgetMb - this.reserveMb); }

  weightMb(model) {
    const n = normName(model);
    if (this.weights[n] != null) return this.weights[n];
    if (this.weights[n.replace(/:latest$/, '')] != null) return this.weights[n.replace(/:latest$/, '')];
    if (this.weights[n + ':latest'] != null) return this.weights[n + ':latest'];
    // Unknown: infer a conservative size from the parameter tag if present.
    const m = n.match(/:(\d+)b\b/i);
    if (m) return Number(m[1]) * 700; // rough Q4 guess
    return 8000;
  }

  isHeavy(model) { return inList(model, this.heavyModels) || this.weightMb(model) >= 16000; }

  /**
   * Very rough KV-cache estimate: scales with weights and context.
   * A 9 GB 14B model at 4k ctx is ~0.5–1 GB; at 16k a couple of GB.
   */
  kvCacheMb(model, numCtx) {
    const w = this.weightMb(model);
    const ctx = Math.max(512, Number(numCtx) || 4096);
    return Math.ceil((w / 9300) * (ctx / 4096) * 700);
  }

  estimateModelVramMb(model, numCtx) {
    return this.weightMb(model) + this.kvCacheMb(model, numCtx);
  }

  fitsAlone(model, numCtx) {
    return this.estimateModelVramMb(model, numCtx) <= this.usableMb();
  }

  setInstalled(list) { this.installed = list || []; }
  setLoaded(list) { this.loaded = list || []; }

  async refreshFromOllama(timeoutMs) {
    const tags = await this._getJson('/api/tags', timeoutMs || 2500);
    const ps = await this._getJson('/api/ps', timeoutMs || 2500);
    this.installed = ((tags && tags.models) || []).map((m) => m.name || m.model).filter(Boolean);
    this.loaded = ((ps && ps.models) || []).map((m) => ({
      name: m.name || m.model,
      size_mb: m.size ? Math.round(m.size / (1024 * 1024)) : null,
      context: (m.context && (m.context.length || m.context)) || m.context_length || null,
      expires_at: m.expires_at || null
    }));
    return { installed: this.installed, loaded: this.loaded, raw_tags: tags, raw_ps: ps };
  }

  async _getJson(pathname, ms) {
    if (!this.fetchImpl) throw new Error('no fetch implementation');
    const ac = typeof AbortController !== 'undefined' ? new AbortController() : null;
    const t = ac ? setTimeout(() => ac.abort(), ms) : null;
    try {
      const r = await this.fetchImpl(this.ollamaBaseUrl + pathname, ac ? { signal: ac.signal } : {});
      if (!r.ok) throw new Error('HTTP ' + r.status);
      return await r.json();
    } finally {
      if (t) clearTimeout(t);
    }
  }

  isInstalled(model, installed) {
    const list = installed || this.installed;
    return inList(model, list);
  }

  loadedEntry(model, loaded) {
    const list = loaded || this.loaded;
    return list.find((m) => namesMatch(typeof m === 'string' ? m : m.name, model)) || null;
  }

  loadedVramMb(loaded) {
    const list = loaded || this.loaded;
    let sum = 0;
    for (const m of list) {
      if (typeof m === 'object' && m.size_mb) sum += m.size_mb;
      else sum += this.estimateModelVramMb(typeof m === 'string' ? m : m.name, (m && m.context) || 4096);
    }
    return sum;
  }

  /**
   * Pick the best installed model for a tier without unnecessary swaps.
   * @param {object} tier  { preferred_models, num_ctx, num_ctx_max, keep_alive, think, temperature }
   * @param {object} [opts]
   */
  selectModel(tier, opts = {}) {
    const preferred = (tier && tier.preferred_models) || [];
    const installed = opts.installed || this.installed;
    const loaded = opts.loaded || this.loaded;
    const allowOffload = opts.allowPartialOffload != null ? opts.allowPartialOffload : this.allowPartialOffload;
    const forceModel = opts.forceModel;
    const wantCtx = Math.min(opts.num_ctx || (tier && tier.num_ctx) || 4096, (tier && tier.num_ctx_max) || opts.num_ctx_max || 16384);

    if (forceModel) {
      if (!this.isInstalled(forceModel, installed)) {
        // fall through to normal selection
      } else {
        return this._plan(forceModel, wantCtx, tier, loaded, { reason: 'forced', allowOffload });
      }
    }

    // 1. Reuse a loaded model that is in the preferred (or compatible) list.
    if (this.reuseLoadedIfCompatible) {
      for (const pref of preferred) {
        const hit = this.loadedEntry(pref, loaded);
        if (hit && this.isInstalled(pref, installed)) {
          const name = typeof hit === 'string' ? hit : hit.name;
          return this._plan(name, wantCtx, tier, loaded, { reason: 'reuse_loaded', swap: false });
        }
      }
    }

    // 2. Walk the preferred chain. Skip models that are not installed.
    //    In prefer_in_vram mode, skip models that would not fit (unless offload allowed).
    const candidates = [];
    for (const pref of preferred) {
      const name = this._resolveInstalledName(pref, installed);
      if (!name) continue;
      const est = this.estimateModelVramMb(name, wantCtx);
      const fits = est <= this.usableMb();
      candidates.push({ name, est, fits, heavy: this.isHeavy(name) });
    }

    let picked = null;
    if (allowOffload && candidates.length) picked = candidates[0];
    else picked = candidates.find((c) => c.fits);
    if (!picked && candidates.length && this.fitPolicy !== 'prefer_in_vram') picked = candidates[0];
    // Last resort: any installed model that fits, preferring smaller.
    if (!picked) {
      const fallbacks = (installed || []).map((n) => ({
        name: n,
        est: this.estimateModelVramMb(n, wantCtx),
        fits: this.fitsAlone(n, wantCtx),
        heavy: this.isHeavy(n)
      })).filter((c) => c.fits).sort((a, b) => a.est - b.est);
      picked = fallbacks[0];
    }
    if (!picked) {
      return {
        ok: false,
        model: null,
        reason: 'no_fitting_model',
        evict: [],
        num_ctx: wantCtx,
        keep_alive: (tier && tier.keep_alive) || '5m',
        think: !!(tier && tier.think),
        warning: 'No installed model fits the ' + this.budgetMb + ' MB VRAM budget.'
      };
    }

    const already = !!this.loadedEntry(picked.name, loaded);
    return this._plan(picked.name, wantCtx, tier, loaded, {
      reason: already ? 'reuse_loaded' : (picked.fits ? 'fit_preferred' : 'offload_allowed'),
      swap: !already
    });
  }

  _resolveInstalledName(pref, installed) {
    const hit = (installed || []).find((n) => namesMatch(n, pref));
    return hit || null;
  }

  _plan(model, numCtx, tier, loaded, meta) {
    const evict = [];
    const already = this.loadedEntry(model, loaded);
    const est = this.estimateModelVramMb(model, numCtx);
    const others = (loaded || []).filter((m) => !namesMatch(typeof m === 'string' ? m : m.name, model));
    const othersMb = this.loadedVramMb(others);
    let warning = null;

    if (!already) {
      const room = this.usableMb() - othersMb;
      if (est > room) {
        // Evict heavies first, then anything that isn't the daily driver if still needed.
        const evictables = others.slice().sort((a, b) => {
          const an = typeof a === 'string' ? a : a.name;
          const bn = typeof b === 'string' ? b : b.name;
          return Number(this.isHeavy(bn)) - Number(this.isHeavy(an));
        });
        let freed = 0;
        for (const m of evictables) {
          if (est <= room + freed) break;
          const name = typeof m === 'string' ? m : m.name;
          evict.push({ name, keep_alive: '0', reason: 'make_room', size_mb: this.estimateModelVramMb(name, (m && m.context) || 4096) });
          freed += evict[evict.length - 1].size_mb;
        }
      }
    }

    if (!this.fitsAlone(model, numCtx)) {
      warning = model + ' is estimated at ' + est + ' MB and will spill past the ' + this.budgetMb + ' MB VRAM budget into system RAM.';
    }

    const keepAlive = evict.length && this.isHeavy(model) ? ((tier && tier.keep_alive) || '5m') : ((tier && tier.keep_alive) || '5m');

    return {
      ok: true,
      model,
      num_ctx: numCtx,
      keep_alive: keepAlive,
      think: !!(tier && tier.think),
      temperature: tier && tier.temperature,
      num_predict: tier && tier.num_predict,
      estimated_vram_mb: est,
      evict,
      swap: !!meta.swap && evict.length > 0 || (!!meta.swap && !already),
      reuse: !!already,
      reason: meta.reason,
      warning
    };
  }

  /**
   * Convenience: compute options for an already-chosen model + tier.
   */
  computeOptions(tier, model, opts) {
    return this.selectModel({ ...(tier || {}), preferred_models: [model].concat((tier && tier.preferred_models) || []) }, { ...(opts || {}), forceModel: model });
  }
}

function createVramScheduler(opts) { return new VramScheduler(opts); }

module.exports = {
  VramScheduler,
  createVramScheduler,
  DEFAULT_WEIGHTS,
  namesMatch
};
