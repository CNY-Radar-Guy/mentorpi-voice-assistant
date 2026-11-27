'use strict';

const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

const DEFAULTS = {
  maxExcerptChars: 800,
  maxFileBytes: 256 * 1024,
  hashBytes: 64 * 1024
};

const BOT_ALIASES = {
  'shell-agent': 'shell-agent',
  shell: 'shell-agent',
  sh: 'shell-agent',
  install: 'shell-agent',
  coder: 'coder',
  code: 'coder',
  dev: 'coder',
  programmer: 'coder',
  researcher: 'researcher',
  research: 'researcher',
  web: 'researcher',
  search: 'researcher',
  'web-research': 'researcher',
  ops: 'ops',
  sys: 'ops',
  system: 'ops',
  status: 'ops',
  'ops-monitor': 'ops',
  general: 'general',
  jarvis: 'general',
  chat: 'general'
};

const URL_RE = /\bhttps?:\/\/[^\s<>"'`]+/gi;
const MENTION_RE = /(?:^|[^\w@])@([a-zA-Z][\w.-]*)/g;
const WIN_PATH_RE = /\b(?:file:\/\/\/)?([A-Za-z]:\\[^\s"'<>|*?]+)/g;
const UNC_PATH_RE = /(\\\\[^\s"'<>|*?]+)/g;
const POSIX_PATH_RE = /(?:^|[\s"'`(])(\/(?:[\w.-]+\/)+[\w.-]+)/g;
const MULTI_STEP_RE = /\b(?:and then|then|after that|afterwards|next,|followed by|secondly)\b|\s*(?:->|→|->)\s*/i;

function unique(list) {
  const seen = new Set();
  const out = [];
  for (const item of list) {
    const key = typeof item === 'string' ? item : JSON.stringify(item);
    if (seen.has(key)) continue;
    seen.add(key);
    out.push(item);
  }
  return out;
}

function stripTrailingPunct(s) {
  return String(s || '').replace(/[.,;:!?)]+$/g, '');
}

function extractUrls(text) {
  const urls = [];
  String(text || '').replace(URL_RE, (m) => {
    urls.push(stripTrailingPunct(m));
    return m;
  });
  return unique(urls);
}

function extractMentions(text, aliasMap) {
  const map = aliasMap || BOT_ALIASES;
  const mentions = [];
  const re = new RegExp(MENTION_RE.source, 'g');
  let m;
  const src = String(text || '');
  while ((m = re.exec(src))) {
    const raw = m[1].toLowerCase();
    const id = map[raw] || (Object.prototype.hasOwnProperty.call(map, raw) ? map[raw] : null);
    // Only treat as a bot mention if it maps to a known specialist (or looks like a bot id).
    if (id) mentions.push(id);
  }
  return unique(mentions);
}

function extractPaths(text) {
  const paths = [];
  const src = String(text || '');
  // Skip URL-looking slices so https:// is not treated as a path.
  const withoutUrls = src.replace(URL_RE, ' ');
  let m;
  const win = new RegExp(WIN_PATH_RE.source, 'g');
  while ((m = win.exec(withoutUrls))) paths.push(stripTrailingPunct((m[1] || m[0]).replace(/^file:\/\/\//, '')));
  const unc = new RegExp(UNC_PATH_RE.source, 'g');
  while ((m = unc.exec(withoutUrls))) paths.push(stripTrailingPunct(m[1] || m[0]));
  const posix = new RegExp(POSIX_PATH_RE.source, 'g');
  while ((m = posix.exec(withoutUrls))) paths.push(stripTrailingPunct(m[1] || m[0]));
  return unique(paths);
}

function safeStat(filePath, opts) {
  const cfg = { ...DEFAULTS, ...opts };
  const info = {
    path: filePath,
    exists: false,
    size: null,
    excerpt: null,
    hash: null,
    truncated: false,
    error: null
  };
  try {
    if (!fs.existsSync(filePath)) return info;
    const st = fs.statSync(filePath);
    info.exists = true;
    info.size = st.size;
    if (st.isDirectory()) {
      info.error = 'directory';
      return info;
    }
    const hashLen = Math.min(st.size, cfg.hashBytes);
    const fd = fs.openSync(filePath, 'r');
    try {
      const hashBuf = Buffer.alloc(hashLen);
      fs.readSync(fd, hashBuf, 0, hashLen, 0);
      info.hash = crypto.createHash('sha256').update(hashBuf).digest('hex');
      const excerptLen = Math.min(st.size, cfg.maxExcerptChars);
      if (excerptLen > 0 && st.size <= cfg.maxFileBytes) {
        const exBuf = Buffer.alloc(excerptLen);
        fs.readSync(fd, exBuf, 0, excerptLen, 0);
        info.excerpt = exBuf.toString('utf8');
        info.truncated = st.size > excerptLen;
      } else if (st.size > cfg.maxFileBytes) {
        const exBuf = Buffer.alloc(Math.min(cfg.maxExcerptChars, st.size));
        fs.readSync(fd, exBuf, 0, exBuf.length, 0);
        info.excerpt = exBuf.toString('utf8');
        info.truncated = true;
      }
    } finally {
      fs.closeSync(fd);
    }
  } catch (err) {
    info.error = err.message;
  }
  return info;
}

function tryParseJson(value) {
  if (value && typeof value === 'object' && !Array.isArray(value)) return value;
  if (typeof value !== 'string') return null;
  const t = value.trim();
  if (!t || (t[0] !== '{' && t[0] !== '[')) return null;
  try {
    return JSON.parse(t);
  } catch {
    return null;
  }
}

function splitMultiStep(text) {
  const src = String(text || '').trim();
  if (!src || !MULTI_STEP_RE.test(src)) return [];
  const parts = src.split(/\s*(?:\band then\b|\bthen\b|\bafter that\b|\bafterwards\b|\bnext,\b|\bfollowed by\b|\bsecondly\b|->|→)\s*/i)
    .map((s) => s.trim())
    .filter((s) => s.length > 2);
  return parts.length >= 2 ? parts : [];
}

function classifyShape({ text, mentions, urls, file_paths, json_payload, steps }) {
  const signals = [];
  if (mentions.length) signals.push('mention');
  if (urls.length) signals.push('url');
  if (file_paths.length) signals.push('path');
  if (json_payload) signals.push('json');
  if (steps.length) signals.push('multi-step');
  if (signals.length === 0) return 'text';
  if (signals.length === 1) {
    return { mention: '@mention', url: 'url', path: 'path', json: 'json', 'multi-step': 'multi-step' }[signals[0]];
  }
  return 'mixed';
}

/**
 * Classify and normalize a user turn without dumping large files into router context.
 * @param {string|object} input
 * @param {object} [opts]
 */
function normalizeInput(input, opts = {}) {
  const cfg = { ...DEFAULTS, ...opts };
  let raw = input;
  let json_payload = null;
  let text = '';

  if (typeof input === 'object' && input !== null) {
    json_payload = input.json || (input.text ? tryParseJson(input.text) : null) || (input.prompt ? tryParseJson(input.prompt) : null) || null;
    text = String(input.text || input.prompt || input.message || input.query || '').trim();
    if (!text && !json_payload) {
      json_payload = input;
      text = JSON.stringify(input);
    }
    if (input.path && !text.includes(String(input.path))) text = (text + ' ' + input.path).trim();
    if (Array.isArray(input.paths)) {
      for (const p of input.paths) if (p && !text.includes(String(p))) text = (text + ' ' + p).trim();
    }
  } else {
    text = String(input || '');
    json_payload = tryParseJson(text);
  }

  const mentions = extractMentions(text, cfg.aliases);
  const urls = unique([].concat(extractUrls(text), Array.isArray(opts.urls) ? opts.urls : []));
  const rawPaths = extractPaths(text);
  if (typeof input === 'object' && input && input.path) rawPaths.push(String(input.path));
  const file_paths = unique(rawPaths).map((p) => {
    if (opts.statFiles === false) {
      return { path: p, exists: null, size: null, excerpt: null, hash: null, truncated: false, error: null };
    }
    return safeStat(p, cfg);
  });
  const steps = splitMultiStep(text);
  const shape = classifyShape({ text, mentions, urls, file_paths, json_payload, steps });

  // Router-safe preview: never include more than the bounded excerpts.
  const previewParts = [text.slice(0, cfg.maxExcerptChars)];
  if (text.length > cfg.maxExcerptChars) previewParts[0] += '…';
  const preview = {
    text: previewParts[0],
    mentions,
    urls,
    files: file_paths.map((f) => ({
      path: f.path,
      exists: f.exists,
      size: f.size,
      hash: f.hash,
      truncated: f.truncated,
      excerpt: f.excerpt
    })),
    json_keys: json_payload && typeof json_payload === 'object' ? Object.keys(json_payload).slice(0, 24) : [],
    steps
  };

  return {
    text,
    raw,
    mentions,
    urls,
    file_paths,
    json_payload,
    steps,
    hints: { multi_step: steps.length >= 2, steps },
    shape,
    preview
  };
}

module.exports = {
  normalizeInput,
  extractUrls,
  extractMentions,
  extractPaths,
  splitMultiStep,
  BOT_ALIASES,
  DEFAULTS
};
