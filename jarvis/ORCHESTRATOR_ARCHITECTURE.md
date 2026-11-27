# Jarvis Multi-Agent Orchestrator Architecture

Phase 1 blueprint for running Jarvis as a **manager / orchestrator** of specialist agents on **ALPHA-ONE**. This document matches the machine as inventoried on 23 Nov 2026 (America/New_York). Phase 1 ships as a **standalone** routing layer under `D:\AI-Models\jarvis\orchestrator\` and does **not** patch `server.js`, `index.html`, `bots.json`, `README.txt`, or `shell-agent.*` (GB-15 is wiring the guarded shell agent in parallel).

## 1. What is actually on ALPHA-ONE

| Resource | Measured value |
|---|---|
| GPU | NVIDIA GeForce RTX 5080, **16303 MiB** (16 GB) VRAM |
| System RAM | **31.46 GB** (not 64 GB — plan offload conservatively) |
| CPU | Intel Core Ultra 7 265K, **20 cores / 20 threads** |
| Node | v20.18.1, Jarvis at `D:\AI-Models\jarvis`, CommonJS, zero npm deps, bind `127.0.0.1:8765` |
| Ollama | `http://127.0.0.1:11434` (no model was loaded at inventory time) |

### Installed Ollama models (from `ollama list` — do not invent others)

| Model | Disk size | VRAM fit on 16 GB | Role in this design |
|---|---|---|---|
| **qwen3.5:4b** | 3.3 GB | Comfortable | **Preferred `fast_router`** when no larger model is already warm |
| **qwen3:14b** | 9.3 GB | Fits (Jarvis daily driver) | `general_chat`, `tool_caller`, and warm-start router (reuse, don't swap) |
| **phi4:latest** | 9.1 GB | Fits | Compact general / fallback |
| **deepseek-r1:14b** | 9.0 GB | Fits | `deep_reasoner` that stays in VRAM |
| **mistral-small:24b** | 14 GB | Tight (weights alone ~14 GB + KV) | Optional general; avoid pairing with anything else |
| **gemma3:27b** | 17 GB | **Spill** | Writing; only with `allow_partial_offload` |
| **qwen2.5-coder:32b** | 19 GB | **Spill** | Preferred coder *name*, but default policy falls back to `qwen3:14b` |
| **deepseek-r1:32b** | 19 GB | **Spill** | Heavy reasoner; default to 14B |
| **qwen3:32b** | 20 GB | **Spill** | Heavier general; default to 14B |
| **llama3.3:70b** | 42 GB | **Will not fit in VRAM+RAM** (16+31 GB) | Archive only; never auto-selected |

`bots.json` is a **cloud / local inventory and coaching file**, not an executable specialist registry. The orchestrator reads it for awareness and for Phase 2 dashboard cards. The live specialist roster lives in `D:\AI-Models\jarvis\orchestrator-config.json`.

Existing Jarvis features the architecture must respect: Systems panel (GPU / CPU / disks / Ollama / Wally @ `192.168.50.125` / Tailscale), Waiting list, Program mode (editable JSON with backups), Brave web access (authorize-then-fetch, read-only, no LAN), voice, and the incoming **guarded `shell-agent`** (Tier 1 auto-safe / Tier 2 click-to-approve). Jarvis chat today uses `qwen3:14b`, `think: false`, `keep_alive: "30m"`, `num_ctx: 8192`.

## 2. Manager / orchestrator pattern (keep Jarvis's own window tiny)

Jarvis is the manager, not the worker. It never swallows a specialist's full transcript. Every turn is a three-stage pipeline:

```
User turn
  │
  ├─ Stage 0  Deterministic fast-path router     0 LLM tokens
  │     regex / @bot / input-shape / session affinity
  │     high confidence → skip Stage 1
  │
  ├─ Stage 1  Compact LLM triage                 ~150–250 tokens max
  │     only if Stage 0 confidence < threshold (default 0.75)
  │     small model (qwen3.5:4b) or warm qwen3:14b
  │     think:false  num_ctx:2048  num_predict:120
  │     sees: latest user turn + 1-line specialist cards + session pointers
  │     NEVER full transcripts
  │     returns { target_bot, intent, inputs, reuse_session, needs_handoff, max_ctx }
  │
  └─ Stage 2  Specialist execution + structured handoff
        specialist gets: its system prompt + its session summary + compact task payload
        returns: { summary, artifacts, tool_events, session_delta, tokens_used }
        Jarvis (and the dashboard) read only that tiny summary
```

Compound requests (`check disk space and install X`) are split in Stage 0 into a short DAG (`ops` → `shell-agent`) without spending tokens. The dispatcher runs steps in order and threads structured artifacts (not raw logs) into the next step.

## 3. Two-stage router details

### Stage 0 — 0-token fast path

Scored rules, first decisive win or highest score ≥ `router.fast_path_confidence` (0.75):

| Signal | Target |
|---|---|
| Explicit `@shell-agent` / `@coder` / `@researcher` / `@ops` / `@general` (and aliases) | That bot, confidence 1.0, always wins |
| Install / download / uninstall / winget / software setup / "run this command" | `shell-agent` |
| Code, debug, OpenSCAD, scripts, stack traces, "write a function" | `coder` |
| `http(s)` URL, "search the web", "look up", Brave-style research | `researcher` |
| GPU / VRAM / nvidia-smi / RAM / disk / Wally / MentorPi / Tailscale / Ollama loaded models | `ops` |
| Follow-up on the last active specialist session (topic overlap or clear anaphora, within `followup_idle_ms`) | Reuse that session, confidence 0.80 |

`ops` prefers **zero-LLM probes** (the same style of signals Jarvis already gathers in `getStatus()`: `nvidia-smi`, Wally ping/SSH, `sc.exe query Tailscale`, `tailscale status`, disk free). The orchestrator does not call an LLM to answer "is Tailscale running?"

### Stage 1 — compact JSON classifier

Used only when Stage 0 is ambiguous ("can you help me with this?"). The prompt is hard-capped (~1,200 characters): latest user text (after input normalization), one line per specialist, and pointers like `coder#ab12 "OpenSCAD cube" (2 turns, 4m ago)` — never the cube script itself. Output is JSON-schema constrained. If Ollama is down or the reply is unparseable, fall back to the best Stage 0 guess, or `general`.

## 4. 16 GB VRAM scheduling (RTX 5080, 31.5 GB RAM)

The scheduler (`vram-scheduler.js`) is the difference between a snappy desk assistant and a machine that stalls while 19 GB weights spill into RAM.

**Rules, in order:**

1. **Reuse a loaded compatible model.** If Jarvis already has `qwen3:14b` warm (`keep_alive: 30m` today), Stage 1 and most specialists use it rather than evicting it to load `qwen3.5:4b`. Swapping a 9 GB model to save a few hundred router tokens is a net loss.
2. **Fit-first (`fit_policy: "prefer_in_vram"`).** Auto-routing will not select a model whose estimated weights + KV cache exceed `16384 − 1536` MiB reserved. That means 27B / 32B / 70B are **not** auto-selected.
3. **Fallback chain, not wishful loading.** `deep_coder` lists `qwen2.5-coder:32b` first as the *preferred* quality pick, then `qwen3:14b`. With `allow_partial_offload: false` (default), the 32B is skipped and `qwen3:14b` runs fully on GPU. Flip the flag or pass an explicit model override only when John accepts RAM offload and a slower first token.
4. **One heavy at a time.** Before loading a competing heavy model, send `keep_alive: 0` to evict the cold/heavy resident. Never leave `qwen3:32b` and `qwen2.5-coder:32b` fighting over 16 GB.
5. **Context caps.** Router `num_ctx: 2048`. Standard specialist `4096`. Coder `8192`, and `16384` only when the task payload sets `max_ctx` and VRAM still fits. Do not silently inherit Jarvis's current 8192/12288 for every call.
6. **keep_alive.** `5m` for the active specialist; `0` on a heavy model being evicted; do not casually unload the daily-driver 14B.
7. **70B is archive.** `llama3.3:70b` (42 GB) is larger than VRAM+RAM combined. The scheduler will not pick it.

Estimated KV overhead is a simple heuristic from weight size and `num_ctx` (good enough to prevent obvious overcommit; not a profiler). Live `/api/ps` size, when present, wins over the estimate.

## 5. Session reuse and redundant-context elimination

Persistent store: `orchestrator/sessions.json` (index) + `orchestrator/sessions/<id>.json`, keyed by `(bot_id, topic_key)`.

Each session keeps:

- A **pinned facts** block (short key/value items learned in that domain).
- A **rolling compressed summary**, auto-compacted when history exceeds **6 turns** or **~1,800 estimated tokens**.
- Only the last **N = 4** raw verbatim turns.

**Affinity:** a follow-up reuses the last specialist session when the topic key still matches or the utterance is a clear continuation ("add error handling", "try again", "and the GPU?"). That avoids cold-start re-explanations.

**Dedup:** shared machine facts (GPU, RAM, paths, Wally IP, active bots) live **once** in `orchestrator/shared-facts.json`, addressed by `id#hash`. Specialist prompts get the id/hash; the full blob is injected only on cache miss or hash change. Hardware facts are not recopied into every bot prompt.

The specialist prompt payload is therefore: system card + pinned facts + rolling summary + shared-fact refs + last N turns + the new structured task. Jarvis itself only stores the structured handoff summary.

## 6. Variable-input normalization and specialist registry

`normalizeInput()` accepts plain text, `@bot` mentions, Windows/UNC/POSIX paths, `file://` URIs, URLs, raw JSON, or a mixed payload. Paths are **not** dumped into the router: the normalizer records `path`, `exists`, `size`, `sha256` (first 64 KB), and an **≤800 character excerpt**, flagged `truncated` when needed. URLs are extracted and stripped out of "is this a path?" detection. Multi-step cues (`and then`, `->`, "after that") become a hint list for the router.

Default specialists (editable in `orchestrator-config.json`):

| Id | Adapter (Phase 2 hook) | Notes |
|---|---|---|
| `shell-agent` | `shell-agent` | Delegates to GB-15's guarded agent. **Must preserve** `approval_required`, `guardrail`, `blocked`, and proposed command fields. The orchestrator never auto-approves Tier 2 work. |
| `coder` | `ollama` | Code / OpenSCAD / scripts; higher `num_ctx` on demand. |
| `researcher` | `web` | Uses Jarvis's existing Brave authorize-then-fetch flow. No new fetch path around the approval card. |
| `ops` | `ops` | Zero-LLM probes first (`getStatus`-style). |
| `general` | `ollama` | Conversational Jarvis fallback, tiny context. |

Chained example: research a vendor URL → extract an installer link (artifact) → pass `{url, product}` to `shell-agent` → **click-to-approve** in the Jarvis UI. Stage 2 will not run the install until that approval exists.

## 7. Token telemetry and observability

Every call appends one JSON line to `orchestrator/telemetry.jsonl`:

- `prompt_eval_count` / `eval_count` from Ollama, split into `orchestrator_tokens` (Stage 1 only; Stage 0 is always 0) vs `specialist_tokens`
- Counters: `fast_path_hits`, `session_reuses`, `compactions`, `model_swaps`, `evictions`
- Route metadata: `target_bot`, `stage` (`fast_path` | `llm_router` | `fallback`), `model`, `num_ctx`, `keep_alive`

Phase 2 can surface these on the Systems / dashboard panel. Phase 1 exposes them at `GET /api/orchestrator/telemetry` once the router is mounted.

## 8. Module map (all new files)

| Path | Role |
|---|---|
| `D:\AI-Models\jarvis\ORCHESTRATOR_ARCHITECTURE.md` | This blueprint |
| `D:\AI-Models\jarvis\orchestrator-config.json` | Editable policy (tiers, VRAM budget, specialists, compaction) |
| `orchestrator/input-normalizer.js` | Shape detection, `@bot`, path excerpts, URLs, JSON, multi-step hints |
| `orchestrator/session-store.js` | Affinity, pinned facts, rolling summary, dedup, compact payloads |
| `orchestrator/vram-scheduler.js` | `/api/tags` + `/api/ps`, fit-first pick, evict, `num_ctx` / `keep_alive` |
| `orchestrator/router.js` | Stage 0 + Stage 1 + DAG split |
| `orchestrator/dispatcher.js` | Route → session → VRAM → pluggable adapter → handoff + telemetry |
| `orchestrator/index.js` | `createOrchestrator()` + `handleOrchestratorRoute()` |
| `orchestrator/test-router.js` | Unit + mock-integration suite (and an optional live `/api/tags`+`/api/ps` probe) |

Public API (ready to mount, not mounted in Phase 1):

- `POST /api/orchestrator/route` — classify only
- `POST /api/orchestrator/chat` — full dispatch
- `GET  /api/orchestrator/sessions`
- `GET  /api/orchestrator/telemetry`
- `GET  /api/orchestrator/specialists`

`createOrchestrator({ configPath, botsJsonPath, shellAgentAdapter, ollamaBaseUrl, opsAdapter, webAdapter, ollamaAdapter })` accepts adapters so Phase 2 can hook GB-15's shell agent and Jarvis's Brave/ops helpers in a few lines.

## 9. Phase 2 hookup (do this only after GB-15 finishes)

Do **not** apply this until `shell-agent` is already wired and the other worker is done with `server.js` / `index.html` / `README.txt`. Then add something equivalent to the following (~20 lines) — adjust names to whatever GB-15 exported:

```js
const { createOrchestrator, handleOrchestratorRoute } = require('./orchestrator');
const orchestrator = createOrchestrator({
  configPath: path.join(ROOT, 'orchestrator-config.json'),
  botsJsonPath: F.bots,
  ollamaBaseUrl: OLLAMA,
  shellAgentAdapter: typeof handleShellAgent === 'function' ? handleShellAgent : null,
  opsAdapter: async () => snapshotForPrompt(await getStatus()), // or a richer ops probe
  webAdapter: null // wrap the existing Brave authorize-then-fetch flow
});
// inside http.createServer, before the static/public fallback:
if (urlPath.startsWith('/api/orchestrator/')) {
  return handleOrchestratorRoute(req, res, orchestrator);
}
```

Optional UI (later): a compact "Routed to coder (fast-path, session reused, 0 orchestrator tokens)" line in the chat transcript, plus telemetry totals on the Systems panel. No change to Program mode, Brave approval cards, or shell-agent Tier 2 approval is required for the mount itself.

## 10. Safety

- Phase 1 is localhost-only by construction once mounted on Jarvis's existing `127.0.0.1` server.
- The orchestrator does not run shell commands itself. All installs/commands go through the guarded `shell-agent` adapter and must keep approval/guardrail fields intact.
- `researcher` must continue to use Jarvis's authorize-then-fetch Brave path; no silent web fetches.
- `ops` is read-only probes.
- No model pulls are triggered by the router (`/api/tags` is read-only). Recommended optional pulls: none required — `qwen3.5:4b` is already present for cold-start routing.
