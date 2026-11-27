# RCA-{{id_short}} — {{title}}

> Standing rule (John, 27 Nov 2026 ET): every crash or unexpected behavior on Wall-E / Jarvis must produce a structured Root Cause Analysis and be saved for the recursive learning loop.

## Metadata

| Field | Value |
|---|---|
| RCA ID | `{{id}}` |
| Timestamp (ET) | {{timestamp_et}} |
| Status | {{status}} |
| Trigger type | `{{trigger}}` |
| Affected component / service | {{component}} |
| Reporter | {{reporter}} |
| Related reports | {{related}} |

## Symptom

{{symptom}}

## Timeline (America/New_York)

{{timeline}}

## Evidence

### Sensor / arm / mode snapshot

```json
{{snapshot_json}}
```

### `guard_event.json`

```json
{{guard_event}}
```

### `gate.log` excerpt

```
{{gate_log}}
```

### `journalctl` excerpt

```
{{journal}}
```

{{extra_evidence}}

## Root Cause

{{root_cause}}

## Fix Applied

{{fix_applied}}

## Regression Test

{{regression_test}}

## Prevention Rule (Learning Loop)

{{prevention_rule}}

## Follow-ups

{{followups}}
