#!/bin/bash
# gb_export.sh - print transcript lines (JSONL) added since the last export, then advance the marker.
#   ssh pi@mentorpi.local ~/grokbot/gb_export.sh >> grokbot_transcripts.jsonl     # incremental pull
#   gb_export.sh --peek   (print without moving the marker)   gb_export.sh --reset   (start over)
# Marker: ~/grokbot/data/transcripts/.export_marker.json  {"file": "YYYY-MM-DD.jsonl", "line": N}
set -euo pipefail
D="${GB_DATA:-$HOME/grokbot/data}/transcripts"; M="$D/.export_marker.json"
[ "${1:-}" = "--reset" ] && { rm -f "$M"; echo "marker reset" >&2; exit 0; }
mf=""; ml=0
if [ -f "$M" ]; then
  mf=$(python3 -c "import json;print(json.load(open('$M'))['file'])"); ml=$(python3 -c "import json;print(json.load(open('$M'))['line'])")
fi
last_f="$mf"; last_l=$ml
for f in $(ls "$D"/*.jsonl 2>/dev/null | sort); do
  b=$(basename "$f"); [ -n "$mf" ] && [[ "$b" < "$mf" ]] && continue
  start=0; [ "$b" = "$mf" ] && start=$ml
  n=$(wc -l < "$f"); [ "$n" -gt "$start" ] && tail -n +"$((start+1))" "$f" | head -n "$((n-start))"
  last_f="$b"; last_l=$n
done
if [ "${1:-}" != "--peek" ] && [ -n "$last_f" ]; then
  printf '{"file": "%s", "line": %d, "exported_at": "%s"}\n' "$last_f" "$last_l" "$(date -Is)" > "$M.tmp" && sync "$M.tmp" && mv "$M.tmp" "$M" && sync "$D"
fi
