#!/bin/bash
# selfhear_test.sh OUTDIR - 10 spoken replies through grokbot's chat path (robot speaker, dry_run motion) while the shared
# mic records; then checks grokbot's log for any self-trigger (heard/kill/ESTOP) in the window.
O=$1; mkdir -p $O; cd /home/pi/grokbot; set -a; . ./grokbot.env 2>/dev/null; set +a
arecord -q -D gbmic -f S16_LE -r 48000 -c 1 $O/mic.wav & R=$!
T0=$(date +%s.%N); echo "[]" > $O/labels.json; sleep 2
PH=("hello there, tell me about yourself in three sentences" "what can you do? answer in a few sentences" "stop"
    "resume" "describe a nice day at the beach in four sentences" "what is the capital of france and why is it famous"
    "tell me a short story about a robot and a cat" "stop" "resume" "give me three tips for staying focused")
i=0
for p in "${PH[@]}"; do
  s=$(date +%s.%N)
  r=$(curl -s -m 90 -H "X-Token: ${WEB_TOKEN}" -H 'Content-Type: application/json' -d "{\"text\": \"$p\", \"robot\": true, \"dry_run\": true}" http://127.0.0.1:8000/api/chat)
  e=$(date +%s.%N)
  python3 - "$O/labels.json" "$T0" "$s" "$e" "$p" "$r" <<'PY'
import json, sys
f, t0, s, e, p, r = sys.argv[1:]; L = json.load(open(f))
try: rep = json.loads(r).get("reply", "")
except Exception: rep = r[:200]
L.append({"kind": "tts", "text": p, "reply": rep[:300], "s": float(s) - float(t0), "e": float(e) - float(t0)}); json.dump(L, open(f, "w"), indent=1)
PY
  i=$((i+1)); echo "reply $i done: $p"; sleep 1.5
done
sleep 2; kill $R; wait $R 2>/dev/null
echo "T0=$T0"
