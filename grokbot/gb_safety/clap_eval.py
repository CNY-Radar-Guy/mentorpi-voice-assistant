#!/usr/bin/env python3
"""clap_eval.py DIR [sens] - run gb_clap offline on DIR/mic.wav with speaking windows from DIR/labels.json."""
import json, sys, os
sys.path.insert(0, "/home/pi/grokbot/gb_safety"); import gb_clap
d = sys.argv[1]; sens = sys.argv[2] if len(sys.argv) > 2 else "high"
lab = json.load(open(os.path.join(d, "labels.json"))); off = 0.0
win = [(l["s"] + off, l["e"] + off) for l in lab]
dbg = []
evs = gb_clap.wav_test(os.path.join(d, "mic.wav"), sens, win, dbg)
def where(t):
    for l in lab:
        if l["s"] - 0.2 <= t <= l["e"] + 0.6: return l["kind"] + ":" + l.get("text", "")[:20]
    return "none"
print("speaking-aware events:")
for e in evs: print("  ", e["at_s"], where(e["at_s"]), e["peak_db"], e["rise_db"], e["hf_rise"], e["speaking"])
ok = [x for x in dbg if x["ok"]]; print("onsets judged", len(dbg), "accepted", len(ok))
for x in dbg:
    if x["peak"] > -30: print("   onset", round(x["t"], 2), where(x["t"]), "peak", x["peak"], "rise", x["rise"], "hf_rise", x["hf_rise"], "spk", x["spk"], "ok", x["ok"])
evs0 = gb_clap.wav_test(os.path.join(d, "mic.wav"), sens, None)
print("speaking-UNaware (old) events:", len(evs0), [(e["at_s"], where(e["at_s"])) for e in evs0][:30])
print("tts segments:", len([l for l in lab if l["kind"] == "tts"]), "clap segments:", [(round(l.get("clap_at", l["s"]), 1), l["kind"]) for l in lab if "clap" in l["kind"]])
