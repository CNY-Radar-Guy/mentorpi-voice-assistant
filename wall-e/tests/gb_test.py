#!/usr/bin/env python3
"""End-to-end intent tests: POST /api/chat, record /rosout + /controller/cmd_vel via rosbridge during each test."""
import json, sys, threading, time, requests, websocket
API = "http://localhost:8000/api/chat"
class Rec:
    def __init__(s):
        s.ws = websocket.create_connection("ws://127.0.0.1:9090", timeout=5); s.ev = []; s.on = True
        s.ws.send(json.dumps({"op": "subscribe", "topic": "/rosout", "type": "rcl_interfaces/msg/Log"}))
        s.ws.send(json.dumps({"op": "subscribe", "topic": "/controller/cmd_vel", "type": "geometry_msgs/msg/Twist"}))
        s.t = threading.Thread(target=s.run, daemon=True); s.t.start()
    def run(s):
        s.ws.settimeout(0.5)
        while s.on:
            try: m = json.loads(s.ws.recv())
            except Exception: continue
            if m.get("op") != "publish": continue
            if m["topic"] == "/rosout":
                n = m["msg"].get("name", "")
                if any(k in n for k in ("line_following", "object_tracking", "hand_gesture", "hand_trajectory", "lidar_app")):
                    s.ev.append((time.time(), "rosout", f"{n}: {m['msg'].get('msg','')[:80]}"))
            else:
                tw = m["msg"]; s.ev.append((time.time(), "cmd", (round(tw["linear"]["x"], 3), round(tw["angular"]["z"], 3))))
    def take(s):
        e, s.ev = s.ev, []; return e
def summarize(ev):
    ros = [x[2] for x in ev if x[1] == "rosout"]
    cmds = [x for x in ev if x[1] == "cmd"]
    nz = [c for c in cmds if c[2] != (0.0, 0.0)]; z = [c for c in cmds if c[2] == (0.0, 0.0)]
    out = {"rosout": ros, "cmd_nonzero": len(nz), "cmd_zero": len(z)}
    if nz:
        out["nonzero_vals"] = sorted(set(c[2] for c in nz)); out["nonzero_span_s"] = round(nz[-1][0] - nz[0][0], 2)
        out["zeros_after_last_nonzero"] = len([c for c in z if c[0] > nz[-1][0]])
    return out
rec = Rec(); time.sleep(1)
tests = json.load(open(sys.argv[1]))
for t in tests:
    text = t["text"]; body = {"text": text, "robot": False}
    if t.get("real"): body["dry_run"] = False
    rec.take(); t0 = time.time()
    try: j = requests.post(API, json=body, timeout=150).json()
    except Exception as ex: j = {"error": str(ex)}
    time.sleep(t.get("wait", 1.5))
    s = summarize(rec.take())
    print(json.dumps({"say": text, "reply": j.get("reply", j.get("error")), "cat": j.get("category"), "backend": j.get("backend"),
                      "tokens": j.get("tokens"), "dry": j.get("dry_run"), "secs": round(time.time() - t0, 1), **s}))
    sys.stdout.flush()
rec.on = False
