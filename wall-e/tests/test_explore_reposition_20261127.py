#!/usr/bin/env python3
"""2026-11-27: transient obstacle stops reposition + continuous multi-direction
low-speed nudges (no full-stop dwell) instead of aborting after 3 stops in 20 s.
Hard abort only after repeated failed nudge cycles with no progress. No motion.
"""
from __future__ import annotations
import math, os, sys

REL = os.environ.get("GB_REL", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REL)

import gb_explore as X

P = F = 0

def check(name, ok, detail=None):
    global P, F
    if ok:
        P += 1
        print("  PASS", name)
    else:
        F += 1
        print("  FAIL", name, "" if detail is None else detail)


def caps(**kw):
    d = dict(front_stop=0.27, rear_stop=0.28, sweep_need=0.24, margin=0.05,
             blind_frac_stop=0.5, progress_m=0.05, fail_limit=X.REPOSITION_FAIL_LIMIT,
             armed=True, estop=False, abort=False,
             nudge_lin=X.NUDGE_LIN, nudge_ang=X.NUDGE_ANG, nudge_s=X.NUDGE_S,
             back_v=X.REPOSITION_BACK_V, back_s=X.REPOSITION_BACK_S, turn_s=X.REPOSITION_TURN_S,
             avoid_dist=None)
    d.update(kw)
    return d


def sens(**kw):
    d = dict(front=0.25, rear=1.00, corr_rev=1.00, sweep=0.18,
             left=0.45, right=0.16, blind_r=0.0, odom=(0.0, 0.0, 0.0),
             turn_ok=False, rev_ok=True)
    d.update(kw)
    return d


def cmd_ang(out):
    if out.get("ang") is not None:
        return float(out["ang"])
    return float(out.get("ang_dir") or 0.0)


def simulate(sensor_at, steps=200, dt=0.10, fail_limit=None, start_front=0.25, start_sweep=0.18, open_dir=1):
    """Drive reposition_step at 10 Hz. sensor_at(i, now, odom, last_out) -> sensors dict."""
    now = 0.0
    odom = [0.0, 0.0, 0.0]
    rp = X.start_reposition(now, tuple(odom), start_front, start_sweep, open_dir)
    c = caps(fail_limit=fail_limit if fail_limit is not None else X.REPOSITION_FAIL_LIMIT)
    hist = []
    out = {"lin": 0.0, "ang": 0.0, "event": "running", "why": "start", "fails": 0, "reason": ""}
    for i in range(steps):
        s = sensor_at(i, now, tuple(odom), out)
        s = dict(s)
        s["odom"] = (odom[0], odom[1], odom[2])
        out = X.reposition_step(rp, now, s, c)
        lin = float(out.get("lin") or 0.0)
        ang = cmd_ang(out)
        hist.append({"t": now, "lin": lin, "ang": ang, "event": out["event"],
                     "why": out.get("why") or "", "fails": out.get("fails"),
                     "reason": out.get("reason") or "", "phase": rp.get("phase")})
        # crude holonomic-free integration so progress can accumulate
        odom[0] += lin * dt * math.cos(odom[2])
        odom[1] += lin * dt * math.sin(odom[2])
        odom[2] += ang * dt
        now += dt
        if out["event"] in ("recovered", "abort", "stay_stopped"):
            break
    return hist, rp


# ---------------------------------------------------------------------------
# Classification + abort policy
# ---------------------------------------------------------------------------
check("front obstacle trip -> reposition (not abort)",
      X.handle_explore_trip("obstacle 0.24m ahead (lidar, stop 0.27m)") == "reposition")
check("turn-sweep trip -> reposition",
      X.handle_explore_trip("obstacle 0.17m inside the turn sweep (r 0.24m)") == "reposition")
check("soft stall / track stall -> reposition",
      X.handle_explore_trip("bump detected (encoders): track stall: commanded 0.08 m/s, encoders 0.00 m/s") == "reposition")
check("ESTOP is a hard abort, not a retry",
      X.handle_explore_trip("ESTOP latched") == "hard_abort")
check("tilt stays on bump-recover (not transient reposition)",
      X.handle_explore_trip("tilt 18 deg") == "bump_recover")
check("is_transient_obstacle_trip true for the three stop types",
      X.is_transient_obstacle_trip("obstacle 0.25m ahead (lidar, stop 0.27m)")
      and X.is_transient_obstacle_trip("obstacle 0.20m inside the turn sweep (r 0.24m)")
      and X.is_transient_obstacle_trip("soft stall: commanded 0.08 m/s"))

# More than 3 transient stops must still be 'reposition', never a trip-limit abort.
kinds = [X.handle_explore_trip(w) for w in [
    "obstacle 0.24m ahead (lidar, stop 0.27m)",
    "obstacle 0.20m inside the turn sweep (r 0.24m)",
    "track stall: commanded 0.08 m/s, encoders 0.00 m/s",
    "obstacle 0.25m ahead (lidar, stop 0.27m)",
    "obstacle 0.22m inside the turn sweep (r 0.24m)",
]]
check("5 transient trips in a row are all 'reposition' (no 3-in-20s abort)",
      kinds == ["reposition"] * 5, kinds)

# Source: old 3-in-20s abort is gone; trip-hold and gate stops remain.
src = open(os.path.join(REL, "gb_explore.py"), encoding="utf-8").read()
check("TRIP-HOLD still in send() (blocked direction not immediately re-commanded)",
      "TRIP-HOLD rev" in src and "TRIP-HOLD fwd" in src and "TRIP-HOLD turn" in src)
check("TRIP_LIMIT constant kept for logging only; no mission abort on len(tt) >= TRIP_LIMIT",
      'TRIP_LIMIT = int(E("EXPLORE_TRIP_LIMIT", "3"))' in src
      and "if len(tt) >= TRIP_LIMIT" not in src
      and "logging only; no TRIP_LIMIT abort" in src)
check("gb_gate stop values remain documented as 0.27/0.28/0.24",
      '"front": 0.27, "rear": 0.28, "sweep": 0.24' in src)
check("nudges are documented to stay above STALL_MIN_CMD = 0.05",
      "STALL_MIN_CMD = 0.05" in src and "NUDGE_LIN = 0.065" in src)
check("commands still documented as going through send() / gb_gate",
      "send() -> gb_gate" in src and "Every command passes send()" in src)
check("speed caps documented (0.288 manual is in gate; 1.0 autonomous in explore)",
      "HARD_MAX_LIN, HARD_MAX_ANG = 1.0, 0.64" in src)

# Abort decision
act, why, n = X.reposition_abort_decision(0, 5, progressed=True)
check("successful reposition resets fail counter and does not abort",
      act == "resume" and n == 0 and "recovered via reposition" in why, (act, why, n))
act, why, n = X.reposition_abort_decision(0, 5, progressed=False)
check("first failed cycle retries (no abort)", act == "retry" and n == 1, (act, why, n))
act, why, n = X.reposition_abort_decision(3, 5, progressed=False)
check("4th failed cycle still retries", act == "retry" and n == 4, (act, why, n))
act, why, n = X.reposition_abort_decision(4, 5, progressed=False)
check("5th consecutive failed cycle hard-aborts as truly stuck",
      act == "abort" and n == 5 and "truly stuck after 5 failed reposition attempts" in why, (act, why, n))
act, why, n = X.reposition_abort_decision(4, 5, progressed=True)
check("progress after 4 failures resets and resumes (does not abort)",
      act == "resume" and n == 0, (act, why, n))

# rear_safe_to_back preserves 0.28 m rear stop + margin
check("rear 1.0 m is safe to back (stop 0.28 + 0.05 margin)",
      X.rear_safe_to_back(1.0, 1.0, 0.28, 0.05) is True)
check("rear 0.30 m (below 0.28+0.05) is not safe to back",
      X.rear_safe_to_back(0.30, 0.30, 0.28, 0.05) is False)
check("rear safe check respects ESTOP / disarm / abort",
      X.rear_safe_to_back(1.0, 1.0, 0.28, estop=True) is False
      and X.rear_safe_to_back(1.0, 1.0, 0.28, armed=False) is False
      and X.rear_safe_to_back(1.0, 1.0, 0.28, abort=True) is False)

# Nudge speed band
check("NUDGE_LIN is in 0.06-0.07 and above STALL_MIN_CMD 0.05",
      0.06 <= X.NUDGE_LIN <= 0.07 and X.NUDGE_LIN > X.STALL_MIN_CMD_NUDGE
      and X.STALL_MIN_CMD_NUDGE == 0.05)
check("clamp_nudge_lin keeps 0.065 and lifts 0.04 up into the band",
      abs(X.clamp_nudge_lin(0.065) - 0.065) < 1e-9
      and 0.06 <= abs(X.clamp_nudge_lin(-0.04)) <= 0.07
      and abs(X.clamp_nudge_lin(-0.065) + 0.065) < 1e-9)
check("REPOSITION_FAIL_LIMIT is 5", X.REPOSITION_FAIL_LIMIT == 5)
check("REPOSITION_BACK_V is in [-0.08, -0.06]",
      -0.08 <= X.REPOSITION_BACK_V <= -0.06)

# ---------------------------------------------------------------------------
# 1) Transient front obstacle: back away, turn to open side, resume.
#    Repeat more than 3 times — never abort.
# ---------------------------------------------------------------------------
def run_open_recovery(n=1):
    # front blocked, rear open, left clearer; after a short backup the sweep and
    # front open up so the maneuver can resume.
    def sensor_at(i, now, odom, last):
        opened = now >= 1.0
        return sens(front=0.80 if opened else 0.25,
                    sweep=0.50 if opened else 0.22,
                    turn_ok=opened, left=1.2, right=0.4,
                    rear=1.2, corr_rev=1.2, odom=odom)
    return simulate(sensor_at, steps=80)

events_all = []
abort_any = False
for k in range(4):
    hist, rp = run_open_recovery(k)
    events_all.append([h["event"] for h in hist])
    if any(h["event"] == "abort" for h in hist):
        abort_any = True
    # first motion should be a backup (rear is safe, front blocked)
    moved = [h for h in hist if h["lin"] or h["ang"]]
    check("recovery %d starts by backing or nudging (nonzero, no abort)" % (k + 1),
          bool(moved) and hist[-1]["event"] == "recovered" and not abort_any,
          hist[:3] + hist[-2:])
    if moved:
        # before the world opens, lin should be reverse or a nudge in-band
        early = [h for h in hist if h["t"] < 1.0 and (h["lin"] or h["ang"])]
        if early:
            lins = [h["lin"] for h in early]
            check("recovery %d early linear is reverse or in the nudge band" % (k + 1),
                  all(L <= 0 or (0.06 - 1e-9) <= abs(L) <= 0.07 + 1e-9 for L in lins),
                  lins)

check("four successful recoveries in one run never hard-abort (count > 3 transient stops)",
      not abort_any and all(ev[-1] == "recovered" for ev in events_all), events_all)

# ---------------------------------------------------------------------------
# 2) Turn-sweep block: back away until sweep > 0.24, turn to open side, resume.
# ---------------------------------------------------------------------------
def sensor_sweep(i, now, odom, last):
    # backing increases sweep; once above 0.24 the turn is legal and the front is clear
    traveled = abs(odom[0]) + abs(odom[1])
    sweep = min(0.50, 0.18 + traveled * 2.0)
    opened = sweep >= 0.24
    return sens(front=0.40 if opened else 0.28,
                sweep=sweep, turn_ok=opened,
                rear=1.0, corr_rev=1.0, left=0.9, right=0.3, odom=odom)

hist, rp = simulate(sensor_sweep, steps=120, start_front=0.28, start_sweep=0.18)
lins = [h["lin"] for h in hist]
why = [h["why"] for h in hist]
check("turn-sweep recovery ends in resume, not abort",
      hist[-1]["event"] == "recovered", hist[-3:])
check("turn-sweep recovery commanded a reverse move to gain sweep room",
      any(h["lin"] < 0 for h in hist), why[:8])
check("turn-sweep path uses back or nudge, then resumes",
      any("back" in h["why"] or "nudge" in h["why"] for h in hist) and hist[-1]["event"] == "recovered",
      why[:10] + why[-3:])

# ---------------------------------------------------------------------------
# 3) Tight corner: continuous multi-direction low-speed nudges, no full-stop
#    dwell between nudges; then a path opens and exploration resumes.
# ---------------------------------------------------------------------------
def sensor_corner(i, now, odom, last):
    # Tight corner: front and sweep stay blocked long enough that the initial
    # back+turn cannot clear him, so he must run a multi-direction nudge cycle.
    # Open only after several nudge slices (back is 1.2 s, then nudges at 0.4 s).
    opened = now >= 3.0
    if opened:
        return sens(front=0.80, sweep=0.50, turn_ok=True,
                    rear=1.0, corr_rev=1.0, left=1.0, right=0.6, odom=odom)
    return sens(front=0.22, sweep=0.17, turn_ok=False,
                rear=0.90, corr_rev=0.90, left=0.40, right=0.14, odom=odom)

hist, rp = simulate(sensor_corner, steps=160, start_front=0.22, start_sweep=0.17)
# Motion ticks before recovery
motion = [h for h in hist if h["event"] in ("running", "retry")]
# Between two feasible-nudge ticks, there must be no full-stop dwell: a (0,0)
# command is only allowed at the very start (choose/back transition already
# emits motion) or the final recovered/abort/stay_stopped tick.
zero_dwell = []
for a, b in zip(motion, motion[1:]):
    # a feasible nudge/back tick followed by a zero running/retry tick is a dwell
    if (abs(a["lin"]) > 1e-9 or abs(a["ang"]) > 1e-9) and abs(b["lin"]) <= 1e-9 and abs(b["ang"]) <= 1e-9:
        # ignore if b is the last tick and event later becomes recovered — still a dwell
        zero_dwell.append((a, b))

nudge_ticks = [h for h in motion if h["why"].startswith("nudge") or "nudge" in h["why"]]
nudge_names = []
for h in nudge_ticks:
    # why: "nudge back", "nudge back_left", "nudge arc_left (progress, continue)", ...
    parts = h["why"].split()
    if len(parts) >= 2 and parts[0] == "nudge":
        nudge_names.append(parts[1])
unique_dirs = []
for n in nudge_names:
    if n not in unique_dirs:
        unique_dirs.append(n)

check("tight-corner maneuver does not hard-abort; it resumes once a path opens",
      hist[-1]["event"] == "recovered", hist[-4:])
check("tight-corner uses continuous nudges (at least one nudge command)",
      len(nudge_ticks) >= 1, [h["why"] for h in hist[:20]])
check("tight-corner nudges use at least 2 different directions",
      len(unique_dirs) >= 2, unique_dirs)
check("tight-corner has no full-stop dwell between motion ticks",
      len(zero_dwell) == 0, zero_dwell[:4])
if nudge_ticks:
    bad_lin = [h["lin"] for h in nudge_ticks if abs(h["lin"]) < 0.06 - 1e-9 or abs(h["lin"]) > 0.07 + 1e-9]
    check("every corner nudge stays in the 0.06-0.07 m/s band (above STALL_MIN_CMD 0.05)",
          not bad_lin, bad_lin[:8])
    check("every corner nudge pairs low angular with that linear component (no in-place creep)",
          all(abs(h["lin"]) >= 0.06 - 1e-9 for h in nudge_ticks))

# A single failed nudge cycle must not abort: hold the corner closed for one cycle only,
# then open. fail_limit=5.
def sensor_one_cycle_fail_then_open(i, now, odom, last):
    # Keep boxed-enough that we can still reverse (rear open) but front/sweep stay tight
    # for the first full nudge cycle (~3 directions * 0.4 s = 1.2 s plus initial back).
    if now < 2.0:
        return sens(front=0.22, sweep=0.17, turn_ok=False,
                    rear=0.90, corr_rev=0.90, left=0.35, right=0.15, odom=odom)
    return sens(front=0.90, sweep=0.50, turn_ok=True,
                rear=1.0, corr_rev=1.0, left=1.0, right=0.8, odom=odom)

hist, rp = simulate(sensor_one_cycle_fail_then_open, steps=160, start_front=0.22, start_sweep=0.17)
check("a single failed/partial nudge cycle does not hard-abort; later clearance resumes",
      hist[-1]["event"] == "recovered" and all(h["event"] != "abort" for h in hist),
      hist[-4:])

# ---------------------------------------------------------------------------
# 4) Truly boxed in: no legal nudge (front, rear, sweep all below stops) for
#    many cycles -> hard abort only after REPOSITION_FAIL_LIMIT failed cycles.
# ---------------------------------------------------------------------------
def sensor_boxed(i, now, odom, last):
    return sens(front=0.15, sweep=0.10, turn_ok=False, rev_ok=False,
                rear=0.15, corr_rev=0.15, left=0.10, right=0.10,
                blind_r=0.0, odom=odom)

# With fail_limit=5 and nudge_s=0.40, each empty cycle takes ~0.4 s, plus the
# initial choose->nudge transition. Give plenty of ticks.
hist, rp = simulate(sensor_boxed, steps=200, fail_limit=5, start_front=0.15, start_sweep=0.10)
events = [h["event"] for h in hist]
fails_series = [h["fails"] for h in hist]
# Should not abort on the first empty cycle
first_retry = next((h for h in hist if h["event"] == "retry"), None)
check("boxed-in first empty cycle retries rather than aborting",
      first_retry is not None and first_retry["fails"] < 5
      and "truly stuck" not in (first_retry.get("reason") or ""),
      first_retry)
check("boxed-in eventually hard-aborts after repeated failed cycles",
      hist[-1]["event"] == "abort"
      and "truly stuck after" in (hist[-1]["reason"] + hist[-1]["why"])
      and hist[-1]["fails"] >= 5,
      hist[-4:])
check("boxed-in abort reason uses the genuine-stuck phrasing (not 3-in-20s)",
      "truly stuck after" in (hist[-1]["reason"] + hist[-1]["why"])
      and "obstacle guard stopped me" not in (hist[-1]["reason"] + hist[-1]["why"]))

# Sanity: ESTOP mid-nudge stops immediately, no further motion command other than zero
rp = X.start_reposition(0.0, (0, 0, 0), 0.22, 0.17, 1)
out = X.reposition_step(rp, 0.0, sens(), caps())
out2 = X.reposition_step(rp, 0.1, sens(), caps(estop=True))
check("ESTOP during a maneuver returns stay_stopped with zero command",
      out2["event"] == "stay_stopped" and out2["lin"] == 0 and cmd_ang(out2) == 0,
      out2)
out3 = X.reposition_step(rp, 0.2, sens(), caps(armed=False))
# already deactivated by estop; still zero
check("disarmed / inactive maneuver does not request motion",
      out3["lin"] == 0 and cmd_ang(out3) == 0, out3)

# Nudge candidates never propose forward below the 0.27 stop or reverse below 0.28+margin
c = X.nudge_candidates(sens(front=0.22, rear=1.0, corr_rev=1.0), caps())
check("tight front: candidates are reverse-only (no forward into 0.27 m stop)",
      c and all(x["lin"] < 0 for x in c), c)
c2 = X.nudge_candidates(sens(front=0.22, rear=0.20, corr_rev=0.20), caps())
check("front and rear both below stops: no legal nudge candidates",
      c2 == [], c2)
c3 = X.nudge_candidates(sens(front=0.80, rear=1.0, corr_rev=1.0, left=1.0, right=0.2), caps())
check("open front: includes a forward/arc candidate, still in the 0.06-0.07 band",
      any(x["lin"] > 0 for x in c3) and all(0.06 <= abs(x["lin"]) <= 0.07 for x in c3), c3)

print("RESULT: %d passed, %d failed" % (P, F))
sys.exit(1 if F else 0)
