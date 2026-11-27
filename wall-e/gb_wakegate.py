"""gb_wakegate.py - strict wake requirement for motion (John, 2026-11-24 04:06 PT).

Motion (missions/explore, navigate, timed drives, drive steps, follow/come/go-to, stock drive features, barge-in drives)
may start ONLY from an utterance that carried the wake phrase ('wake up Wally', wake=True), or from the first command
inside the window that a bare wake phrase opened. Follow-up windows after Wally's own replies, overheard speech and
chat context never start motion. Stops / e-stop are handled before this gate and are always honoured.
The context is thread-local: only the voice main loop closes it; web /api/chat and other callers are unaffected.
"""
import re, threading, time

MOTION_INTENTS = {"drive", "mission", "navigate", "line_follow", "color_track", "gesture", "lidar_mode",
                  "follow", "come", "goto", "explore"}
LOCAL_MOTION_RE = re.compile(
    r"\b(follow|come (?:here|to me|over|back)|go to|drive|navigate|head (?:to|over)|take me to|explore|wander|"
    r"map (?:the|this|my) (?:room|house|place)|back up|reverse|spin|turn (?:left|right|around)|move (?:forward|back|left|right)|"
    r"go (?:forward|back|backward|left|right|straight))\b", re.I)
REJECT_LINE = "I only start moving when you say 'wake up Wally' first."
_L = threading.local()
LAST = {"t": 0.0, "kind": None, "detail": None}


def motion_allowed(matched, awake, followup):
    """matched: this utterance had the wake phrase. awake+not followup: window opened by a bare wake phrase."""
    return bool(matched or (awake and not followup))


def set_context(motion_ok, text=""):
    _L.ok = bool(motion_ok); _L.text = text or ""


def clear_context():
    _L.ok = True; _L.text = ""


def is_open():
    return getattr(_L, "ok", True)


def block(kind, detail=""):
    """None when motion may start; otherwise logs the rejection and returns the spoken refusal."""
    try:                                              # 2026-11-25 arm: disarmed until "wake up Wally" (all threads)
        import gb_arm
        if not gb_arm.is_armed():
            LAST.update(t=time.time(), kind=kind, detail=str(detail)[:200])
            return gb_arm.refuse(kind, detail)
    except Exception as ex:
        print("[WAKEGATE] arm check failed -> refusing:", ex, flush=True); return REJECT_LINE
    if is_open():
        return None
    LAST.update(t=time.time(), kind=kind, detail=str(detail)[:200])
    print(time.strftime("%H:%M:%S"), "[WAKEGATE] rejected %s (no wake phrase in this utterance): %r heard=%r" % (
        kind, str(detail)[:160], getattr(_L, "text", "")[:160]), flush=True)
    return REJECT_LINE


def looks_like_motion(text):
    return bool(LOCAL_MOTION_RE.search(text or ""))
