"""gb_cadence.py - Piper length from John's speaking rate.

length_scale > 1 speaks slower. A faster utterance (more words per second)
produces a smaller scale, clamped so the voice stays intelligible.
No audio device is required: pass duration and the transcript.
"""
_scale = None


def length_scale(audio_s, text, base=1.0):
    words = max(1, len((text or "").split()))
    try:
        dur = float(audio_s)
    except (TypeError, ValueError):
        dur = 0.0
    if dur <= 0.05:
        return None
    wps = words / max(0.4, dur)
    # About 2.5 words/s is a comfortable conversational rate.
    scale = (2.5 / wps) * float(base or 1.0)
    return max(0.85, min(1.25, scale))


def observe(audio_s, text):
    global _scale
    s = length_scale(audio_s, text)
    if s is not None:
        _scale = s
    return _scale


def current_scale():
    return _scale


def reset():
    global _scale
    _scale = None
