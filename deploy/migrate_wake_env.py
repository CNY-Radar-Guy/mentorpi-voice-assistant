#!/usr/bin/env python3
"""Surgically fix WAKE_WORDS, WAKE_ANYWHERE, and WAKE_PROMPT.

Backs up the env file next to itself. Does not rewrite other keys, and does
not open secrets.env. "wake up wally" is added if WAKE_WORDS already exists.
Hello/Hey/Hi Wally and a bare wally token are removed. WAKE_ACK is left as-is.
"""
import os, re, sys, time

WAKE_KEYS = ("WAKE_WORDS", "WAKE_ANYWHERE", "WAKE_PROMPT")
DROP_PHRASES = {"hello wally", "hey wally", "hi wally", "wally"}
EXACT = "wake up wally"


def _split(value):
    return [p.strip().lower() for p in value.split(",") if p.strip()]


def migrate_text(text):
    changed = False
    out = []
    for line in text.splitlines(keepends=True):
        m = re.match(r"^([A-Za-z0-9_]+)=(.*)$", line.rstrip("\n"))
        if not m or m.group(1) not in WAKE_KEYS:
            out.append(line)
            continue
        key, val = m.group(1), m.group(2)
        nl = "\n" if line.endswith("\n") else ""
        if key == "WAKE_PROMPT":
            new = val
            if val.strip().strip('"').strip("'").lower() in ("wally.", "wally", "hello wally", "hey wally", "hi wally"):
                new = "Wake up Wally."
            if new != val:
                changed = True
            out.append(key + "=" + new + nl)
            continue
        parts = [p for p in _split(val) if p not in DROP_PHRASES]
        if key == "WAKE_WORDS" and EXACT not in parts:
            parts.insert(0, EXACT)
        new = ",".join(parts)
        if new != val:
            changed = True
        out.append(key + "=" + new + nl)
    body = "".join(out)
    if text.endswith("\n") and body and not body.endswith("\n"):
        body += "\n"
    return body, changed


def migrate_file(path):
    with open(path) as f:
        original = f.read()
    updated, changed = migrate_text(original)
    if not changed:
        return path, False
    backup = path + ".bak-wake-" + time.strftime("%Y%m%d-%H%M%S")
    with open(backup, "w") as f:
        f.write(original)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(updated)
    os.replace(tmp, path)
    pointer = os.path.join(os.path.dirname(os.path.abspath(path)), ".wally-last-wake-backup")
    with open(pointer, "w") as f:
        f.write(backup + "\n")
    return backup, True


def main(argv):
    if len(argv) != 2:
        print("usage: migrate_wake_env.py grokbot.env", file=sys.stderr)
        return 2
    path = argv[1]
    if os.path.basename(path) == "secrets.env":
        print("refusing to edit secrets.env", file=sys.stderr)
        return 3
    backup, changed = migrate_file(path)
    print("unchanged" if not changed else "updated; backup " + backup)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
