#!/usr/bin/env python3
"""Overwrite the software wake fields. Leave every other line alone.

WAKE_WORDS becomes exactly "wake up wally". WAKE_ANYWHERE and WAKE_FUZZ are
cleared so an alias or a similarity score cannot wake. WAKE_ACK, the startup
greeting, and the identity line are not rewritten. The WonderEcho chip is not
mentioned here and is not reflashed.
"""
import os, re, sys, time

WAKE_KEYS = ("WAKE_WORDS", "WAKE_ANYWHERE", "WAKE_FUZZ")
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
        new = EXACT if key == "WAKE_WORDS" else ""
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
