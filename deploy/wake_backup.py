#!/usr/bin/env python3
"""Pick the wake-env backup without a shell glob.

A `[ -f grokbot.env.bak-wake-* ]` test exits under `set -e` when more than one
backup matches. Callers must use this helper instead.
"""
import os, sys


def select_wake_backup(directory, pointer_text=None):
    """Return (path or None, reason).

    reason is apply-specific, newest, or no-match.
    An apply-specific pointer wins when that file exists.
    Otherwise the newest grokbot.env.bak-wake-* in directory, by mtime then path.
    """
    pointer = (pointer_text or "").strip()
    if pointer and os.path.isfile(pointer):
        return pointer, "apply-specific"
    matches = []
    try:
        names = os.listdir(directory)
    except OSError:
        return None, "no-match"
    for name in names:
        if name.startswith("grokbot.env.bak-wake-"):
            matches.append(os.path.join(directory, name))
    if not matches:
        return None, "no-match"
    matches.sort(key=lambda p: (os.path.getmtime(p), p))
    return matches[-1], "newest"


def main(argv):
    if len(argv) < 2:
        print("usage: wake_backup.py DEST [pointer-file]", file=sys.stderr)
        return 2
    dest = argv[1]
    pointer = ""
    pointer_file = argv[2] if len(argv) > 2 else os.path.join(dest, ".wally-last-wake-backup")
    if os.path.isfile(pointer_file):
        try:
            with open(pointer_file) as f:
                pointer = f.read()
        except OSError:
            pointer = ""
    path, why = select_wake_backup(dest, pointer)
    if path is None:
        print("no wake-field backup next to grokbot.env")
        return 0
    print("%s %s" % (why, path))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
