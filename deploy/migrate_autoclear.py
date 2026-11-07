#!/usr/bin/env python3
"""Set ESTOP_AUTOCLEAR_S to 0. One key, one backup. Wake lines and the greeting are left as they are."""
import os, re, sys, time

KEY = "ESTOP_AUTOCLEAR_S"


def migrate_text(text):
    found = False
    changed = False
    out = []
    for line in text.splitlines(keepends=True):
        m = re.match(r"^([A-Za-z0-9_]+)=(.*)$", line.rstrip("\n"))
        if not m or m.group(1) != KEY:
            out.append(line)
            continue
        found = True
        nl = "\n" if line.endswith("\n") else ""
        val = m.group(2).strip().strip('"').strip("'")
        if val == "0":
            out.append(line)
            continue
        changed = True
        out.append(KEY + "=0" + nl)
    body = "".join(out)
    if not found:
        if body and not body.endswith("\n"):
            body += "\n"
        body += KEY + "=0\n"
        changed = True
    return body, changed


def migrate_file(path):
    with open(path) as f:
        original = f.read()
    updated, changed = migrate_text(original)
    if not changed:
        return path, False
    backup = path + ".bak-autoclear-" + time.strftime("%Y%m%d-%H%M%S")
    with open(backup, "w") as f:
        f.write(original)
    tmp = path + ".tmp-autoclear"
    with open(tmp, "w") as f:
        f.write(updated)
    os.replace(tmp, path)
    pointer = os.path.join(os.path.dirname(os.path.abspath(path)), ".wally-last-autoclear-backup")
    with open(pointer, "w") as f:
        f.write(backup + "\n")
    return backup, True


def main(argv):
    if len(argv) != 2:
        print("usage: migrate_autoclear.py grokbot.env", file=sys.stderr)
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
