"""Process-matching helpers shared with unit tests.

Jarvis remote.py keeps a standalone copy of is_live_explore_cmd (it is piped
over SSH and cannot import grokbot modules). Keep the two implementations in
sync; tests in this tree import this module.
"""


def is_live_explore_cmd(cmd):
    """True only for a Python process whose argv includes the gb_explore.py script.

    grep/sed/zsh/journalctl (or python -c) that merely mention the filename must not match.
    """
    if not cmd:
        return False
    args = [a for a in str(cmd).replace("\0", " ").split() if a]
    if not args:
        return False
    exe = args[0].rstrip("/").split("/")[-1].lower()
    if not (exe == "python" or exe.startswith("python3") or exe.startswith("python2")):
        return False
    if "-c" in args:
        return False
    for a in args[1:]:
        base = a.rstrip("/").split("/")[-1]
        if base == "gb_explore.py":
            return True
    return False
