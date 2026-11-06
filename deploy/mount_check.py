#!/usr/bin/env python3
"""Decide whether safety files may be copied onto the shared mount.

Live evidence is `docker inspect` `.Mounts` (Source -> Destination). An optional
`/proc/1/mountinfo` from inside the container must name the same destination
when that text is supplied.

`HostConfig.Binds` is the saved create-time list. On this robot it still says
`/home/pi/docker_ros2/tmp -> /home/ubuntu/share/tmp`, and those paths do not
exist. The live `.Mounts` entry and mountinfo are
`/home/pi/docker/tmp -> /home/ubuntu/shared` (inode 2130768, 2026-11-06 17:49 ET).
A saved bind string is never enough to allow a copy, and it never overrides
`.Mounts`.

A parent-supplied realpath map can accept a symlink whose canonical path equals
the assumed host path. This module does not call os.path.realpath itself.
"""
import json, sys

# What gb_safety_run.sh / gb_guard_run.sh write through. Confirmed live, not a guess.
ASSUMED = (
    ("/home/pi/docker/tmp", "/home/ubuntu/shared"),
)


def parse_binds(binds):
    """Saved HostConfig.Binds strings. Returns {container: host}. Not live evidence."""
    out = {}
    for item in binds or []:
        if not isinstance(item, str) or ":" not in item:
            continue
        host, rest = item.split(":", 1)
        cont = rest.split(":", 1)[0]
        if host and cont:
            out[cont] = host
    return out


def parse_mounts(mounts):
    """docker inspect .Mounts entries. Returns {container: host}."""
    out = {}
    for item in mounts or []:
        if not isinstance(item, dict):
            continue
        host = item.get("Source") or item.get("source")
        cont = item.get("Destination") or item.get("destination")
        if host and cont:
            out[cont] = host
    return out


def parse_inspect(obj):
    """Normalize a full inspect object, a .Mounts list, or a legacy binds list."""
    if obj is None:
        return {"mounts": {}, "stale_binds": {}, "source": "empty"}
    if isinstance(obj, list) and not obj:
        return {"mounts": {}, "stale_binds": {}, "source": "empty"}
    if isinstance(obj, list) and isinstance(obj[0], str):
        return {"mounts": {}, "stale_binds": parse_binds(obj), "source": "hostconfig_binds"}
    if isinstance(obj, list) and isinstance(obj[0], dict) and ("Mounts" in obj[0] or "HostConfig" in obj[0]):
        obj = obj[0]
    if isinstance(obj, list) and isinstance(obj[0], dict):
        return {"mounts": parse_mounts(obj), "stale_binds": {}, "source": "mounts"}
    if isinstance(obj, dict) and ("Mounts" in obj or "HostConfig" in obj):
        stale = {}
        hc = obj.get("HostConfig") or {}
        if isinstance(hc, dict):
            stale = parse_binds(hc.get("Binds") or [])
        return {"mounts": parse_mounts(obj.get("Mounts") or []), "stale_binds": stale, "source": "inspect"}
    if isinstance(obj, dict) and ("Source" in obj or "Destination" in obj):
        return {"mounts": parse_mounts([obj]), "stale_binds": {}, "source": "mounts"}
    return {"mounts": {}, "stale_binds": {}, "source": "empty"}


def paths_equivalent(assumed_host, actual_host, realpaths):
    """Literal match, or the same parent-supplied canonical path for both sides.

    A one-sided realpath is not equivalence.
    """
    if assumed_host == actual_host:
        return True
    if not realpaths:
        return False
    a = realpaths.get(assumed_host)
    b = realpaths.get(actual_host)
    return bool(a) and bool(b) and a == b


def mountinfo_has_dest(text, dest):
    """True when a mountinfo line's mount point is exactly dest.

    Fields: id parent major:minor root mountpoint options ...
    """
    if not text or not str(text).strip():
        return False
    for line in str(text).splitlines():
        parts = line.split()
        if len(parts) >= 5 and parts[4] == dest:
            return True
    return False


def safety_copy_allowed(evidence, realpaths=None, mountinfo=None, inodes=None):
    """(ok, problems, mapping).

    evidence: inspect dict, .Mounts list, or a HostConfig.Binds string list.
    A binds list alone is refused. mountinfo is optional; empty text is ignored.
    When mountinfo is non-empty it must contain the container destination.
    inodes is (host_inode, container_inode) or None. Both must match when both
    are present. A one-sided inode is not a reason to block.
    """
    parsed = evidence if isinstance(evidence, dict) and "mounts" in evidence and "source" in evidence else parse_inspect(evidence)
    mapping = parsed["mounts"]
    problems = []
    for host, cont in ASSUMED:
        got = mapping.get(cont)
        if got is None:
            if parsed["source"] == "hostconfig_binds":
                problems.append("HostConfig.Binds is not live mount evidence; docker inspect .Mounts was not supplied")
            else:
                problems.append("%s is not in docker inspect .Mounts" % cont)
            continue
        if not paths_equivalent(host, got, realpaths):
            problems.append("%s is mounted from %s, not %s (no verified realpath equivalence)" % (cont, got, host))
            continue
        if mountinfo is not None and str(mountinfo).strip() and not mountinfo_has_dest(mountinfo, cont):
            problems.append("mountinfo does not show %s" % cont)
    if inodes is not None:
        host_i, cont_i = inodes
        if host_i and cont_i and str(host_i) != str(cont_i):
            problems.append("host inode %s != container inode %s; refusing to write" % (host_i, cont_i))
    return (not problems), problems, mapping


def ignored_stale_binds(evidence):
    parsed = evidence if isinstance(evidence, dict) and "source" in evidence else parse_inspect(evidence)
    notes = []
    for cont, host in (parsed.get("stale_binds") or {}).items():
        live = (parsed.get("mounts") or {}).get(cont)
        if live != host:
            notes.append("ignored stale HostConfig.Binds %s -> %s" % (host, cont))
    return notes


def decide_from_inspect(evidence, realpaths=None, mountinfo=None, inodes=None):
    ok, problems, mapping = safety_copy_allowed(evidence, realpaths, mountinfo, inodes)
    return {
        "safety_modules": "allowed" if ok else "blocked",
        "problems": problems,
        "binds": mapping,
        "ignored_stale_binds": ignored_stale_binds(evidence),
        "source": (evidence if isinstance(evidence, dict) and "source" in evidence else parse_inspect(evidence))["source"],
        "note": "Copy only through the live .Mounts source. Do not restart gb-gate, gb-watchdog, gb-guard, or gb-rgbguard while safety_modules is blocked.",
    }


def _load_json(path):
    with open(path) as f:
        return json.load(f)


def main(argv):
    inspect_path = None
    mountinfo_path = None
    realpaths = None
    inodes = None
    positional = []
    args = list(argv[1:])
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--inspect":
            inspect_path = args[i + 1]
            i += 2
        elif a == "--mountinfo":
            mountinfo_path = args[i + 1]
            i += 2
        elif a == "--realpaths":
            realpaths = _load_json(args[i + 1])
            i += 2
        elif a == "--inodes":
            host_i, cont_i = args[i + 1].split(",", 1)
            inodes = (host_i, cont_i)
            i += 2
        else:
            positional.append(a)
            i += 1
    mountinfo = None
    if inspect_path:
        evidence = _load_json(inspect_path)
    elif positional:
        try:
            evidence = json.loads(positional[0])
        except json.JSONDecodeError:
            print("mount evidence is not JSON", file=sys.stderr)
            return 2
        if len(positional) > 1:
            try:
                realpaths = json.loads(positional[1])
            except json.JSONDecodeError:
                print("realpath map is not JSON", file=sys.stderr)
                return 2
    else:
        evidence = []
    if mountinfo_path:
        try:
            with open(mountinfo_path) as f:
                mountinfo = f.read()
        except OSError as ex:
            print("mountinfo unreadable: %s" % ex, file=sys.stderr)
            return 2
    decision = decide_from_inspect(evidence, realpaths, mountinfo, inodes)
    print(json.dumps(decision, indent=2))
    return 0 if decision["safety_modules"] == "allowed" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
