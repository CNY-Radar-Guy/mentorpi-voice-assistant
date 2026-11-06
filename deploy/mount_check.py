#!/usr/bin/env python3
"""Compare Docker bind mounts with the paths the safety launchers assume.

The 2026-11-06 17:44 ET preflight saw only:
  /home/pi/docker_ros2/tmp -> /home/ubuntu/share/tmp
The launchers in this tree still mention /home/pi/docker/tmp and /home/ubuntu/shared.
Those strings are not evidence of a mount. If the live bind list does not contain
the launcher's host:container pair, safety files must not be copied.
"""
import json, sys

# What gb_safety_run.sh / gb_guard_run.sh currently assume. Not a guess at the live robot.
ASSUMED = (
    ("/home/pi/docker/tmp", "/home/ubuntu/shared"),
)


def parse_binds(binds):
    """binds from docker inspect HostConfig.Binds. Returns {container: host}."""
    out = {}
    for item in binds or []:
        if not isinstance(item, str) or ":" not in item:
            continue
        host, rest = item.split(":", 1)
        cont = rest.split(":", 1)[0]
        if host and cont:
            out[cont] = host
    return out


def safety_copy_allowed(binds):
    """(ok, problems, mapping). ok is False unless every assumed pair is present."""
    mapping = parse_binds(binds)
    problems = []
    for host, cont in ASSUMED:
        got = mapping.get(cont)
        if got is None:
            problems.append("%s is not in the container bind list" % cont)
        elif got != host:
            problems.append("%s is mounted from %s, not %s" % (cont, got, host))
    return (not problems), problems, mapping


def decide_from_inspect(binds):
    ok, problems, mapping = safety_copy_allowed(binds)
    return {
        "safety_modules": "allowed" if ok else "blocked",
        "problems": problems,
        "binds": mapping,
        "note": "Host Python can still be staged. Do not restart gb-gate, gb-watchdog, gb-guard, or gb-rgbguard while safety_modules is blocked.",
    }


def main(argv):
    raw = argv[1] if len(argv) > 1 else "[]"
    try:
        binds = json.loads(raw)
    except json.JSONDecodeError:
        print("bind list is not JSON", file=sys.stderr)
        return 2
    decision = decide_from_inspect(binds)
    print(json.dumps(decision, indent=2))
    return 0 if decision["safety_modules"] == "allowed" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
