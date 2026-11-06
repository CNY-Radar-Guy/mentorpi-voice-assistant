"""gb_point.py - pointed-target mission.

Ground one target with a camera point and a valid metric depth. An ambiguous
set of targets produces no drive command. The rover stops before contact,
dwells, then returns to the person who started the mission if that track is
still fresh, otherwise to the start pose.

Goals are not executable across reboot. A status file may exist; load() drops
any goal, speed, or phase it finds.

Planner: a geometric standoff controller. When a Nav2 transport is injected
and LiDAR is the ranging mode, a NavigateToPose goal dict is built and sent.
Nav2 (Humble) is Apache-2.0, https://github.com/ros-navigation/navigation2.
This file does not vendor Nav2. slam_toolbox, used by the existing mapper, is
LGPL-2.1 and is not copied here.

Localization loss, timeout, person-lost on the return, cancel, estop, and
loss of both range sensors all zero the output and drop the goal.
"""
import json, math, os, time
import gb_health

CONTACT_STOP_M = 0.40
APPROACH_MAX_S = 60.0
RETURN_MAX_S = 90.0
PERSON_CONF = 0.75
TARGET_CONF = 0.55


class Nav2Transport(object):
    """Test double or a rosbridge sender. send() must not raise on a dead graph."""

    def send(self, goal):
        return False


def navigate_to_pose_goal(x, y, yaw=0.0, frame="map"):
    """Goal shape compatible with nav2_msgs/NavigateToPose. Not a copy of Nav2 source.

    x and y must already be measured. Yaw is encoded as a quaternion. This does
    not invent a waypoint.
    """
    half = float(yaw) / 2.0
    return {
        "action": "navigate_to_pose",
        "pose": {
            "header": {"frame_id": frame},
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)},
            },
        },
    }


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return number == number and abs(number) < 1e6


def ground_observation(raw, now=None):
    """Drop anything that is not a fresh measurement. Never fills in x, y, or depth."""
    now = time.time() if now is None else float(now)
    raw = raw or {}
    if raw.get("from_llm") or raw.get("from_vision"):
        return {"candidates": [], "person": None, "localized": False, "tf_ok": False,
                "camera_ok": False, "from_llm": True, "from_vision": bool(raw.get("from_vision")), "start": None}
    kept = []
    for cand in raw.get("candidates") or []:
        if not isinstance(cand, dict) or cand.get("from_llm") or cand.get("from_vision"):
            continue
        try:
            conf = float(cand.get("conf") or 0)
        except (TypeError, ValueError):
            continue
        if conf < TARGET_CONF or not cand.get("depth_valid") or cand.get("depth_m") is None:
            continue
        if "depth_t" in cand:
            try:
                if now - float(cand["depth_t"]) > 0.5:
                    continue
            except (TypeError, ValueError):
                continue
        if cand.get("source") == "depth" and cand.get("unit") not in gb_health.VALIDATED_DEPTH_UNITS:
            continue
        item = dict(cand)
        if not (_finite(cand.get("x")) and _finite(cand.get("y"))):
            item.pop("x", None)
            item.pop("y", None)
        kept.append(item)
    pose = raw.get("pose") if isinstance(raw.get("pose"), dict) else None
    tf_ok = bool(pose and pose.get("fresh") and pose.get("frame") in ("map", "odom") and _finite(pose.get("x")) and _finite(pose.get("y")))
    localized = bool(tf_ok and pose.get("frame") == "map")
    start = None
    if localized:
        start = {"x": float(pose["x"]), "y": float(pose["y"]), "yaw": float(pose.get("yaw") or 0.0)}
    person = raw.get("person") if isinstance(raw.get("person"), dict) else None
    if person and (person.get("from_llm") or not person.get("fresh")):
        person = None
    camera_ok = raw.get("camera_ok")
    if camera_ok is None:
        camera_ok = bool(raw.get("camera_t") and now - float(raw.get("camera_t") or 0) <= 0.5)
    return {
        "candidates": kept,
        "person": person,
        "localized": localized,
        "tf_ok": tf_ok,
        "camera_ok": bool(camera_ok),
        "from_llm": False,
        "from_vision": False,
        "start": start,
        "pose": pose if tf_ok else None,
    }


class PointMission(object):
    def __init__(self, status_path):
        self.path = status_path
        self.phase = "idle"
        self.goal = None
        self.commands = []
        self.nav_goals = []
        self.speech = []
        self.epoch = None
        self._t_phase = 0.0
        self._dwell_left = 0.0
        self._nav_sent = False
        self.transport = None
        self.use_nav2 = False

    def load(self):
        """Drop anything that looks like a stored drive goal. Returns None."""
        if not self.path or not os.path.exists(self.path):
            return None
        try:
            with open(self.path) as f:
                raw = f.read()
        except OSError:
            return None
        unsafe = False
        try:
            d = json.loads(raw)
            if isinstance(d, dict) and any(k in d for k in ("goal", "lin", "phase", "ang")):
                unsafe = True
        except Exception:
            unsafe = "goal" in raw or "lin" in raw
        if unsafe:
            try:
                os.remove(self.path)
            except OSError:
                pass
        self.phase = "idle"
        self.goal = None
        return None

    def _status(self, note):
        if not self.path:
            return
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            # Status only. No speeds, no phase, no goal pose.
            with open(self.path, "w") as f:
                json.dump({"executable": False, "note": note, "t": time.time()}, f)
        except OSError:
            pass

    def cancel(self, why="cancel"):
        self.phase = "idle"
        self.goal = None
        self.commands.append({"lin": 0.0, "ang": 0.0, "why": why})
        self.speech.append(why)
        self._status("cancelled")
        return {"lin": 0.0, "ang": 0.0, "phase": "idle", "why": why}

    def start(self, target_name, dwell_s, obs, health, autonomy=False, protections=None):
        """obs keys: candidates [{name, conf, depth_m, depth_valid, bearing, x, y}],
        person {conf, depth_m, fresh, track_id}, localized bool, start {x, y, yaw}.
        """
        self.commands = []
        self.nav_goals = []
        self.speech = []
        protections = protections or {}
        if not autonomy:
            self.cancel("autonomy_off")
            self.speech.append("Autonomy is off. I will not drive to that.")
            return {"ok": False, "say": "Autonomy is off. I will not drive to that."}
        if obs.get("from_vision"):
            self.speech.append("A camera description can't start a drive.")
            return {"ok": False, "say": "A camera description can't start a drive."}
        blocked = gb_health.motion_policy("autonomous", health.mode, protections)
        if not blocked.allow:
            self.cancel(blocked.reason)
            say = blocked.speech or "I won't drive."
            self.speech.append(say)
            return {"ok": False, "say": say}
        cands = []
        for c in obs.get("candidates") or []:
            if _same(c.get("name"), target_name) and float(c.get("conf") or 0) >= TARGET_CONF:
                cands.append(c)
        if len(cands) != 1:
            # Ambiguous or missing: no command is issued at all.
            self.phase = "idle"
            self.goal = None
            say = "I see more than one possible %s, so I won't drive." % (target_name or "target") if len(cands) > 1 else "I don't see one clear target, so I won't drive."
            self.speech.append(say)
            self._status("refused")
            return {"ok": False, "say": say}
        c = cands[0]
        if not c.get("depth_valid") or c.get("depth_m") is None:
            self.phase = "idle"
            say = "I don't have a valid depth to that, so I won't drive."
            self.speech.append(say)
            return {"ok": False, "say": say}
        dist = float(c["depth_m"])
        if dist <= CONTACT_STOP_M:
            say = "That's already inside my stop distance, so I won't drive."
            self.speech.append(say)
            return {"ok": False, "say": say}
        if c.get("sector_clear") is False:
            say = "The way ahead is unknown, so I won't drive."
            self.speech.append(say)
            return {"ok": False, "say": say}
        measured = _finite(c.get("x")) and _finite(c.get("y"))
        nav_ready = self.transport is not None and getattr(self.transport, "ready", lambda: False)()
        if not measured or not obs.get("tf_ok") or not obs.get("localized") or not nav_ready:
            self.phase = "idle"
            self.goal = None
            say = "I won't drive toward that without a measured map pose and navigation."
            self.speech.append(say)
            self._status("refused")
            return {"ok": False, "say": say}
        goal = navigate_to_pose_goal(c["x"], c["y"], frame="map")
        # Stop short of contact: the planner goal is the measured point, and the
        # mission cancels if a later depth sample is inside CONTACT_STOP_M.
        if not self.transport.send(goal):
            self.phase = "idle"
            say = "Navigation did not accept the goal, so I won't drive."
            self.speech.append(say)
            return {"ok": False, "say": say}
        self.nav_goals.append(goal)
        self.goal = {
            "name": target_name,
            "dwell_s": max(0.0, float(dwell_s)),
            "start": dict(obs.get("start") or {"x": 0, "y": 0, "yaw": 0}),
            "person_id": (obs.get("person") or {}).get("track_id"),
            "tx": float(c["x"]), "ty": float(c["y"]),
        }
        self.epoch = health.goal_epoch
        self.phase = "approach"
        self._t_phase = 0.0
        self._nav_sent = True
        self._dwell_left = self.goal["dwell_s"]
        self._status("running-not-a-goal")
        return {"ok": True, "say": "I'll go to the %s, wait, and come back." % target_name, "phase": "approach"}

    def tick(self, dt, obs, health, protections=None):
        protections = protections or {}
        if self.phase == "idle" or self.goal is None:
            return {"lin": 0.0, "ang": 0.0, "phase": "idle"}
        if health.goal_epoch != self.epoch or health.mode == "none":
            say = gb_health.NO_RANGE if health.mode == "none" else "stopped"
            self.speech.append(say)
            return self.cancel(say)
        pol = gb_health.motion_policy("autonomous", health.mode, protections)
        if not pol.allow:
            self.speech.append(pol.speech or pol.reason)
            return self.cancel(pol.reason)
        self._t_phase += float(dt)
        if self.phase == "approach":
            if self._t_phase > APPROACH_MAX_S:
                self._cancel_nav()
                self.speech.append("I took too long, so I stopped.")
                return self.cancel("timeout")
            dist = _target_depth(obs, self.goal["name"])
            if dist is not None and dist <= CONTACT_STOP_M:
                self._cancel_nav()
                self.speech.append("That's inside my stop distance, so I stopped.")
                return self.cancel("contact")
            status = self.transport.poll() if self.transport is not None else "unavailable"
            if status in ("unavailable", "failed", "canceled", "rejected"):
                self.speech.append("Navigation stopped, so I stopped.")
                return self.cancel("nav2")
            if status == "succeeded":
                self.phase = "dwell"
                self._dwell_left = self.goal["dwell_s"]
                self._t_phase = 0.0
                cmd = {"lin": 0.0, "ang": 0.0, "phase": "dwell", "why": "nav_arrived"}
                self.commands.append(cmd)
                return cmd
            cmd = {"lin": 0.0, "ang": 0.0, "phase": "approach", "why": "nav_active"}
            self.commands.append(cmd)
            return cmd
        if self.phase == "dwell":
            self._dwell_left -= float(dt)
            if self._dwell_left > 0:
                cmd = {"lin": 0.0, "ang": 0.0, "phase": "dwell", "left": self._dwell_left}
                self.commands.append(cmd)
                return cmd
            self._begin_return(obs)
            cmd = {"lin": 0.0, "ang": 0.0, "phase": self.phase, "why": "dwell_done"}
            self.commands.append(cmd)
            return cmd
        if self.phase in ("return_person", "return_start"):
            if self._t_phase > RETURN_MAX_S:
                self._cancel_nav()
                return self.cancel("timeout")
            if not obs.get("localized") or not obs.get("tf_ok"):
                self._cancel_nav()
                self.speech.append("I lost localization, so I stopped.")
                return self.cancel("localization")
            status = self.transport.poll() if self.transport is not None else "unavailable"
            if status in ("unavailable", "failed", "canceled", "rejected"):
                return self.cancel("nav2")
            if status == "succeeded":
                why = "back_with_person" if self.phase == "return_person" else "start_pose"
                cmd = {"lin": 0.0, "ang": 0.0, "phase": "done", "why": why}
                self.phase = "idle"
                self.goal = None
                self.commands.append(cmd)
                self._status("done")
                return cmd
            cmd = {"lin": 0.0, "ang": 0.0, "phase": self.phase, "why": "nav_active"}
            self.commands.append(cmd)
            return cmd
        return self.cancel("idle")

    def _cancel_nav(self):
        if self.transport is not None:
            try:
                self.transport.cancel_all()
            except Exception:
                pass

    def _begin_return(self, obs):
        self._t_phase = 0.0
        person = obs.get("person") or {}
        px, py = person.get("x"), person.get("y")
        if (person.get("fresh") and float(person.get("conf") or 0) >= PERSON_CONF
                and _finite(px) and _finite(py) and obs.get("tf_ok")):
            goal = navigate_to_pose_goal(px, py, frame="map")
            self.phase = "return_person"
        else:
            start = self.goal.get("start") or {}
            if not (_finite(start.get("x")) and _finite(start.get("y")) and obs.get("tf_ok")):
                self.speech.append("I don't have a pose to return to, so I stopped.")
                self.cancel("tf")
                return
            goal = navigate_to_pose_goal(start["x"], start["y"], start.get("yaw") or 0.0, frame="map")
            self.phase = "return_start"
        if self.transport is None or not self.transport.send(goal):
            self.cancel("nav2")
            return
        self.nav_goals.append(goal)


def _same(a, b):
    return (a or "").strip().lower() == (b or "").strip().lower()


def _target_depth(obs, name):
    hits = [c for c in (obs.get("candidates") or []) if _same(c.get("name"), name) and c.get("depth_valid") and c.get("depth_m") is not None]
    if len(hits) != 1:
        return None
    if hits[0].get("sector_clear") is False:
        return None
    return float(hits[0]["depth_m"])
