"""NavigateToPose session.

The rclpy client is constructed only when nav2_msgs imports. Tests use
ScriptedNav. A missing server is "unavailable", which the mission treats as
stop. This module never publishes a velocity.
"""


class ScriptedNav(object):
    """poll() returns the next scripted status. send() records the goal."""

    def __init__(self, polls=None):
        self.polls = list(polls or [])
        self.goals = []
        self.cancelled = False

    def ready(self):
        return True

    def send(self, goal):
        try:
            pos = goal["pose"]["pose"]["position"]
            float(pos["x"]); float(pos["y"])
        except (KeyError, TypeError, ValueError):
            return False
        if goal.get("pose", {}).get("header", {}).get("frame_id") != "map":
            return False
        self.goals.append(goal)
        return True

    def poll(self):
        if not self.polls:
            return "active"
        return self.polls.pop(0)

    def cancel_all(self):
        self.cancelled = True


class UnavailableNav(object):
    def ready(self):
        return False

    def send(self, goal):
        return False

    def poll(self):
        return "unavailable"

    def cancel_all(self):
        return None


class RclpyNav2(object):
    """Real action client. Imported only from the container node."""

    def __init__(self, node, action_type, client):
        self.node = node
        self.action_type = action_type
        self.client = client
        self._goal = None
        self._result = None
        self._status = "idle"

    def ready(self):
        try:
            return bool(self.client.server_is_ready())
        except Exception:
            return False

    def send(self, goal):
        if not self.ready():
            self._status = "unavailable"
            return False
        try:
            pos = goal["pose"]["pose"]["position"]
            quat = goal["pose"]["pose"]["orientation"]
            msg = self.action_type.Goal()
            msg.pose.header.frame_id = "map"
            msg.pose.pose.position.x = float(pos["x"])
            msg.pose.pose.position.y = float(pos["y"])
            msg.pose.pose.orientation.z = float(quat["z"])
            msg.pose.pose.orientation.w = float(quat["w"])
        except Exception:
            self._status = "failed"
            return False
        self._status = "active"
        send = self.client.send_goal_async(msg)
        send.add_done_callback(self._accepted)
        return True

    def _accepted(self, future):
        try:
            handle = future.result()
        except Exception:
            self._status = "failed"
            return
        if handle is None or not handle.accepted:
            self._status = "failed"
            return
        result = handle.get_result_async()
        result.add_done_callback(self._finished)

    def _finished(self, future):
        try:
            wrapped = future.result()
            status = int(getattr(wrapped, "status", 0))
        except Exception:
            self._status = "failed"
            return
        # action_msgs/GoalStatus: STATUS_SUCCEEDED = 4
        self._status = "succeeded" if status == 4 else "failed"

    def poll(self):
        return self._status

    def cancel_all(self):
        try:
            self.client.cancel_all_goals_async()
        except Exception:
            pass
        self._status = "canceled"


def make_session(node=None):
    """Return a live client when the messages import, otherwise unavailable."""
    try:
        import rclpy
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
    except Exception:
        return UnavailableNav()
    if node is None:
        return UnavailableNav()
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=2.0):
        return UnavailableNav()
    return RclpyNav2(node, NavigateToPose, client)
