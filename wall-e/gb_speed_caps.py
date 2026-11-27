"""Shared Wall-E linear speed hard caps (no ROS, no motion).

2026-11-26 (John, GB-48): two separate caps.
  * Manual drive (Jarvis pad/keyboard, joystick, voice teleop) = 0.288 m/s
  * Autonomous explore / navigation = 1.0 m/s hard cap
Stop distances (0.27 / 0.28 / 0.24 m) are not defined here and must not change.
"""
MANUAL_MAX_LIN = 0.288
AUTO_MAX_LIN = 1.0

# Sources that count as autonomous even if a MISSION_LEASE file is briefly missing.
AUTO_SOURCES = frozenset({
    "explore", "nav2", "follow", "guard", "mission", "autonomous",
    "gb_explore", "wander", "nav",
})


def lin_cap_for(src="stream", lease_live=False, joystick=False):
    """Return the linear hard cap in m/s for this command.

    Manual /gb/cmd_in (no live mission lease, including Jarvis) and joystick
    are clamped to MANUAL_MAX_LIN (0.288). A live MISSION_LEASE or an
    autonomous/explore source is clamped to AUTO_MAX_LIN (1.0).
    """
    if joystick or str(src or "").strip().lower() == "joystick":
        return MANUAL_MAX_LIN
    if lease_live:
        return AUTO_MAX_LIN
    s = str(src or "").strip().lower()
    key = s.split(":")[0].split("/")[-1] if s else ""
    if key in AUTO_SOURCES or "explore" in s:
        return AUTO_MAX_LIN
    return MANUAL_MAX_LIN
