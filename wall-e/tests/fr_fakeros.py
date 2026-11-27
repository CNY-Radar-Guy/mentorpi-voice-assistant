"""fr_fakeros.py - test-only fake ROS + simulated world/gate for the 2026-11-27 fusion stuck-recovery tests.
NO real ROS, NO hardware: commands go to an in-memory simulated robot. Never imported by production code."""
import json, math, os, sys, tempfile, threading, time, types

INF = float("inf")


# ------------------------------------------------------------------ fake ROS modules
class _V(object):
    def __init__(self, **kw):
        self.x = self.y = self.z = 0.0; self.w = 1.0
        self.__dict__.update(kw)


class Twist(object):
    def __init__(self):
        self.linear = _V(); self.angular = _V()


class String(object):
    def __init__(self, data=""):
        self.data = data


class _Any(object):
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, k):
        return _Any()

    def __call__(self, *a, **k):
        return _Any()


class Pub(object):
    def __init__(self, bus, topic):
        self.bus, self.topic = bus, topic

    def publish(self, m):
        self.bus.on_publish(self.topic, m)


class Bus(object):
    def __init__(self):
        self.subs = {}; self.pubs = []; self.handler = None

    def on_publish(self, topic, m):
        self.pubs.append((time.time(), topic, getattr(getattr(m, "linear", None), "x", None),
                          getattr(getattr(m, "angular", None), "z", None)))
        if self.handler:
            self.handler(topic, m)

    def deliver(self, topic, m):
        for cb in list(self.subs.get(topic, [])):
            cb(m)


BUS = Bus()


class Node(object):
    def __init__(self, name, *a, **k):
        self._name = name

    def create_publisher(self, t, topic, q=10):
        return Pub(BUS, topic)

    def create_subscription(self, t, topic, cb, q=10):
        BUS.subs.setdefault(topic, []).append(cb)
        return object()

    def create_timer(self, p, cb):
        return object()

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)

    def get_clock(self):
        return _Any()

    def destroy_node(self):
        pass


class Executor(object):
    def __init__(self, *a, **k):
        pass

    def add_node(self, n):
        pass

    def spin(self):
        while True:
            time.sleep(1.0)

    def shutdown(self):
        pass


def install():
    if "rclpy" in sys.modules and getattr(sys.modules["rclpy"], "_fr_fake", False):
        return
    m = lambda n: sys.modules.setdefault(n, types.ModuleType(n))
    rclpy = m("rclpy"); rclpy._fr_fake = True
    rclpy.init = lambda *a, **k: None; rclpy.shutdown = lambda *a, **k: None; rclpy.spin = lambda *a, **k: None
    m("rclpy.node").Node = Node; rclpy.node = sys.modules["rclpy.node"]
    m("rclpy.executors").MultiThreadedExecutor = Executor
    q = m("rclpy.qos"); q.qos_profile_sensor_data = 10; q.QoSProfile = _Any
    q.DurabilityPolicy = _Any(); q.ReliabilityPolicy = _Any(); q.HistoryPolicy = _Any()
    m("rclpy.action").ActionClient = _Any
    sg = m("rclpy.signals"); sg.SignalHandlerOptions = types.SimpleNamespace(NO=0)
    tm = m("rclpy.time"); tm.Time = _Any; rclpy.time = tm
    m("geometry_msgs"); gm = m("geometry_msgs.msg"); gm.Twist = Twist; gm.PoseStamped = _Any
    m("sensor_msgs"); sm = m("sensor_msgs.msg"); sm.LaserScan = _Any; sm.Imu = _Any; sm.Image = _Any; sm.CameraInfo = _Any
    m("nav_msgs"); nm = m("nav_msgs.msg"); nm.Odometry = _Any; nm.OccupancyGrid = _Any
    m("std_msgs"); stm = m("std_msgs.msg"); stm.String = String
    tf = m("tf2_ros")

    class Buf(object):
        def lookup_transform(self, *a, **k):
            raise RuntimeError("no tf in sim")
    tf.Buffer = Buf; tf.TransformListener = _Any


# ------------------------------------------------------------------ world
def ray_circle(ox, oy, dx, dy, c):
    cx, cy, r = c; fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy; cc = fx * fx + fy * fy - r * r; disc = b * b - cc
    if disc < 0:
        return INF
    t = -b - math.sqrt(disc)
    return t if t > 1e-6 else INF


def ray_seg(ox, oy, dx, dy, s):
    x1, y1, x2, y2 = s; ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return INF
    t = ((x1 - ox) * ey - (y1 - oy) * ex) / den; u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
    return t if t > 1e-6 and 0 <= u <= 1 else INF


ROOM = [(-3, -3, 3, -3), (3, -3, 3, 3), (3, 3, -3, 3), (-3, 3, -3, -3)]


class World(object):
    N = 400

    def __init__(self, circles=(), segs=(), pose=(0.0, 0.0, 0.0)):
        self.circles = list(circles); self.segs = list(segs) + ROOM
        self.x, self.y, self.th = pose
        self.min_gap = INF

    def ray(self, ox, oy, a):
        dx, dy = math.cos(a), math.sin(a)
        d = INF
        for c in self.circles:
            d = min(d, ray_circle(ox, oy, dx, dy, c))
        for s in self.segs:
            d = min(d, ray_seg(ox, oy, dx, dy, s))
        return d

    def scan(self):
        inc = 2 * math.pi / self.N
        r = []
        for i in range(self.N):
            d = self.ray(self.x, self.y, self.th + i * inc)
            r.append(d if 0.10 <= d <= 10.0 else float("nan"))
        return r, 0.0, inc

    def body_gap(self):
        """Smallest distance from any obstacle to the real chassis rectangle (negative = collision)."""
        import gb_obstacle as OB
        c, s = math.cos(self.th), math.sin(self.th)
        best = INF
        def rect_d(px, py):
            dx = px - self.x; dy = py - self.y
            xr = c * dx + s * dy; yr = -s * dx + c * dy
            ex = max(xr - OB.BODY_FRONT, -OB.BODY_REAR - xr, 0.0); ey = max(abs(yr) - OB.BODY_HALF_W, 0.0)
            if ex == 0 and ey == 0:
                return -min(OB.BODY_FRONT - xr, xr + OB.BODY_REAR, OB.BODY_HALF_W - abs(yr))
            return math.hypot(ex, ey)
        for cx, cy, r in self.circles:
            best = min(best, rect_d(cx, cy) - r)
        for x1, y1, x2, y2 in self.segs:
            for k in range(41):
                t = k / 40.0
                best = min(best, rect_d(x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
        return best

    def step(self, v, w, dt):
        self.th += w * dt
        self.x += v * dt * math.cos(self.th); self.y += v * dt * math.sin(self.th)
        self.min_gap = min(self.min_gap, self.body_gap())

    def camera(self, now, mode="ok"):
        """Synthetic depth summary in the gb_fusion_recovery.depth_summary format (robot frame)."""
        import gb_fusion_recovery as FR
        if mode == "offline":
            return None
        t = now - (3.0 if mode == "stale" else 0.0)
        bins = []; obs = []
        lo = math.radians(-36); bw = math.radians(6)
        for b in range(12):
            a0 = lo + b * bw; a1 = a0 + bw; ac = (a0 + a1) / 2
            d = self.ray(self.x + FR.CAM_X * math.cos(self.th), self.y + FR.CAM_X * math.sin(self.th), self.th + ac)
            close = d < FR.DEPTH_MIN_VALID_M
            valid = 0.05 if (close or mode == "lowvalid") else 0.62
            bins.append([round(a0, 4), round(a1, 4), valid, 0 if close else 12, 0])
            for k in range(7):
                a = a0 + (k + 0.5) * bw / 7
                ox = self.x + FR.CAM_X * math.cos(self.th); oy = self.y + FR.CAM_X * math.sin(self.th)
                d = self.ray(ox, oy, self.th + a)
                if FR.DEPTH_MIN_VALID_M <= d <= 1.1:
                    obs.append([round(FR.CAM_X + d * math.cos(a), 3), round(d * math.sin(a), 3)])
        return {"t": t, "valid": 0.1 if mode == "lowvalid" else 0.62, "bins": bins, "obs": obs}


# ------------------------------------------------------------------ simulated gate (REAL Gate.guard + Gate._fusion_nudge)
def load_gate(root):
    install()
    if root not in sys.path:
        sys.path.insert(0, root)
    sys.path.insert(0, os.path.join(root, "gb_safety"))
    import importlib
    import gb_gate
    importlib.reload(gb_gate)
    return gb_gate


class GateStandIn(object):
    def __init__(self, gate_mod, fz):
        self.G = gate_mod
        self.guard = types.MethodType(gate_mod.Gate.guard, self)
        fn = getattr(gate_mod.Gate, "_fusion_nudge", None)
        if fn is not None:
            self._fusion_nudge = types.MethodType(fn, self)
        self.fz = fz; self.slow_why = ""; self._thr = {}; self.rgb_chk = 1e18
        self.front = self.rear = self.sweep = INF
        self.mode = "stream"; self.src = "stream"; self.estop = False; self.trip_t = 0.0
        self.fr_cam = None; self.fr_last = None; self.logs = []; self.lease = True; self.bench = False

    def log(self, m):
        self.logs.append(m)

    def log_throttle(self, m, every=2.0):
        self.logs.append(m)

    def bench_off(self):
        return self.bench

    def _lease_live(self):
        return self.lease

    def _lin_cap(self, src=None, joystick=False):
        return 1.0 if self.lease else 0.288


def make_fusion(world, now, depth_valid=0.6):
    import gb_obstacle as OB
    fz = OB.Fusion()
    fz.on_odom(now, 0.0, 0.0, world.x, world.y, world.th)
    fz.imu_t = now
    D = OB.DepthView(now); D.fwd = INF; D.valid_frac = depth_valid
    fz.on_depth(D)
    r, a0, inc = world.scan()
    fz.on_lidar(OB.lidar_analyze(r, a0, inc, 0.10, 10.0, now))
    return fz


class Sim(object):
    """Runs the world + gate at 20 Hz in a thread and feeds a (fake-ROS) gb_explore node."""

    def __init__(self, root, world, cam_mode="ok", tmp=None):
        import gb_obstacle as OB
        self.OB = OB
        self.root = root; self.world = world; self.cam_mode = cam_mode
        self.G = load_gate(root)
        self.cmd = (0.0, 0.0, 0.0); self.trips = 0; self.last_trip = ""
        self.allowed = 0; self.fusion_out = []; self.max_lin = 0.0; self.run = True
        self.gate = GateStandIn(self.G, make_fusion(world, time.time()))
        self.lock = threading.Lock()
        BUS.handler = self.on_pub
        self.min_normal_front_moving = INF

    def on_pub(self, topic, m):
        if topic == "/gb/cmd_in":
            with self.lock:
                self.cmd = (m.linear.x, m.angular.z, time.time())

    def _scan_msg(self, r, a0, inc):
        return types.SimpleNamespace(ranges=r, angle_min=a0, angle_increment=inc, range_min=0.10, range_max=10.0)

    def _odom_msg(self):
        th = self.world.th
        o = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(th / 2), w=math.cos(th / 2))
        p = types.SimpleNamespace(position=types.SimpleNamespace(x=self.world.x, y=self.world.y), orientation=o)
        return types.SimpleNamespace(pose=types.SimpleNamespace(pose=p))

    def loop(self, cont_dir):
        k = 0; dt = 0.05
        while self.run:
            t0 = time.time(); now = t0
            with self.lock:
                lin, ang, ct = self.cmd
            fz = make_fusion(self.world, now)
            g = self.gate; g.fz = fz; g.fr_cam = self.world.camera(now, self.cam_mode)
            L = fz.lidar; g.front, g.rear, g.sweep = fz.front(now)[0], L.rev, L.sweep
            v = w = 0.0
            if now - ct <= 0.5 and (lin or ang):
                l, a, why = g.guard(lin, ang, stream=True)
                if why:
                    self.trips += 1; self.last_trip = why; g.trip_t = now
                else:
                    v, w = l, a
                    if g.slow_why.startswith("fusion micro-nudge"):
                        self.allowed += 1; self.fusion_out.append((v, w))
                    elif v > 0:
                        self.min_normal_front_moving = min(self.min_normal_front_moving, g.front)
            self.max_lin = max(self.max_lin, abs(v))
            self.world.step(v, w, dt)
            if k % 2 == 0:
                r, a0, inc = self.world.scan()
                BUS.deliver("/scan_raw", self._scan_msg(r, a0, inc))
                BUS.deliver("/odom", self._odom_msg())
                st = {"t": now, "mode": "stream" if (lin or ang) else "idle", "trips": self.trips,
                      "last_trip": self.last_trip, "estop": False,
                      "front": round(g.front, 3) if math.isfinite(g.front) else None,
                      "rear": round(L.rev, 3) if math.isfinite(L.rev) else None,
                      "sweep": round(L.sweep, 3) if math.isfinite(L.sweep) else None,
                      "fusion": {"depth_fwd": None, "fresh": fz.fresh(now)}, "sensors": fz.snapshot(now)}
                FR = self.G.FR
                if FR is not None:
                    st["fr"] = {"cam": FR.cam_for_status(g.fr_cam, now)}
                BUS.deliver("/gb/gate_status", String(json.dumps(st)))
                with open(os.path.join(cont_dir, "host_hb.tmp"), "w") as f:
                    f.write(str(time.time()))
                os.replace(os.path.join(cont_dir, "host_hb.tmp"), os.path.join(cont_dir, "host_hb"))
            k += 1
            time.sleep(max(0.0, dt - (time.time() - t0)))


def run_mission(root, plan, world, cam_mode="ok", timeout=90.0):
    """Run gb_explore.node_main (non-dry, fake ROS, simulated gate). Returns (result dict, sim, log lines)."""
    install()
    tmp = tempfile.mkdtemp(prefix="frmis_")
    shared = os.path.join(tmp, "shared"); cont = os.path.join(shared, "grokbot", "explore")
    gbs = os.path.join(shared, "gb_safety")
    for d in (cont, gbs, os.path.join(shared, "gb_guard")):
        os.makedirs(d, exist_ok=True)
    os.environ["ROS_SHARED_CONTAINER"] = shared; os.environ["ROS_SHARED_HOST"] = shared
    os.environ["GB_OBSTACLE_DIR"] = root
    os.environ["GB_ESTOP_FILE"] = os.path.join(gbs, "ESTOP")
    os.environ["GATE_ARMED_FILE"] = os.path.join(gbs, "ARMED")
    open(os.path.join(gbs, "ARMED"), "w").write("{}")
    if root not in sys.path:
        sys.path.insert(0, root)
    import importlib
    import gb_fusion_recovery as FR
    FR._pid_is_explore = lambda pid: int(pid) == os.getpid()      # the node runs in this test process
    import gb_explore as X
    importlib.reload(X)
    X.ESTOP_CONT = os.path.join(gbs, "ESTOP"); X.ARMED_CONT = os.path.join(gbs, "ARMED")
    X.GUARD_FLAG_CONT = os.path.join(shared, "gb_guard", "mission_active")
    BUS.subs.clear(); BUS.pubs[:] = []
    sim = Sim(root, world, cam_mode)
    sim.gate.G.ESTOP = X.ESTOP_CONT
    th = threading.Thread(target=sim.loop, args=(cont,), daemon=True); th.start()
    lines = []
    import builtins
    real_print = builtins.print
    def cap(*a, **k):
        lines.append(" ".join(str(x) for x in a))
    builtins.print = cap
    import signal
    old = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)
    try:
        def killer():
            time.sleep(timeout)
            if sim.run:
                open(os.path.join(cont, "abort"), "w").write("test timeout")
        threading.Thread(target=killer, daemon=True).start()
        X.node_main(plan, False, False)
    finally:
        builtins.print = real_print
        signal.signal(signal.SIGINT, old[0]); signal.signal(signal.SIGTERM, old[1])
        sim.run = False
    res = json.load(open(os.path.join(cont, "result.json")))
    tok_left = os.path.exists(os.path.join(gbs, FR.TOKEN_NAME))
    return res, sim, lines, tok_left
