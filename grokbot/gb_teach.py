"""gb_teach.py - local object teaching for Wally.

Multi-turn: "I'm going to tell you what this object is", then a short name.
Stores a small crop and a 64-bit difference hash in SQLite. Recognition is
nearest-hash with an explicit uncertainty band. A perceptual hash matches a
similar view under similar lighting. It does not handle arbitrary viewpoints.

No large local vision model is loaded. Grok may describe an unfamiliar scene
later; that text is not allowed to start motion (see gb_wally.vision_may_move).
"""
import os, sqlite3, time

MAX_OBJECTS = int(os.environ.get("TEACH_MAX_OBJECTS", "40"))
MAX_BYTES = int(os.environ.get("TEACH_MAX_BYTES", "8192"))
MATCH_BITS = 12          # hamming <= this: confident
UNSURE_BITS = 20         # hamming <= this: say so, do not pretend

_AWAIT = "await_label"
_IDLE = "idle"


def dhash(gray):
    """gray: sequence of rows, each a sequence of numbers. Uses the top-left 9x8."""
    rows = list(gray)
    if len(rows) < 2 or len(rows[0]) < 2:
        raise ValueError("image too small")
    h = min(8, len(rows))
    w = min(9, len(rows[0]))
    bits = []
    for y in range(h):
        row = list(rows[y])
        for x in range(w - 1):
            bits.append(1 if float(row[x]) < float(row[x + 1]) else 0)
    # pad to 64 so distances stay comparable on tiny fixtures
    while len(bits) < 64:
        bits.append(0)
    return bits[:64]


def hamming(a, b):
    return sum(1 for x, y in zip(a, b) if x != y)


def _pack(bits):
    return "".join("1" if b else "0" for b in bits)


def _unpack(s):
    return [1 if c == "1" else 0 for c in s]


class Teacher(object):
    def __init__(self, path):
        self.path = path
        self.phase = _IDLE
        self.last_id = None
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self._init()

    def _conn(self):
        c = sqlite3.connect(self.path, timeout=5)
        c.row_factory = sqlite3.Row
        return c

    def _init(self):
        with self._conn() as c:
            c.execute("""CREATE TABLE IF NOT EXISTS objects(
                id INTEGER PRIMARY KEY,
                name TEXT NOT NULL,
                phash TEXT NOT NULL,
                image BLOB,
                depth BLOB,
                created REAL NOT NULL)""")

    def _prune(self, c):
        n = c.execute("SELECT count(*) FROM objects").fetchone()[0]
        extra = n - MAX_OBJECTS
        if extra > 0:
            c.execute("DELETE FROM objects WHERE id IN (SELECT id FROM objects ORDER BY created ASC LIMIT ?)", (extra,))

    def begin(self):
        self.phase = _AWAIT
        return "Okay. Tell me the name."

    def teach(self, name, gray, depth_crop=None):
        name = " ".join((name or "").strip().split())
        if not name or len(name) > 40:
            return "I didn't catch a short name. Tell me what it is."
        bits = dhash(gray)
        img = _crop_bytes(gray)
        dep = _crop_bytes(depth_crop) if depth_crop is not None else None
        with self._conn() as c:
            cur = c.execute("INSERT INTO objects(name, phash, image, depth, created) VALUES(?,?,?,?,?)",
                            (name, _pack(bits), img, dep, time.time()))
            self.last_id = cur.lastrowid
            self._prune(c)
        self.phase = _IDLE
        return "Got it, that's a %s." % name

    def recognize(self, gray):
        bits = dhash(gray)
        with self._conn() as c:
            rows = c.execute("SELECT id, name, phash FROM objects").fetchall()
        if not rows:
            return {"kind": "unknown", "say": None}
        scored = []
        for r in rows:
            d = hamming(bits, _unpack(r["phash"]))
            scored.append((d, r["name"], r["id"]))
        scored.sort()
        best, name, oid = scored[0]
        second = scored[1][0] if len(scored) > 1 else 99
        conf = 1.0 - best / 64.0
        close_names = []
        for d, n, _i in scored:
            if d <= best + 3 and n not in close_names:
                close_names.append(n)
        self.last_id = oid
        if len(close_names) >= 2 and second <= UNSURE_BITS:
            return {"kind": "uncertain", "confidence": conf, "names": close_names,
                    "say": "I'm not sure. It might be a %s." % " or a ".join(close_names[:2])}
        if best <= MATCH_BITS:
            return {"kind": "match", "confidence": conf, "name": name,
                    "say": "That looks like a %s." % name}
        if best <= UNSURE_BITS:
            return {"kind": "uncertain", "confidence": conf, "name": name,
                    "say": "I'm not sure. It might be a %s." % name}
        return {"kind": "unknown", "confidence": conf, "say": None}

    def correct(self, name):
        name = " ".join((name or "").strip().split())
        if not self.last_id or not name:
            return "I don't have a recent object to correct."
        with self._conn() as c:
            c.execute("UPDATE objects SET name=? WHERE id=?", (name, self.last_id))
        return "Got it, that's a %s." % name

    def hear(self, text, capture):
        """capture() -> (gray, depth_or_None). Returns a reply, or None if this turn is not teaching."""
        t = " ".join((text or "").lower().replace("'", "").split())
        if "going to tell you what this object is" in t or "going to tell you what this is" in t:
            return self.begin()
        if self.phase == _AWAIT:
            if not t or t in ("stop", "cancel", "never mind"):
                self.phase = _IDLE
                return "Okay, I won't save that."
            gray, depth = capture()
            label = text.strip().rstrip(".!?")
            for prefix in ("it is ", "it's ", "its ", "a ", "an ", "the ", "that's a ", "thats a "):
                if label.lower().startswith(prefix):
                    label = label[len(prefix):]
                    break
            return self.teach(label, gray, depth)
        if "no thats " in t or "no, thats " in t or "thats actually" in t or "that's actually" in (text or "").lower():
            name = text.lower()
            for token in ("no, that's a ", "no, thats a ", "no thats a ", "that's actually a ", "thats actually a ",
                          "no, that's ", "no that's "):
                if token in name:
                    name = name.split(token, 1)[1]
                    break
            return self.correct(name.strip(" .!?"))
        return None

    def stored_bytes(self):
        with self._conn() as c:
            rows = c.execute("SELECT image, depth FROM objects").fetchall()
        n = 0
        for r in rows:
            n += len(r["image"] or b"") + len(r["depth"] or b"")
        return n


def _crop_bytes(gray):
    if gray is None:
        return None
    # Bound the stored crop: 32x32 of one byte per pixel, then hard cap.
    # A flat sample list (depth crop) is stored as one row.
    rows = list(gray)[:32]
    if rows and not isinstance(rows[0], (list, tuple)):
        rows = [rows]
    raw = bytearray()
    for row in rows:
        for v in list(row)[:32]:
            try:
                b = int(max(0, min(255, float(v))))
            except (TypeError, ValueError):
                b = 0
            raw.append(b)
    return bytes(raw[:MAX_BYTES])
