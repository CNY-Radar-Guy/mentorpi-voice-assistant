"""gb_store.py - SD-card persistence for Grok Bot (SQLite WAL + synchronous=FULL, atomic file writes)."""
import hashlib, json, os, re, sqlite3, threading, time

DATA = os.path.expanduser(os.environ.get("GB_DATA", "~/grokbot/data"))
DB_PATH = os.path.join(DATA, "grokbot.db")
for sub in ("", "snapshots", "maps", "transcripts"):
    os.makedirs(os.path.join(DATA, sub), exist_ok=True)

STOP = set("""a an the is are was were be been to of and or in on at for with that this it its i me my you your
we our he she they them what who where when how do does did about tell know remember forget please can could
would should will just than then there here have has had from as by not no yes""".split())

SCHEMA = """
CREATE TABLE IF NOT EXISTS facts(id INTEGER PRIMARY KEY, text TEXT NOT NULL, source TEXT NOT NULL DEFAULT 'voice',
  created REAL NOT NULL, updated REAL NOT NULL);
CREATE TABLE IF NOT EXISTS conversation(id INTEGER PRIMARY KEY, ts REAL NOT NULL, role TEXT NOT NULL,
  text TEXT NOT NULL, category TEXT, summarized INTEGER NOT NULL DEFAULT 0);
CREATE TABLE IF NOT EXISTS summary(id INTEGER PRIMARY KEY CHECK(id=1), text TEXT, updated REAL, upto_id INTEGER);
CREATE TABLE IF NOT EXISTS labels(id INTEGER PRIMARY KEY, name TEXT NOT NULL, embedding BLOB, emb_dim INTEGER,
  map_x REAL, map_y REAL, map_theta REAL, map_id TEXT, frame TEXT, snapshot TEXT, note TEXT,
  created REAL NOT NULL, last_seen REAL NOT NULL, count INTEGER NOT NULL DEFAULT 1);
CREATE TABLE IF NOT EXISTS maps(id INTEGER PRIMARY KEY, name TEXT NOT NULL, yaml_path TEXT, pgm_path TEXT,
  resolution REAL, width INTEGER, height INTEGER, origin TEXT, created REAL NOT NULL);
CREATE TABLE IF NOT EXISTS usage(id INTEGER PRIMARY KEY, ts REAL NOT NULL, day TEXT NOT NULL, backend TEXT NOT NULL,
  model TEXT, category TEXT NOT NULL, prompt_tokens INTEGER, completion_tokens INTEGER, total_tokens INTEGER,
  cached_tokens INTEGER);
CREATE INDEX IF NOT EXISTS usage_day ON usage(day, backend, category);
CREATE TABLE IF NOT EXISTS events(id INTEGER PRIMARY KEY, ts REAL NOT NULL, kind TEXT NOT NULL, detail TEXT);
CREATE TABLE IF NOT EXISTS meta(k TEXT PRIMARY KEY, v TEXT);
"""
FTS = """
CREATE VIRTUAL TABLE IF NOT EXISTS facts_fts USING fts5(text, content='facts', content_rowid='id');
CREATE TRIGGER IF NOT EXISTS facts_ai AFTER INSERT ON facts BEGIN
  INSERT INTO facts_fts(rowid, text) VALUES (new.id, new.text); END;
CREATE TRIGGER IF NOT EXISTS facts_ad AFTER DELETE ON facts BEGIN
  INSERT INTO facts_fts(facts_fts, rowid, text) VALUES('delete', old.id, old.text); END;
CREATE TRIGGER IF NOT EXISTS facts_au AFTER UPDATE ON facts BEGIN
  INSERT INTO facts_fts(facts_fts, rowid, text) VALUES('delete', old.id, old.text);
  INSERT INTO facts_fts(rowid, text) VALUES (new.id, new.text); END;
"""

def fsync_dir(path):
    try:
        fd = os.open(path, os.O_DIRECTORY); os.fsync(fd); os.close(fd)
    except OSError:
        pass

def atomic_write(path, data):
    """write-then-rename with fsync of file and directory"""
    if isinstance(data, str):
        data = data.encode()
    tmp = f"{path}.tmp{os.getpid()}"
    with open(tmp, "wb") as f:
        f.write(data); f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path); fsync_dir(os.path.dirname(path) or ".")

def words(text):
    return [w for w in re.findall(r"[a-z0-9']+", text.lower()) if len(w) > 2 and w not in STOP]

class Store:
    def __init__(self, path=DB_PATH):
        self.lock = threading.RLock()
        self.db = sqlite3.connect(path, check_same_thread=False, isolation_level=None, timeout=30)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")     # fsync the WAL on every commit
        self.db.executescript(SCHEMA)
        try:
            self.db.executescript(FTS); self.fts = True
            if self.db.execute("SELECT count(*) FROM facts").fetchone()[0] and \
               not self.db.execute("SELECT count(*) FROM facts_fts").fetchone()[0]:
                self.db.execute("INSERT INTO facts_fts(facts_fts) VALUES('rebuild')")
        except sqlite3.OperationalError:
            self.fts = False

    def tx(self, sql, args=()):
        with self.lock:
            self.db.execute("BEGIN IMMEDIATE")
            try:
                cur = self.db.execute(sql, args); self.db.execute("COMMIT"); return cur
            except Exception:
                self.db.execute("ROLLBACK"); raise

    def q(self, sql, args=()):
        with self.lock:
            return self.db.execute(sql, args).fetchall()

    def checkpoint(self):
        with self.lock:
            self.db.execute("PRAGMA wal_checkpoint(FULL)")
        os.sync()

    # ---------------- facts
    def add_fact(self, text, source="voice"):
        now = time.time()
        dup = self.q("SELECT id FROM facts WHERE lower(text)=lower(?)", (text,))
        if dup:
            self.tx("UPDATE facts SET updated=? WHERE id=?", (now, dup[0]["id"])); return dup[0]["id"]
        return self.tx("INSERT INTO facts(text,source,created,updated) VALUES(?,?,?,?)", (text, source, now, now)).lastrowid

    def search_facts(self, query, k=3):
        ws = words(query)
        if not ws:
            return []
        if self.fts:
            m = " OR ".join(f'"{w}"' for w in ws)
            try:
                return [r["text"] for r in self.q(
                    "SELECT f.text FROM facts_fts JOIN facts f ON f.id=facts_fts.rowid "
                    "WHERE facts_fts MATCH ? ORDER BY bm25(facts_fts) LIMIT ?", (m, k))]
            except sqlite3.OperationalError:
                pass
        rows = self.q("SELECT text FROM facts")
        scored = sorted(((sum(w in r["text"].lower() for w in ws), r["text"]) for r in rows), reverse=True)
        return [t for s, t in scored[:k] if s > 0]

    def forget(self, query):
        if query.strip().lower() in ("everything", "all", "all facts", "everything you know"):
            return None  # caller must confirm
        ws = words(query)
        if not ws:
            return []
        rows = self.q("SELECT id,text FROM facts WHERE source!='seed'")
        hits = [r for r in rows if all(w in r["text"].lower() for w in ws)]
        for r in hits:
            self.tx("DELETE FROM facts WHERE id=?", (r["id"],))
        return [r["text"] for r in hits]

    def forget_all(self):
        n = self.q("SELECT count(*) c FROM facts WHERE source!='seed'")[0]["c"]
        self.tx("DELETE FROM facts WHERE source!='seed'"); return n

    def fact_count(self):
        return self.q("SELECT count(*) c FROM facts")[0]["c"]

    # ---------------- seed knowledge file
    def import_seed(self, path):
        """(Re)import a markdown knowledge file as facts with source='seed'. Returns count or None if unchanged."""
        if not os.path.exists(path):
            return None
        raw = open(path, encoding="utf-8", errors="replace").read()
        sha = hashlib.sha256(raw.encode()).hexdigest()
        if self.get_meta("seed_sha") == sha:
            return None
        chunks, para = [], []
        body = re.sub(r"<!--.*?-->", "", raw, flags=re.S)          # drop (multi-line) HTML comments
        for line in body.splitlines():
            s = line.strip()
            if s.startswith("<!--") or s.startswith("#") or not s:
                if para: chunks.append(" ".join(para)); para = []
                continue
            if re.match(r"^([-*+]|\d+\.)\s+", s):
                if para: chunks.append(" ".join(para)); para = []
                chunks.append(re.sub(r"^([-*+]|\d+\.)\s+", "", s))
            else:
                para.append(s)
        if para: chunks.append(" ".join(para))
        chunks = [c[:600] for c in chunks if len(c) > 3 and "PLACEHOLDER" not in c.upper()]
        with self.lock:
            self.db.execute("BEGIN IMMEDIATE")
            self.db.execute("DELETE FROM facts WHERE source='seed'")
            now = time.time()
            for c in chunks:
                self.db.execute("INSERT INTO facts(text,source,created,updated) VALUES(?,?,?,?)", (c, "seed", now, now))
            self.db.execute("INSERT OR REPLACE INTO meta(k,v) VALUES('seed_sha',?)", (sha,))
            self.db.execute("COMMIT")
        return len(chunks)

    # ---------------- conversation + rolling summary
    def add_turn(self, role, text, category):
        self.tx("INSERT INTO conversation(ts,role,text,category) VALUES(?,?,?,?)", (time.time(), role, text, category))
        self.tx("DELETE FROM conversation WHERE id < (SELECT max(id) - 2000 FROM conversation)")

    def last_turns(self, n_msgs=8):
        # only real LLM exchanges: canned local replies (battery, usage, "still setting up"...) would be parroted
        rows = self.q("SELECT role,text FROM conversation WHERE category IN ('conversation','smalltalk','vision') "
                      "ORDER BY id DESC LIMIT ?", (n_msgs,))
        return [{"role": r["role"], "content": r["text"]} for r in reversed(rows)]

    def unsummarized_old(self, keep=8):
        return self.q("SELECT id,role,text FROM conversation WHERE summarized=0 AND id <= "
                      "(SELECT coalesce(max(id),0) - ? FROM conversation) ORDER BY id", (keep,))

    def get_summary(self):
        r = self.q("SELECT text FROM summary WHERE id=1")
        return r[0]["text"] if r else ""

    def set_summary(self, text, upto_id):
        self.tx("INSERT OR REPLACE INTO summary(id,text,updated,upto_id) VALUES(1,?,?,?)", (text, time.time(), upto_id))
        self.tx("UPDATE conversation SET summarized=1 WHERE id<=?", (upto_id,))

    # ---------------- labels
    def add_label(self, name, emb=None, pose=None, snapshot=None, note=""):
        now = time.time(); pose = pose or {}
        blob = emb.astype("float16").tobytes() if emb is not None else None
        lid = self.tx("INSERT INTO labels(name,embedding,emb_dim,map_x,map_y,map_theta,map_id,frame,snapshot,note,"
                      "created,last_seen,count) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,1)",
                      (name, blob, None if emb is None else int(emb.shape[0]), pose.get("x"), pose.get("y"),
                       pose.get("theta"), pose.get("map_id"), pose.get("frame"), snapshot, note, now, now)).lastrowid
        self.write_labels_tsv(); return lid

    def touch_label(self, lid):
        self.tx("UPDATE labels SET last_seen=?, count=count+1 WHERE id=?", (time.time(), lid))

    def labels(self):
        return self.q("SELECT * FROM labels ORDER BY id")

    def find_labels(self, name):
        ws = words(name) or [name.lower()]
        return [r for r in self.labels() if all(w in r["name"].lower() for w in ws)]

    def write_labels_tsv(self):
        lines = ["# name\tx,y\tnote  (map frame; auto-generated, do not edit)"]
        for r in self.labels():
            xy = f"{r['map_x']:.2f},{r['map_y']:.2f}" if r["map_x"] is not None else "?,?"
            note = f"{r['frame'] or '?'} {r['map_id'] or ''} seen{r['count']} {r['note'] or ''}".strip()
            lines.append(f"{r['name']}\t{xy}\t{note}")
        atomic_write(os.path.join(DATA, "labels.tsv"), "\n".join(lines) + "\n")

    # ---------------- maps
    def add_map(self, name, yaml_path, pgm_path, res, w, h, origin):
        return self.tx("INSERT INTO maps(name,yaml_path,pgm_path,resolution,width,height,origin,created) VALUES(?,?,?,?,?,?,?,?)",
                       (name, yaml_path, pgm_path, res, w, h, json.dumps(origin), time.time())).lastrowid

    def current_map_id(self):
        r = self.q("SELECT name FROM maps ORDER BY id DESC LIMIT 1")
        return r[0]["name"] if r else None

    # ---------------- usage
    def log_usage(self, backend, model, category, usage):
        usage = usage or {}
        p = usage.get("prompt_tokens", usage.get("input_tokens")) or 0
        c = usage.get("completion_tokens", usage.get("output_tokens")) or 0
        t = usage.get("total_tokens") or (p + c)
        det = usage.get("prompt_tokens_details") or usage.get("input_tokens_details") or {}
        cached = det.get("cached_tokens") or 0
        self.tx("INSERT INTO usage(ts,day,backend,model,category,prompt_tokens,completion_tokens,total_tokens,cached_tokens)"
                " VALUES(?,?,?,?,?,?,?,?,?)", (time.time(), time.strftime("%Y-%m-%d"), backend, model, category, p, c, t, cached))
        return t

    def usage_sum(self, since_ts=None, day=None, backend=None):
        sql = "SELECT category, coalesce(sum(total_tokens),0) t, count(*) n FROM usage WHERE 1=1"
        args = []
        if day: sql += " AND day=?"; args.append(day)
        if since_ts: sql += " AND ts>=?"; args.append(since_ts)
        if backend: sql += " AND backend=?"; args.append(backend)
        sql += " GROUP BY category"
        return {r["category"]: (r["t"], r["n"]) for r in self.q(sql, args)}

    def xai_today(self, category):
        """vision = vision calls; conversation = everything else (conversation, smalltalk, summary)"""
        cond = "category='vision'" if category == "vision" else "category!='vision'"
        r = self.q(f"SELECT coalesce(sum(total_tokens),0) t FROM usage WHERE day=? AND backend='xai' AND {cond}",
                   (time.strftime("%Y-%m-%d"),))
        return r[0]["t"]

    # ---------------- misc
    def event(self, kind, detail=""):
        self.tx("INSERT INTO events(ts,kind,detail) VALUES(?,?,?)", (time.time(), kind, detail))

    def get_meta(self, k):
        r = self.q("SELECT v FROM meta WHERE k=?", (k,))
        return r[0]["v"] if r else None

def append_transcript(rec):
    """one JSON line per exchange in data/transcripts/YYYY-MM-DD.jsonl, fsynced"""
    path = os.path.join(DATA, "transcripts", time.strftime("%Y-%m-%d") + ".jsonl")
    line = json.dumps(rec, ensure_ascii=False) + "\n"
    with open(path, "a", encoding="utf-8") as f:
        f.write(line); f.flush(); os.fsync(f.fileno())
