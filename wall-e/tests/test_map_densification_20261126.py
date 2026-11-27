"""Unit tests for continuous master-map densification in gb_mapping.py (2026-11-26).

Zero motion: these tests mock docker/ROS and never start slam_toolbox or arm Wall-E.
"""
import os
import sys
import time
import types
from pathlib import Path

sys.path.insert(0, "/home/pi/grokbot/releases/e48d3d8")
import gb_mapping


def _ok_proc(stdout="", stderr="", rc=0):
    return types.SimpleNamespace(returncode=rc, stdout=stdout, stderr=stderr)


def _write_grid(dirpath: Path, stem: str, w, h, pixels, res=0.05, origin=(-1.0, -2.0, 0.0), mtime=None):
    dirpath.mkdir(parents=True, exist_ok=True)
    pgm = gb_mapping._write_pgm_bytes(w, h, pixels)
    yaml = gb_mapping._format_map_yaml(stem + ".pgm", res, list(origin))
    (dirpath / (stem + ".pgm")).write_bytes(pgm)
    (dirpath / (stem + ".yaml")).write_text(yaml)
    if mtime is not None:
        for ext in (".yaml", ".pgm"):
            os.utime(dirpath / (stem + ext), (mtime, mtime))
    return dirpath / (stem + ".yaml"), dirpath / (stem + ".pgm")


def _write_graph(dirpath: Path, stem: str, payload=b"graph"):
    dirpath.mkdir(parents=True, exist_ok=True)
    (dirpath / (stem + ".posegraph")).write_bytes(payload + b"-posegraph")
    (dirpath / (stem + ".data")).write_bytes(payload + b"-data")


def _patch_paths(monkeypatch, tmp_path, log=True):
    maps_dir = tmp_path / "maps"
    host_dir = tmp_path / "host"
    (host_dir / "maps").mkdir(parents=True)
    maps_dir.mkdir(parents=True)
    monkeypatch.setattr(gb_mapping, "MAPS", str(maps_dir))
    monkeypatch.setattr(gb_mapping, "HOST_DIR", str(host_dir))
    monkeypatch.setattr(gb_mapping, "CONT_DIR", "/home/ubuntu/shared/grokbot")
    monkeypatch.setattr(gb_mapping, "PIDF_HOST", str(host_dir / "mapping.pid"))
    if log:
        monkeypatch.setattr(gb_mapping, "LOG", str(tmp_path / "mapping.log"))
    monkeypatch.setattr(gb_mapping, "_log", lambda msg: None)
    monkeypatch.setattr(gb_mapping, "_setup", lambda: (maps_dir.mkdir(parents=True, exist_ok=True),
                                                       (host_dir / "maps").mkdir(parents=True, exist_ok=True)))
    return maps_dir, host_dir


# ---------------------------------------------------------------------------
# resolve_save_name
# ---------------------------------------------------------------------------
def test_resolve_auto_and_empty_names_go_to_master():
    assert gb_mapping.MASTER_MAP_NAME == "master"
    assert gb_mapping.MATCH_START_AT_FIRST_NODE == 1
    assert gb_mapping.resolve_save_name(None) == ("master", False)
    assert gb_mapping.resolve_save_name("") == ("master", False)
    assert gb_mapping.resolve_save_name("explore_20261126_200700") == ("master", False)
    assert gb_mapping.resolve_save_name("map_20261126_200700") == ("master", False)
    assert gb_mapping.resolve_save_name("master") == ("master", False)


def test_resolve_custom_name_is_preserved():
    assert gb_mapping.resolve_save_name("living_room") == ("living_room", True)
    assert gb_mapping.resolve_save_name("Living Room") == ("living_room", True)


# ---------------------------------------------------------------------------
# start() deserialize
# ---------------------------------------------------------------------------
def test_start_deserializes_existing_master(tmp_path, monkeypatch):
    maps_dir, host_dir = _patch_paths(monkeypatch, tmp_path)
    _write_graph(maps_dir, "master", b"PRIOR")
    calls = []

    monkeypatch.setattr(gb_mapping, "running", lambda: None)
    monkeypatch.setattr(gb_mapping.time, "sleep", lambda s: None)
    monkeypatch.setattr(gb_mapping.subprocess, "Popen", lambda *a, **k: types.SimpleNamespace(pid=1))
    monkeypatch.setattr(gb_mapping.gb_store, "Store",
                        lambda *a, **k: types.SimpleNamespace(event=lambda *aa, **kk: None))
    # mapping_ros.log is opened via gb_store.DATA; point DATA's log at tmp by opening a dummy.
    monkeypatch.setattr(gb_mapping.gb_store, "DATA", str(tmp_path))
    (tmp_path / "mapping_ros.log").touch()

    def fake_dx(args, timeout=20, user="1000"):
        calls.append(("dx", list(args)))
        if args[:2] == ["pgrep", "-f"] and "sync_slam_toolbox_node" in args[2]:
            return _ok_proc("123\n")
        return _ok_proc()

    def fake_ros2(args, timeout=20):
        calls.append(("ros2", list(args)))
        if args[:2] == ["ros2", "service"] and "list" in args:
            return _ok_proc("/slam_toolbox/deserialize_map\n/slam_toolbox/serialize_map\n")
        if "service" in args and "call" in args and "/slam_toolbox/deserialize_map" in args:
            return _ok_proc("response: {}\n")
        return _ok_proc()

    monkeypatch.setattr(gb_mapping, "_dx", fake_dx)
    monkeypatch.setattr(gb_mapping, "_ros2_call", fake_ros2)

    ok, msg = gb_mapping.start()
    assert ok is True
    deser = [c for c in calls if c[0] == "ros2" and "/slam_toolbox/deserialize_map" in c[1]]
    assert deser, f"expected deserialize_map call, got {calls}"
    argv = deser[0][1]
    assert "slam_toolbox/srv/DeserializePoseGraph" in argv
    payload = argv[-1]
    assert "/home/ubuntu/shared/grokbot/maps/master" in payload
    assert "match_type: 1" in payload
    # Staged from MAPS into the shared folder the container can see
    assert (host_dir / "maps" / "master.posegraph").exists()
    assert (host_dir / "maps" / "master.data").exists()
    assert "existing master" in msg.lower() or "densify" in msg.lower()


def test_start_fresh_when_no_master_graph(tmp_path, monkeypatch):
    maps_dir, host_dir = _patch_paths(monkeypatch, tmp_path)
    calls = []
    monkeypatch.setattr(gb_mapping, "running", lambda: None)
    monkeypatch.setattr(gb_mapping.time, "sleep", lambda s: None)
    monkeypatch.setattr(gb_mapping.subprocess, "Popen", lambda *a, **k: types.SimpleNamespace(pid=1))
    monkeypatch.setattr(gb_mapping.gb_store, "Store",
                        lambda *a, **k: types.SimpleNamespace(event=lambda *aa, **kk: None))
    monkeypatch.setattr(gb_mapping.gb_store, "DATA", str(tmp_path))
    (tmp_path / "mapping_ros.log").touch()

    def fake_dx(args, timeout=20, user="1000"):
        if args[:2] == ["pgrep", "-f"] and "sync_slam_toolbox_node" in args[2]:
            return _ok_proc("9\n")
        return _ok_proc()

    def fake_ros2(args, timeout=20):
        calls.append(list(args))
        return _ok_proc()

    monkeypatch.setattr(gb_mapping, "_dx", fake_dx)
    monkeypatch.setattr(gb_mapping, "_ros2_call", fake_ros2)

    ok, msg = gb_mapping.start()
    assert ok is True
    assert not any("/slam_toolbox/deserialize_map" in a for c in calls for a in c)
    assert "fresh master" in msg.lower()


def test_start_deserialize_failure_continues_fresh(tmp_path, monkeypatch):
    maps_dir, host_dir = _patch_paths(monkeypatch, tmp_path)
    _write_graph(maps_dir, "master", b"CORRUPT")
    monkeypatch.setattr(gb_mapping, "running", lambda: None)
    monkeypatch.setattr(gb_mapping.time, "sleep", lambda s: None)
    monkeypatch.setattr(gb_mapping.subprocess, "Popen", lambda *a, **k: types.SimpleNamespace(pid=1))
    monkeypatch.setattr(gb_mapping.gb_store, "Store",
                        lambda *a, **k: types.SimpleNamespace(event=lambda *aa, **kk: None))
    monkeypatch.setattr(gb_mapping.gb_store, "DATA", str(tmp_path))
    (tmp_path / "mapping_ros.log").touch()

    def fake_dx(args, timeout=20, user="1000"):
        if args[:2] == ["pgrep", "-f"] and "sync_slam_toolbox_node" in args[2]:
            return _ok_proc("9\n")
        return _ok_proc()

    def fake_ros2(args, timeout=20):
        if args[:2] == ["ros2", "service"] and "list" in args:
            return _ok_proc("/slam_toolbox/deserialize_map\n")
        if "/slam_toolbox/deserialize_map" in args:
            return _ok_proc(stdout="", stderr="failed to deserialize (corrupt)", rc=1)
        return _ok_proc()

    monkeypatch.setattr(gb_mapping, "_dx", fake_dx)
    monkeypatch.setattr(gb_mapping, "_ros2_call", fake_ros2)

    ok, msg = gb_mapping.start()
    assert ok is True
    assert "fresh" in msg.lower()


# ---------------------------------------------------------------------------
# occupancy merge
# ---------------------------------------------------------------------------
def test_merge_preserves_known_cells_when_new_is_unknown(tmp_path):
    U, O, F = gb_mapping.PGM_UNKNOWN, gb_mapping.PGM_OCCUPIED, gb_mapping.PGM_FREE
    # 2x2 old: occupied, free / unknown, occupied
    old = [O, F, U, O]
    # 2x2 new: unknown, occupied / free, unknown  — same origin/res
    new = [U, O, F, U]
    _write_grid(tmp_path, "old", 2, 2, old, origin=(0.0, 0.0, 0.0))
    _write_grid(tmp_path, "new", 2, 2, new, origin=(0.0, 0.0, 0.0))
    result = gb_mapping.merge_occupancy_maps(
        tmp_path / "old.yaml", tmp_path / "old.pgm",
        tmp_path / "new.yaml", tmp_path / "new.pgm",
        image_name="master.pgm",
    )
    assert result["merged"] is True
    pixels = result["pgm"].split(b"\n", 3)[-1]
    # row-major, top row first: [O, F, U, O] merged with [U, O, F, U]
    # (0,0 top-left in PGM): old O + new U -> O
    # (1,0): old F + new O -> O (occupied wins)
    # (0,1): old U + new F -> F
    # (1,1): old O + new U -> O
    assert list(pixels) == [O, O, F, O]
    assert "image: master.pgm" in result["yaml"]


def test_merge_expands_canvas_when_origins_differ(tmp_path):
    U, O, F = gb_mapping.PGM_UNKNOWN, gb_mapping.PGM_OCCUPIED, gb_mapping.PGM_FREE
    # old 1x1 occupied at origin (0, 0); new 1x1 free at origin (0.05, 0) — one cell to the right
    _write_grid(tmp_path, "old", 1, 1, [O], res=0.05, origin=(0.0, 0.0, 0.0))
    _write_grid(tmp_path, "new", 1, 1, [F], res=0.05, origin=(0.05, 0.0, 0.0))
    result = gb_mapping.merge_occupancy_maps(
        tmp_path / "old.yaml", tmp_path / "old.pgm",
        tmp_path / "new.yaml", tmp_path / "new.pgm",
        image_name="master.pgm",
    )
    assert result["merged"] is True
    assert result["width"] == 2
    assert result["height"] == 1
    pixels = list(result["pgm"].split(b"\n", 3)[-1])
    assert pixels == [O, F]


# ---------------------------------------------------------------------------
# primary save()
# ---------------------------------------------------------------------------
def _stub_stop(monkeypatch):
    monkeypatch.setattr(gb_mapping, "stop", lambda: (True, "Mapping stopped."))


def test_primary_save_auto_name_writes_master_and_backup(tmp_path, monkeypatch):
    maps_dir, host_dir = _patch_paths(monkeypatch, tmp_path)
    # Prior master that must be rolled back before overwrite
    old_pixels = [gb_mapping.PGM_OCCUPIED] * 4
    _write_grid(maps_dir, "master", 2, 2, old_pixels, origin=(-1.0, -2.0, 0.0), mtime=1700000000)
    _write_graph(maps_dir, "master", b"OLDGRAPH")
    # Extra stale auto maps + extra backups to confirm MAP_MAX_KEEP rotation
    base = 1700000100
    for i, stem in enumerate([
        "explore_20261124_062756",
        "explore_20261125_124809",
        "explore_20261125_135753",
        "explore_20261126_194352",
        "explore_20261126_200700",
        "explore_20261126_201000",
    ]):
        _write_grid(maps_dir, stem, 2, 2, old_pixels, mtime=base + i)
    for i in range(6):
        bak = f"master_bak_2026112{i}_120000"
        _write_grid(maps_dir, bak, 2, 2, old_pixels, mtime=base + 100 + i)
        _write_graph(maps_dir, bak, b"BAK")

    new_pixels = [gb_mapping.PGM_FREE] * 9
    new_yaml = gb_mapping._format_map_yaml("master.pgm", 0.05, [-3.0, -4.0, 0.0])
    new_pgm = gb_mapping._write_pgm_bytes(3, 3, new_pixels)

    monkeypatch.setattr(gb_mapping, "running", lambda: "42")
    _stub_stop(monkeypatch)
    monkeypatch.setattr(gb_mapping.time, "sleep", lambda s: None)

    def fake_dx(args, timeout=20, user="1000"):
        # map_saver_cli
        if "map_saver_cli" in args:
            (host_dir / "maps" / "master.yaml").write_text(
                "image: /home/ubuntu/shared/grokbot/maps/master.pgm\nmode: trinary\n"
                "resolution: 0.05\norigin: [-3.0, -4.0, 0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
            (host_dir / "maps" / "master.pgm").write_bytes(new_pgm)
            return _ok_proc("Map saved\n")
        return _ok_proc()

    def fake_ros2(args, timeout=20):
        if args[:2] == ["ros2", "service"] and "list" in args:
            return _ok_proc("/slam_toolbox/serialize_map\n")
        if "/slam_toolbox/serialize_map" in args:
            assert "slam_toolbox/srv/SerializePoseGraph" in args
            payload = args[-1]
            assert "/home/ubuntu/shared/grokbot/maps/master" in payload
            _write_graph(host_dir / "maps", "master", b"NEWGRAPH")
            return _ok_proc("result: 0\n")
        return _ok_proc()

    monkeypatch.setattr(gb_mapping, "_dx", fake_dx)
    monkeypatch.setattr(gb_mapping, "_ros2_call", fake_ros2)

    ok, msg, info = gb_mapping.save(name="explore_20261126_211500", stop_after=True)
    assert ok is True
    assert info["name"] == "master"
    assert info["custom"] is False
    assert info["serialized"] is True
    assert info["fallback"] is False
    assert info["width"] == 3 and info["height"] == 3
    assert (maps_dir / "master.yaml").exists()
    assert (maps_dir / "master.pgm").exists()
    assert "image: master.pgm" in (maps_dir / "master.yaml").read_text()
    assert "image: " in (maps_dir / "master.yaml").read_text()
    assert not (maps_dir / "master.yaml").read_text().split("image:")[1].strip().startswith("/")
    assert (maps_dir / "master.posegraph").read_bytes().startswith(b"NEWGRAPH")
    assert (maps_dir / "master.data").read_bytes().startswith(b"NEWGRAPH")
    # Rollback of the prior master exists
    assert info["backup"] and info["backup"].startswith("master_bak_")
    assert (maps_dir / (info["backup"] + ".yaml")).exists()
    assert (maps_dir / (info["backup"] + ".posegraph")).read_bytes().startswith(b"OLDGRAPH")
    # MAP_MAX_KEEP bounds both leftover auto maps and master backups
    auto = sorted(p.stem for p in maps_dir.glob("explore_*.yaml"))
    assert len(auto) <= gb_mapping.MAP_MAX_KEEP
    baks = gb_mapping.list_master_backups()
    assert len(baks) <= gb_mapping.MAP_MAX_KEEP
    assert "master" in msg.lower()
    assert "densified" in msg.lower()
    # No new disconnected explore_* file for this save
    assert not (maps_dir / "explore_20261126_211500.yaml").exists()


def test_primary_save_enforces_map_max_keep_on_backups(tmp_path, monkeypatch):
    maps_dir, host_dir = _patch_paths(monkeypatch, tmp_path)
    _write_grid(maps_dir, "master", 1, 1, [0], mtime=1700000000)
    for i in range(8):
        bak = f"master_bak_20261120_0{i}0000"
        _write_grid(maps_dir, bak, 1, 1, [0], mtime=1700000000 + i)
    monkeypatch.setattr(gb_mapping, "running", lambda: "1")
    _stub_stop(monkeypatch)
    monkeypatch.setattr(gb_mapping.time, "sleep", lambda s: None)

    def fake_dx(args, timeout=20, user="1000"):
        if "map_saver_cli" in args:
            (host_dir / "maps" / "master.yaml").write_text(
                "image: master.pgm\nmode: trinary\nresolution: 0.05\n"
                "origin: [0.0, 0.0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
            (host_dir / "maps" / "master.pgm").write_bytes(gb_mapping._write_pgm_bytes(1, 1, [254]))
            return _ok_proc()
        return _ok_proc()

    def fake_ros2(args, timeout=20):
        if "list" in args:
            return _ok_proc("/slam_toolbox/serialize_map\n")
        if "/slam_toolbox/serialize_map" in args:
            _write_graph(host_dir / "maps", "master", b"G")
            return _ok_proc("result: 0\n")
        return _ok_proc()

    monkeypatch.setattr(gb_mapping, "_dx", fake_dx)
    monkeypatch.setattr(gb_mapping, "_ros2_call", fake_ros2)
    ok, msg, info = gb_mapping.save(name=None, stop_after=False)
    assert ok is True
    assert len(gb_mapping.list_master_backups()) == gb_mapping.MAP_MAX_KEEP
    assert (maps_dir / "master.yaml").exists()


# ---------------------------------------------------------------------------
# fallback rosbridge save()
# ---------------------------------------------------------------------------
def test_fallback_save_updates_master_merges_and_serializes(tmp_path, monkeypatch):
    maps_dir, host_dir = _patch_paths(monkeypatch, tmp_path)
    U, O, F = gb_mapping.PGM_UNKNOWN, gb_mapping.PGM_OCCUPIED, gb_mapping.PGM_FREE
    # Prior master 2x2
    _write_grid(maps_dir, "master", 2, 2, [O, F, U, O], origin=(0.0, 0.0, 0.0), mtime=1700000000)
    _write_graph(maps_dir, "master", b"OLD")

    monkeypatch.setattr(gb_mapping, "running", lambda: "7")
    _stub_stop(monkeypatch)
    monkeypatch.setattr(gb_mapping.time, "sleep", lambda s: None)

    def fake_dx(args, timeout=20, user="1000"):
        # map_saver_cli fails so we take the rosbridge path
        if "map_saver_cli" in args:
            return _ok_proc(stdout="", stderr="timeout waiting for map", rc=1)
        return _ok_proc()

    ser_calls = []

    def fake_ros2(args, timeout=20):
        ser_calls.append(list(args))
        if args[:2] == ["ros2", "service"] and "list" in args:
            return _ok_proc("/slam_toolbox/serialize_map\n")
        if "/slam_toolbox/serialize_map" in args:
            _write_graph(host_dir / "maps", "master", b"NEW")
            return _ok_proc("result: 0\n")
        return _ok_proc()

    def fake_fallback(name, maps_dir_):
        assert name == "master"
        # gb_robot.save_map writes a timestamped stem; include some unknown cells.
        stem = "20261126_211500_master"
        _write_grid(Path(maps_dir_), stem, 2, 2, [U, O, F, U], origin=(0.0, 0.0, 0.0))
        return (str(Path(maps_dir_) / stem), 0.05, 2, 2, [0.0, 0.0, 0.0])

    monkeypatch.setattr(gb_mapping, "_dx", fake_dx)
    monkeypatch.setattr(gb_mapping, "_ros2_call", fake_ros2)
    monkeypatch.setattr(gb_mapping, "_fallback_save_map", fake_fallback)

    # serialize succeeds -> use the fresh /map (no occupancy merge required)
    ok, msg, info = gb_mapping.save(name="explore_20261126_211500", stop_after=False)
    assert ok is True
    assert info["fallback"] is True
    assert info["name"] == "master"
    assert info["serialized"] is True
    assert any("/slam_toolbox/serialize_map" in a for c in ser_calls for a in c)
    assert (maps_dir / "master.yaml").exists()
    assert "image: master.pgm" in (maps_dir / "master.yaml").read_text()
    assert (maps_dir / "master.posegraph").read_bytes().startswith(b"NEW")
    # Throwaway timestamped fallback file must not remain
    leftover = list(maps_dir.glob("20261126_211500_master.*"))
    assert leftover == []
    assert any(p.name.startswith("master_bak_") and p.suffix == ".yaml" for p in maps_dir.iterdir())


def test_fallback_save_merges_occupancy_when_serialize_unavailable(tmp_path, monkeypatch):
    maps_dir, host_dir = _patch_paths(monkeypatch, tmp_path)
    U, O, F = gb_mapping.PGM_UNKNOWN, gb_mapping.PGM_OCCUPIED, gb_mapping.PGM_FREE
    _write_grid(maps_dir, "master", 2, 2, [O, F, U, O], origin=(0.0, 0.0, 0.0), mtime=1700000000)

    monkeypatch.setattr(gb_mapping, "running", lambda: "7")
    _stub_stop(monkeypatch)
    monkeypatch.setattr(gb_mapping.time, "sleep", lambda s: None)

    def fake_dx(args, timeout=20, user="1000"):
        if "map_saver_cli" in args:
            return _ok_proc(stderr="no map", rc=1)
        return _ok_proc()

    def fake_ros2(args, timeout=20):
        # serialize service missing / call fails
        if "list" in args:
            return _ok_proc("")
        return _ok_proc(stderr="service not available", rc=1)

    def fake_fallback(name, maps_dir_):
        stem = "20261126_220000_master"
        _write_grid(Path(maps_dir_), stem, 2, 2, [U, O, F, U], origin=(0.0, 0.0, 0.0))
        return (str(Path(maps_dir_) / stem), 0.05, 2, 2, [0.0, 0.0, 0.0])

    monkeypatch.setattr(gb_mapping, "_dx", fake_dx)
    monkeypatch.setattr(gb_mapping, "_ros2_call", fake_ros2)
    monkeypatch.setattr(gb_mapping, "_fallback_save_map", fake_fallback)

    ok, msg, info = gb_mapping.save(name=None, stop_after=False)
    assert ok is True
    assert info["fallback"] is True
    assert info["serialized"] is False
    assert info["merged"] is True
    pixels = list((maps_dir / "master.pgm").read_bytes().split(b"\n", 3)[-1][:4])
    # Same merge as test_merge_preserves_known_cells_when_new_is_unknown
    assert pixels == [O, O, F, O]
    assert "merged" in msg.lower()
    assert "image: master.pgm" in (maps_dir / "master.yaml").read_text()


# ---------------------------------------------------------------------------
# custom-named maps
# ---------------------------------------------------------------------------
def test_custom_named_save_is_preserved_and_not_purged(tmp_path, monkeypatch):
    maps_dir, host_dir = _patch_paths(monkeypatch, tmp_path)
    _write_grid(maps_dir, "living_room", 2, 2, [0, 0, 0, 0], mtime=1600000000)
    # Prior master should be left alone by a custom save
    _write_grid(maps_dir, "master", 1, 1, [0], mtime=1600000001)
    _write_graph(maps_dir, "master", b"MASTER")

    monkeypatch.setattr(gb_mapping, "running", lambda: "3")
    _stub_stop(monkeypatch)
    monkeypatch.setattr(gb_mapping.time, "sleep", lambda s: None)

    def fake_dx(args, timeout=20, user="1000"):
        if "map_saver_cli" in args:
            assert any(a.endswith("/maps/living_room") or a.endswith("living_room") or "-f" in str(args) for a in args)
            (host_dir / "maps" / "living_room.yaml").write_text(
                "image: /abs/living_room.pgm\nmode: trinary\nresolution: 0.05\n"
                "origin: [1.0, 2.0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
            (host_dir / "maps" / "living_room.pgm").write_bytes(
                gb_mapping._write_pgm_bytes(2, 2, [254, 254, 254, 254]))
            return _ok_proc()
        return _ok_proc()

    def fake_ros2(args, timeout=20):
        if "list" in args:
            return _ok_proc("/slam_toolbox/serialize_map\n")
        if "/slam_toolbox/serialize_map" in args:
            assert "/maps/living_room" in args[-1]
            _write_graph(host_dir / "maps", "living_room", b"CUSTOM")
            return _ok_proc("result: 0\n")
        return _ok_proc()

    monkeypatch.setattr(gb_mapping, "_dx", fake_dx)
    monkeypatch.setattr(gb_mapping, "_ros2_call", fake_ros2)

    ok, msg, info = gb_mapping.save(name="living_room", stop_after=False)
    assert ok is True
    assert info["name"] == "living_room"
    assert info["custom"] is True
    assert info["serialized"] is True
    assert (maps_dir / "living_room.yaml").exists()
    assert "image: living_room.pgm" in (maps_dir / "living_room.yaml").read_text()
    assert (maps_dir / "living_room.posegraph").read_bytes().startswith(b"CUSTOM")
    # Master was not overwritten or backed up by the custom save
    assert info["backup"] is None
    assert (maps_dir / "master.posegraph").read_bytes().startswith(b"MASTER")
    assert list((maps_dir / "master.pgm").read_bytes())  # still present

    # Auto-purge of many explore_* maps must not delete living_room or master
    base = 1700000000
    for i in range(8):
        _write_grid(maps_dir, f"explore_2026112{i}_120000" if i < 7 else "explore_20261126_150000",
                    1, 1, [0], mtime=base + i)
    # fix names to actually match _AUTO_MAP_RX
    # The f-string above for i<7 produces explore_20261120_120000 etc. which is valid
    purged = gb_mapping.purge_old_maps()
    assert (maps_dir / "living_room.yaml").exists()
    assert (maps_dir / "living_room.pgm").exists()
    assert (maps_dir / "living_room.posegraph").exists()
    assert (maps_dir / "master.yaml").exists()
    assert (maps_dir / "master.posegraph").exists()
    assert "living_room" not in purged
    assert "master" not in purged
    remaining_auto = [p.stem for p in maps_dir.glob("explore_*.yaml")]
    assert len(remaining_auto) <= gb_mapping.MAP_MAX_KEEP


def test_auto_save_does_not_overwrite_custom_map(tmp_path, monkeypatch):
    maps_dir, host_dir = _patch_paths(monkeypatch, tmp_path)
    _write_grid(maps_dir, "living_room", 2, 2, [1, 2, 3, 4], mtime=1600000000)
    custom_yaml = (maps_dir / "living_room.yaml").read_text()
    custom_pgm = (maps_dir / "living_room.pgm").read_bytes()

    monkeypatch.setattr(gb_mapping, "running", lambda: "3")
    _stub_stop(monkeypatch)
    monkeypatch.setattr(gb_mapping.time, "sleep", lambda s: None)

    def fake_dx(args, timeout=20, user="1000"):
        if "map_saver_cli" in args:
            (host_dir / "maps" / "master.yaml").write_text(
                "image: master.pgm\nmode: trinary\nresolution: 0.05\n"
                "origin: [0.0, 0.0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
            (host_dir / "maps" / "master.pgm").write_bytes(gb_mapping._write_pgm_bytes(1, 1, [254]))
            return _ok_proc()
        return _ok_proc()

    def fake_ros2(args, timeout=20):
        if "list" in args:
            return _ok_proc("/slam_toolbox/serialize_map\n")
        if "/slam_toolbox/serialize_map" in args:
            _write_graph(host_dir / "maps", "master", b"G")
            return _ok_proc("result: 0\n")
        return _ok_proc()

    monkeypatch.setattr(gb_mapping, "_dx", fake_dx)
    monkeypatch.setattr(gb_mapping, "_ros2_call", fake_ros2)
    ok, msg, info = gb_mapping.save(name="explore_20261126_200700", stop_after=False)
    assert ok and info["name"] == "master"
    assert (maps_dir / "living_room.yaml").read_text() == custom_yaml
    assert (maps_dir / "living_room.pgm").read_bytes() == custom_pgm
