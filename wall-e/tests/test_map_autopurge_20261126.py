import os
import sys
import time
from pathlib import Path

sys.path.insert(0, "/home/pi/grokbot/releases/e48d3d8")
import gb_mapping


def _touch_map(dirpath: Path, stem: str, mtime: float):
    for ext in (".yaml", ".pgm"):
        fp = dirpath / (stem + ext)
        fp.write_text(f"dummy {stem}{ext}")
        os.utime(fp, (mtime, mtime))


def test_default_policy_is_keep_two():
    assert gb_mapping.MAP_MAX_KEEP == 5


def test_purge_keeps_two_newest_and_preserves_custom_maps(tmp_path, monkeypatch):
    maps_dir = tmp_path / "maps"
    host_dir = tmp_path / "host"
    (host_dir / "maps").mkdir(parents=True)
    maps_dir.mkdir(parents=True)
    monkeypatch.setattr(gb_mapping, "MAPS", str(maps_dir))
    monkeypatch.setattr(gb_mapping, "HOST_DIR", str(host_dir))
    monkeypatch.setattr(gb_mapping, "_log", lambda msg: None)

    base = 1700000000.0
    stems = [
        "explore_20261124_062756",
        "explore_20261125_124809",
        "explore_20261126_194352",
        "explore_20261126_200700",
    ]
    for i, s in enumerate(stems):
        _touch_map(maps_dir, s, base + i * 100)
        _touch_map(host_dir / "maps", s, base + i * 100)
    # Custom user-named map must never be purged
    _touch_map(maps_dir, "living_room", base - 500)

    purged = gb_mapping.purge_old_maps(max_keep=2, keep=("explore_20261126_200700",))

    assert set(purged) == {"explore_20261124_062756", "explore_20261125_124809"}
    remaining_yaml = sorted(p.stem for p in maps_dir.glob("*.yaml"))
    assert remaining_yaml == [
        "explore_20261126_194352",
        "explore_20261126_200700",
        "living_room",
    ]
    remaining_host = sorted(p.stem for p in (host_dir / "maps").glob("*.yaml"))
    assert remaining_host == [
        "explore_20261126_194352",
        "explore_20261126_200700",
    ]


def test_purge_with_two_or_fewer_deletes_nothing(tmp_path, monkeypatch):
    maps_dir = tmp_path / "maps"
    host_dir = tmp_path / "host"
    (host_dir / "maps").mkdir(parents=True)
    maps_dir.mkdir(parents=True)
    monkeypatch.setattr(gb_mapping, "MAPS", str(maps_dir))
    monkeypatch.setattr(gb_mapping, "HOST_DIR", str(host_dir))
    monkeypatch.setattr(gb_mapping, "_log", lambda msg: None)

    _touch_map(maps_dir, "explore_20261126_194352", 1700000100.0)
    _touch_map(maps_dir, "explore_20261126_200700", 1700000200.0)

    purged = gb_mapping.purge_old_maps(max_keep=2)
    assert purged == []
    assert len(list(maps_dir.glob("*.yaml"))) == 2
