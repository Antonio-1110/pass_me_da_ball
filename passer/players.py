"""
players.py -- per-player settings, applied for the length of a session.

Each player is a small YAML file in app.players_dir, e.g.
configs/players/antonio.yaml:

    name: Antonio
    person_height_m: 1.78        # the camera judges distance from body height
    lead_m: 1.5                  # how far ahead side passes are led
    passes:
      chest: {launch_angle_deg: 20, target_height_m: 1.4}
      lob:   {launch_angle_deg: 55, target_height_m: 2.1}

Only the fields in FIELDS / PASS_FIELDS can be set, each within a range, so a
profile can tailor the throw to a player but never touch the machine's hard
limits (max rpm, arm angles, ...). Every throw still goes through the normal
launch-plan checks. Leaving a field out keeps the machine's own setting.
"""

from __future__ import annotations

import logging
import re
from pathlib import Path
from typing import Any, Dict, List, Optional

from .config import Config

log = logging.getLogger(__name__)

# field -> (min, max, unit, label)
FIELDS = {
    "person_height_m": (1.0, 2.3, "m", "Height"),
    "lead_m": (0.0, 3.0, "m", "Side-pass lead"),
}
PASS_FIELDS = {
    "launch_angle_deg": (5.0, 70.0, "°", "Arc (launch angle)"),
    "target_height_m": (0.5, 3.0, "m", "Catch height"),
}
ID_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,39}$")


def player_id(name: str) -> str:
    """File-safe id from a display name: "Antonio M." -> "antonio-m"."""
    pid = re.sub(r"[^a-z0-9]+", "-", name.strip().lower()).strip("-")[:40]
    if not pid or not ID_RE.match(pid):
        raise ValueError("name needs at least one letter or digit")
    return pid


def _number(where: str, value: Any, lo: float, hi: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{where}: expected a number, got {value!r}")
    if not lo <= value <= hi:
        raise ValueError(f"{where}: {value} is outside {lo}..{hi}")
    return float(value)


def validate(data: dict, pass_names) -> dict:
    """A clean player dict, or ValueError/TypeError naming the bad field."""
    if not isinstance(data, dict):
        raise TypeError("expected a player object")
    name = str(data.get("name", "")).strip()
    if not name or len(name) > 40:
        raise ValueError("name: 1-40 characters")
    out: Dict[str, Any] = {"name": name}
    for k, (lo, hi, _, _) in FIELDS.items():
        if data.get(k) is not None:
            out[k] = _number(k, data[k], lo, hi)
    passes = data.get("passes") or {}
    if not isinstance(passes, dict):
        raise TypeError("passes: expected a mapping")
    for pname, vals in passes.items():
        if pname not in pass_names:
            raise ValueError(f"passes.{pname}: unknown pass type (have {', '.join(pass_names)})")
        if not isinstance(vals, dict):
            raise TypeError(f"passes.{pname}: expected a mapping")
        clean = {k: _number(f"passes.{pname}.{k}", vals[k], *PASS_FIELDS[k][:2])
                 for k in PASS_FIELDS if vals.get(k) is not None}
        unknown = set(vals) - set(PASS_FIELDS)
        if unknown:
            raise ValueError(f"passes.{pname}: can't set {', '.join(sorted(unknown))}")
        if clean:
            out.setdefault("passes", {})[pname] = clean
    unknown = set(data) - set(FIELDS) - {"name", "passes", "id"}
    if unknown:
        raise ValueError(f"can't set {', '.join(sorted(unknown))} in a player profile")
    return out


def overrides(player: dict) -> dict:
    """The config patch a player profile stands for."""
    patch: Dict[str, Any] = {}
    if "person_height_m" in player:
        patch["distance"] = {"person_height_m": player["person_height_m"]}
    if "lead_m" in player:
        patch["launcher_axis"] = {"lead_m": player["lead_m"]}
    if player.get("passes"):
        patch["launcher"] = {"profiles": {k: dict(v) for k, v in player["passes"].items()}}
    return patch


def current_values(cfg: Config, patch: dict) -> dict:
    """The config's present values for every field in `patch` (to restore later)."""
    out: Dict[str, Any] = {}
    for section, fields in patch.items():
        obj = getattr(cfg, section)
        if section == "launcher":
            out["launcher"] = {"profiles": {
                p: {k: getattr(obj.profiles[p], k) for k in vals}
                for p, vals in fields["profiles"].items()}}
        else:
            out[section] = {k: getattr(obj, k) for k in fields}
    return out


class PlayerStore:
    """Player profiles on disk, plus which one the next session uses."""

    def __init__(self, directory: str | Path):
        self.dir = Path(directory)
        self.selected: Optional[str] = None

    def _path(self, pid: str) -> Path:
        if not ID_RE.match(pid or ""):
            raise ValueError(f"bad player id {pid!r}")
        return self.dir / f"{pid}.yaml"

    def list(self) -> List[dict]:
        import yaml

        out = []
        for f in sorted(self.dir.glob("*.yaml")) if self.dir.is_dir() else []:
            try:
                data = yaml.safe_load(f.read_text(encoding="utf-8")) or {}
                out.append({"id": f.stem, **data})
            except Exception:
                log.exception("unreadable player profile %s", f)
        return out

    def get(self, pid: str) -> Optional[dict]:
        import yaml

        f = self._path(pid)
        if not f.is_file():
            return None
        return {"id": pid, **(yaml.safe_load(f.read_text(encoding="utf-8")) or {})}

    def save(self, data: dict, cfg: Config) -> dict:
        """Create or update; a new name for an existing id renames the file."""
        import yaml

        clean = validate(data, list(cfg.launcher.profiles))
        pid = player_id(clean["name"])
        old = data.get("id")
        if old and old != pid and self.get(pid) is not None:
            raise ValueError(f"a player called {clean['name']!r} already exists")
        if not old and self.get(pid) is not None:
            raise ValueError(f"a player called {clean['name']!r} already exists")
        self.dir.mkdir(parents=True, exist_ok=True)
        self._path(pid).write_text("# Player profile, edited from the dashboard.\n"
                                   + yaml.safe_dump(clean, sort_keys=False), encoding="utf-8")
        if old and old != pid:
            self._path(old).unlink(missing_ok=True)
            if self.selected == old:
                self.selected = pid
        log.info("Player profile saved: %s", pid)
        return {"id": pid, **clean}

    def delete(self, pid: str) -> None:
        self._path(pid).unlink(missing_ok=True)
        if self.selected == pid:
            self.selected = None
        log.info("Player profile deleted: %s", pid)

    def select(self, pid: Optional[str]) -> None:
        if pid is not None and self.get(pid) is None:
            raise ValueError(f"no player {pid!r}")
        self.selected = pid
