from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class ConfigError(RuntimeError):
    pass


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise ConfigError(
            f"Missing config file: {path}. Copy config.example.json to config.json and fill it in."
        )
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_config(path: Path) -> dict[str, Any]:
    config = load_json(path)
    config.setdefault("profile", {})
    config.setdefault("sources", [])
    config.setdefault("output", {})
    config.setdefault("scheduler", {})
    config.setdefault("tracking", {})
    return config


def load_cv(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise ConfigError(
            f"Missing CV file: {path}. Copy cv.example.json to cv.json and fill it in."
        )
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)
