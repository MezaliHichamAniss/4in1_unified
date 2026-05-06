from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Iterable

from job_agent.models import Job


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def job_key(job: Job) -> str:
    base = job.job_id or job.url or f"{job.title}|{job.company}"
    payload = f"{job.source}|{base}".encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


class SeenStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.seen: set[str] = set()
        if path.exists():
            with path.open("r", encoding="utf-8") as handle:
                self.seen = set(json.load(handle))

    def is_seen(self, key: str) -> bool:
        return key in self.seen

    def add(self, keys: Iterable[str]) -> None:
        self.seen.update(keys)

    def save(self) -> None:
        ensure_dir(self.path.parent)
        with self.path.open("w", encoding="utf-8") as handle:
            json.dump(sorted(self.seen), handle, ensure_ascii=False, indent=2)
