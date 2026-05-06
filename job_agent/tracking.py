from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from job_agent.models import Job
from job_agent.storage import ensure_dir, job_key


class TrackingStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.rows: dict[str, dict[str, Any]] = {}
        self.fieldnames = [
            "job_key",
            "created_at",
            "status",
            "score",
            "title",
            "company",
            "location",
            "url",
            "source",
            "notes",
        ]
        self._load()

    def _load(self) -> None:
        if not self.path.exists():
            return
        with self.path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                if "job_key" in row:
                    self.rows[row["job_key"]] = row

    def upsert(self, job: Job, score: float) -> None:
        key = job_key(job)
        if key in self.rows:
            return
        self.rows[key] = {
            "job_key": key,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "status": "new",
            "score": f"{score:.2f}",
            "title": job.title,
            "company": job.company,
            "location": job.location,
            "url": job.url,
            "source": job.source,
            "notes": "",
        }

    def save(self) -> None:
        ensure_dir(self.path.parent)
        with self.path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=self.fieldnames)
            writer.writeheader()
            for row in self.rows.values():
                writer.writerow(row)
