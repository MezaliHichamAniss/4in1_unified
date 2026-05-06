from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

from job_agent.models import Job
from job_agent.sources.base import JobSource


class StaticSource(JobSource):
    def __init__(self, name: str, path: Path) -> None:
        super().__init__(name)
        self.path = path

    def fetch(self) -> Iterable[Job]:
        with self.path.open("r", encoding="utf-8") as handle:
            items = json.load(handle)
        jobs: list[Job] = []
        for item in items:
            jobs.append(
                Job(
                    job_id=str(item.get("job_id") or item.get("url") or item.get("title")),
                    source=self.name,
                    title=str(item.get("title") or "").strip(),
                    company=str(item.get("company") or self.name).strip(),
                    location=str(item.get("location") or "").strip(),
                    url=str(item.get("url") or "").strip(),
                    description=str(item.get("description") or "").strip(),
                    tags=list(item.get("tags") or []),
                    language=item.get("language"),
                    hours_per_week=item.get("hours_per_week"),
                    posted_at=item.get("posted_at"),
                )
            )
        return jobs
