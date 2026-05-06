from __future__ import annotations

import json
import urllib.request
from typing import Any, Iterable

from job_agent.models import Job
from job_agent.sources.base import JobSource


def _get_by_path(data: Any, path: str) -> Any:
    current = data
    for part in path.split("."):
        if isinstance(current, list):
            try:
                index = int(part)
            except ValueError:
                return None
            if index >= len(current):
                return None
            current = current[index]
        elif isinstance(current, dict):
            current = current.get(part)
        else:
            return None
    return current


class JsonApiSource(JobSource):
    def __init__(self, name: str, url: str, items_path: str | None, mapping: dict[str, str]) -> None:
        super().__init__(name)
        self.url = url
        self.items_path = items_path
        self.mapping = mapping

    def _extract(self, item: dict[str, Any], key: str) -> Any:
        path = self.mapping.get(key)
        if not path:
            return None
        return _get_by_path(item, path)

    def fetch(self) -> Iterable[Job]:
        with urllib.request.urlopen(self.url) as response:
            data = json.loads(response.read().decode("utf-8"))
        items = _get_by_path(data, self.items_path) if self.items_path else data
        if not isinstance(items, list):
            return []
        jobs: list[Job] = []
        for item in items:
            if not isinstance(item, dict):
                continue
            tags = self._extract(item, "tags") or []
            if isinstance(tags, str):
                tags = [tag.strip() for tag in tags.split(",") if tag.strip()]
            hours = self._extract(item, "hours_per_week")
            if isinstance(hours, str) and hours.isdigit():
                hours = int(hours)
            jobs.append(
                Job(
                    job_id=str(self._extract(item, "id") or self._extract(item, "url") or ""),
                    source=self.name,
                    title=str(self._extract(item, "title") or "").strip(),
                    company=str(self._extract(item, "company") or self.name).strip(),
                    location=str(self._extract(item, "location") or "").strip(),
                    url=str(self._extract(item, "url") or "").strip(),
                    description=str(self._extract(item, "description") or "").strip(),
                    tags=tags,
                    language=self._extract(item, "language"),
                    hours_per_week=hours,
                    posted_at=self._extract(item, "posted_at"),
                )
            )
        return jobs
