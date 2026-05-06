from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass(frozen=True)
class Job:
    job_id: str
    source: str
    title: str
    company: str
    location: str
    url: str
    description: str = ""
    tags: list[str] = field(default_factory=list)
    language: Optional[str] = None
    hours_per_week: Optional[int] = None
    posted_at: Optional[str] = None
