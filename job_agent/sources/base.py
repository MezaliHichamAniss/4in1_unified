from __future__ import annotations

from typing import Iterable

from job_agent.models import Job


class JobSource:
    def __init__(self, name: str) -> None:
        self.name = name

    def fetch(self) -> Iterable[Job]:
        raise NotImplementedError
