from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable

from job_agent.sources.base import JobSource
from job_agent.sources.json_api import JsonApiSource
from job_agent.sources.rss import RssSource
from job_agent.sources.static import StaticSource


def build_sources(configs: Iterable[dict[str, Any]], base_dir: Path) -> list[JobSource]:
    sources: list[JobSource] = []
    for config in configs:
        if config.get("enabled", True) is False:
            continue
        source_type = config.get("type")
        name = config.get("name") or source_type or "source"
        if source_type == "rss":
            sources.append(RssSource(name, config["url"]))
        elif source_type == "json":
            sources.append(
                JsonApiSource(
                    name,
                    config["url"],
                    config.get("items_path"),
                    config.get("mapping", {}),
                )
            )
        elif source_type == "static":
            path = Path(config["path"])
            if not path.is_absolute():
                path = base_dir / path
            sources.append(StaticSource(name, path))
        else:
            raise ValueError(f"Unknown source type: {source_type}")
    return sources
