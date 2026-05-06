from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

from job_agent.config import ConfigError, load_config, load_cv
from job_agent.matching import rank_jobs
from job_agent.reporting import maybe_email, notify_console, write_reports
from job_agent.sources import build_sources
from job_agent.storage import SeenStore, job_key
from job_agent.tracking import TrackingStore


def _resolve_path(path: str | None, default: Path) -> Path:
    if path:
        return Path(path).expanduser()
    return default


def _resolve_relative(base_dir: Path, value: str | Path | None, fallback: Path) -> Path:
    if value:
        path = Path(value).expanduser()
    else:
        path = fallback
    if not path.is_absolute():
        path = base_dir / path
    return path


def _run_once(config_path: Path, cv_path: Path, limit: int) -> int:
    config = load_config(config_path)
    cv = load_cv(cv_path)

    base_dir = config_path.parent
    sources = build_sources(config.get("sources", []), base_dir)

    all_jobs = []
    for source in sources:
        try:
            all_jobs.extend(list(source.fetch()))
        except Exception as exc:  # noqa: BLE001 - keep agent running
            print(f"[warn] source {source.name} failed: {exc}")

    profile = config.get("profile", {})
    ranked = rank_jobs(all_jobs, profile, cv)

    output_config = config.get("output", {})
    report_dir = _resolve_relative(base_dir, output_config.get("report_dir"), base_dir / "reports")
    formats = output_config.get("formats", ["markdown", "csv", "json"])
    report_paths = write_reports(ranked, report_dir, formats, limit)

    notify_console(ranked, limit)
    maybe_email(output_config.get("email", {}), ranked, limit)

    tracking_config = config.get("tracking", {})
    tracking_path = _resolve_relative(
        base_dir, tracking_config.get("file"), base_dir / "data" / "applications.csv"
    )
    tracking = TrackingStore(tracking_path)
    min_score = tracking_config.get("auto_add_min_score", 0.0)
    for item in ranked:
        if item["score"] >= min_score:
            tracking.upsert(item["job"], item["score"])
    tracking.save()

    seen_path = _resolve_relative(
        base_dir, tracking_config.get("seen_file"), base_dir / "data" / "seen.json"
    )
    seen = SeenStore(seen_path)
    new_keys = [job_key(item["job"]) for item in ranked]
    seen.add(new_keys)
    seen.save()

    if report_paths:
        print("\nReports written:")
        for fmt, path in report_paths.items():
            print(f"- {fmt}: {path}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Werkstudent job search agent")
    parser.add_argument(
        "--config",
        default=None,
        help="Path to config.json (default: job_agent/config.json)",
    )
    parser.add_argument(
        "--cv",
        default=None,
        help="Path to cv.json (default: job_agent/cv.json)",
    )
    parser.add_argument("--limit", type=int, default=25, help="Max results per report")
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Run on a schedule defined in config.scheduler.interval_minutes",
    )

    args = parser.parse_args(argv)
    base_dir = Path(__file__).resolve().parent
    config_path = _resolve_path(args.config, base_dir / "config.json")
    cv_path = _resolve_path(args.cv, base_dir / "cv.json")

    try:
        if not args.watch:
            return _run_once(config_path, cv_path, args.limit)

        interval = load_config(config_path).get("scheduler", {}).get("interval_minutes", 180)
        while True:
            _run_once(config_path, cv_path, args.limit)
            time.sleep(max(1, int(interval)) * 60)
    except ConfigError as exc:
        print(str(exc))
        return 1


if __name__ == "__main__":
    sys.exit(main())
