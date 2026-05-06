from __future__ import annotations

import re
from typing import Any

from job_agent.models import Job


def _normalize(text: str) -> str:
    return re.sub(r"\s+", " ", text.lower()).strip()


def _keyword_hits(text: str, keywords: list[str]) -> list[str]:
    hits: list[str] = []
    normalized = _normalize(text)
    for keyword in keywords:
        if keyword and keyword.lower() in normalized:
            hits.append(keyword)
    return hits


def _to_list(value: Any) -> list[str]:
    if not value:
        return []
    if isinstance(value, list):
        return [str(item) for item in value if str(item).strip()]
    return [str(value)]


def score_job(job: Job, profile: dict[str, Any], cv: dict[str, Any]) -> tuple[float, list[str]]:
    combined_text = " ".join(
        [job.title, job.company, job.location, job.description, " ".join(job.tags)]
    )
    reasons: list[str] = []
    score = 0.0

    title_keywords = _to_list(profile.get("title_keywords"))
    title_hits = _keyword_hits(job.title, title_keywords)
    if title_hits:
        score += 3.0
        reasons.append(f"title:{', '.join(sorted(set(title_hits)))}")

    field_keywords = _to_list(profile.get("fields"))
    field_hits = _keyword_hits(combined_text, field_keywords)
    if field_hits:
        score += min(10.0, 0.75 * len(field_hits))
        reasons.append(f"fields:{', '.join(sorted(set(field_hits)))}")

    cv_keywords = _to_list(cv.get("skills")) + _to_list(cv.get("domains"))
    cv_hits = _keyword_hits(combined_text, cv_keywords)
    if cv_hits:
        score += min(8.0, 0.5 * len(cv_hits))
        reasons.append(f"skills:{', '.join(sorted(set(cv_hits)))}")

    location_keywords = _to_list(profile.get("locations"))
    location_hits = _keyword_hits(job.location or combined_text, location_keywords)
    if location_hits:
        score += 1.5
        reasons.append(f"location:{', '.join(sorted(set(location_hits)))}")

    language_keywords = _to_list(profile.get("languages"))
    language_hits = _keyword_hits(combined_text, language_keywords)
    if language_hits:
        score += 0.5
        reasons.append(f"language:{', '.join(sorted(set(language_hits)))}")

    return score, reasons


def filter_job(job: Job, profile: dict[str, Any]) -> bool:
    combined_text = " ".join(
        [job.title, job.company, job.location, job.description, " ".join(job.tags)]
    )
    require_werkstudent = profile.get("require_werkstudent", True)
    if require_werkstudent:
        title_keywords = _to_list(profile.get("title_keywords"))
        if title_keywords and not _keyword_hits(job.title, title_keywords):
            return False

    locations = _to_list(profile.get("locations"))
    if locations:
        location_hits = _keyword_hits(job.location or combined_text, locations)
        if not location_hits:
            return False

    languages = _to_list(profile.get("languages"))
    if languages:
        language_hits = _keyword_hits(combined_text, languages)
        if not language_hits and job.language and job.language not in languages:
            return False

    max_hours = profile.get("hours_per_week_max")
    if max_hours and job.hours_per_week and job.hours_per_week > max_hours:
        return False

    return True


def rank_jobs(
    jobs: list[Job],
    profile: dict[str, Any],
    cv: dict[str, Any],
) -> list[dict[str, Any]]:
    ranked: list[dict[str, Any]] = []
    for job in jobs:
        if not filter_job(job, profile):
            continue
        score, reasons = score_job(job, profile, cv)
        ranked.append({"job": job, "score": score, "reasons": reasons})
    ranked.sort(key=lambda item: item["score"], reverse=True)
    return ranked
