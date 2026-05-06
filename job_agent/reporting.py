from __future__ import annotations

import csv
import json
import smtplib
from email.message import EmailMessage
from pathlib import Path
from typing import Any

from job_agent.models import Job
from job_agent.storage import ensure_dir


def _job_to_dict(item: dict[str, Any]) -> dict[str, Any]:
    job: Job = item["job"]
    return {
        "score": round(item["score"], 2),
        "title": job.title,
        "company": job.company,
        "location": job.location,
        "url": job.url,
        "source": job.source,
        "posted_at": job.posted_at,
        "reasons": "; ".join(item["reasons"]),
    }


def write_reports(
    ranked: list[dict[str, Any]],
    report_dir: Path,
    formats: list[str],
    limit: int,
) -> dict[str, Path]:
    ensure_dir(report_dir)
    payload = [_job_to_dict(item) for item in ranked[:limit]]
    outputs: dict[str, Path] = {}

    if "json" in formats:
        path = report_dir / "report.json"
        with path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2)
        outputs["json"] = path

    if "csv" in formats:
        path = report_dir / "report.csv"
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(payload[0].keys()) if payload else [])
            if payload:
                writer.writeheader()
                writer.writerows(payload)
        outputs["csv"] = path

    if "markdown" in formats:
        path = report_dir / "report.md"
        with path.open("w", encoding="utf-8") as handle:
            handle.write("# Werkstudent job matches\n\n")
            if not payload:
                handle.write("No matches found.\n")
            else:
                handle.write("| Score | Title | Company | Location | Source |\n")
                handle.write("| --- | --- | --- | --- | --- |\n")
                for item in payload:
                    handle.write(
                        f"| {item['score']} | {item['title']} | {item['company']} | {item['location']} | {item['source']} |\n"
                    )
        outputs["markdown"] = path

    return outputs


def notify_console(ranked: list[dict[str, Any]], limit: int) -> None:
    print("\nTop matches:")
    for item in ranked[:limit]:
        job: Job = item["job"]
        print(f"- {item['score']:.2f} | {job.title} | {job.company} | {job.location} | {job.url}")


def maybe_email(
    email_config: dict[str, Any],
    ranked: list[dict[str, Any]],
    limit: int,
) -> None:
    if not email_config.get("enabled"):
        return
    host = email_config.get("smtp_host")
    port = email_config.get("smtp_port", 587)
    username = email_config.get("username")
    password = email_config.get("password")
    from_addr = email_config.get("from")
    to_addrs = email_config.get("to", [])
    if not all([host, username, password, from_addr, to_addrs]):
        return

    message = EmailMessage()
    message["Subject"] = email_config.get("subject", "Werkstudent job matches")
    message["From"] = from_addr
    message["To"] = ", ".join(to_addrs)
    lines = []
    for item in ranked[:limit]:
        job: Job = item["job"]
        lines.append(
            f"{item['score']:.2f} | {job.title} | {job.company} | {job.location}\n{job.url}\n"
        )
    message.set_content("\n".join(lines) or "No matches found.")

    with smtplib.SMTP(host, port) as smtp:
        smtp.starttls()
        smtp.login(username, password)
        smtp.send_message(message)
