from __future__ import annotations

import html
import urllib.request
import xml.etree.ElementTree as ET
from typing import Iterable

from job_agent.models import Job
from job_agent.sources.base import JobSource

ATOM_NS = "{http://www.w3.org/2005/Atom}"


def _text(item: ET.Element, tags: list[str]) -> str:
    for tag in tags:
        node = item.find(tag)
        if node is not None and node.text:
            return node.text.strip()
    return ""


def _link(item: ET.Element) -> str:
    link = item.find("link")
    if link is not None:
        href = link.attrib.get("href")
        if href:
            return href.strip()
        if link.text:
            return link.text.strip()
    link = item.find(f"{ATOM_NS}link")
    if link is not None:
        href = link.attrib.get("href")
        if href:
            return href.strip()
    return ""


def _split_title_company(title: str) -> tuple[str, str]:
    for sep in (" - ", " | ", " – "):
        if sep in title:
            left, right = title.split(sep, 1)
            return left.strip(), right.strip()
    return title.strip(), ""


class RssSource(JobSource):
    def __init__(self, name: str, url: str) -> None:
        super().__init__(name)
        self.url = url

    def fetch(self) -> Iterable[Job]:
        with urllib.request.urlopen(self.url) as response:
            data = response.read()
        root = ET.fromstring(data)
        items = root.findall(".//item")
        if not items:
            items = root.findall(f".//{ATOM_NS}entry")
        jobs: list[Job] = []
        for item in items:
            raw_title = _text(item, ["title", f"{ATOM_NS}title"])
            title, company = _split_title_company(html.unescape(raw_title))
            description = html.unescape(
                _text(item, ["description", f"{ATOM_NS}summary"]) or ""
            )
            url = _link(item)
            job_id = _text(item, ["guid", f"{ATOM_NS}id"]) or url or title
            location = _text(item, ["location", f"{ATOM_NS}location"])
            posted_at = _text(item, ["pubDate", f"{ATOM_NS}updated", f"{ATOM_NS}published"])
            jobs.append(
                Job(
                    job_id=job_id,
                    source=self.name,
                    title=title,
                    company=company or self.name,
                    location=location,
                    url=url,
                    description=description,
                    posted_at=posted_at,
                )
            )
        return jobs
