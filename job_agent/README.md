# Job Search Agent (Werkstudent)

Standalone Python module for searching and ranking Werkstudent jobs based on your CV. It is **separate from the C++ car controller** and can be run from the repo root.

## ✅ What it does
- Pulls jobs from RSS, JSON APIs, or a static JSON file
- Normalizes and deduplicates results
- Scores roles against your CV and preferences
- Writes reports (Markdown/CSV/JSON)
- Maintains a simple `applications.csv` tracking list
- Optional email digest (SMTP)

## ⚠️ Data handling & privacy
- Your CV (`cv.json`) and config (`config.json`) are **ignored by Git**.
- Reports and tracking data are written to `job_agent/reports/` and `job_agent/data/`.
- Keep secrets (SMTP password) out of the repo.

## Quick start
```bash
# From repo root
cp job_agent/config.example.json job_agent/config.json
cp job_agent/cv.example.json job_agent/cv.json

# Run a single search (uses sample_jobs.json by default)
python -m job_agent.main
```

## Enable real sources
Edit `job_agent/config.json` and add sources:

- **RSS** (e.g., Indeed RSS via `&rss=1`)
- **JSON API** (company career API)
- **Static** (local JSON export)

Set `"enabled": true` for any source you want active.
You can also set `timeout_seconds` per source to avoid hanging requests.

## Reports
Reports are written to `job_agent/reports/`:
- `report.md`
- `report.csv`
- `report.json`

## Tracking list
A simple `applications.csv` is generated in `job_agent/data/`. Use it to track applied / interviewing / rejected status.

## Scheduler
Run every N minutes:
```bash
python -m job_agent.main --watch
```

## Add/remove sources
- Add a new entry under `sources` in `config.json`.
- Disable a source by setting `"enabled": false`.

## Tune filters
Edit `profile` in `config.json`:
- `title_keywords`
- `fields`
- `locations`
- `languages`
- `hours_per_week_max`
- `require_werkstudent`
