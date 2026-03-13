from __future__ import annotations

from pathlib import Path
from typing import List, Tuple

import subprocess

import requests
from bs4 import BeautifulSoup
from fastapi import BackgroundTasks, FastAPI, Form, HTTPException, Request
from fastapi.responses import FileResponse, HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from .services.processing import process_project
from .services.utils import (
    ParsedInput,
    InputKind,
    parse_user_input,
    list_completed_projects,
)

BASE_DIR = Path(__file__).resolve().parent.parent
DATA_DIR = BASE_DIR / "data"
RESULTS_DIR = DATA_DIR / "results"
REPOS_DIR = DATA_DIR / "repos"

DATA_DIR.mkdir(exist_ok=True)
RESULTS_DIR.mkdir(exist_ok=True)
REPOS_DIR.mkdir(exist_ok=True)

app = FastAPI(title="Learn English from Famous Python Projects")

templates = Jinja2Templates(directory=str(BASE_DIR / "templates"))


@app.get("/", response_class=HTMLResponse)
def index(request: Request) -> HTMLResponse:
    trending = fetch_trending_projects(limit=5)
    completed = list_completed_projects(RESULTS_DIR)
    return templates.TemplateResponse(
        "index.html",
        {
            "request": request,
            "trending": trending,
            "completed": completed,
        },
    )


@app.post("/analyze", response_class=HTMLResponse)
def analyze(
    request: Request,
    background_tasks: BackgroundTasks,
    project_input: str = Form(...),
) -> HTMLResponse:
    parsed: ParsedInput
    try:
        parsed = parse_user_input(project_input)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc))

    if parsed.kind is InputKind.URL:
        repo_name = parsed.repo_name
        dest = REPOS_DIR / repo_name
        # 不重复拉取：如果目录已存在则复用
        if not dest.exists():
            clone_repository(parsed.url, dest)
        project_path = dest
        project_label = repo_name
    elif parsed.kind is InputKind.LOCAL_PATH:
        project_path = parsed.path
        project_label = project_path.name
    else:
        raise HTTPException(status_code=400, detail="Unsupported input type")

    background_tasks.add_task(
        process_project,
        project_path=project_path,
        project_label=project_label,
        results_dir=RESULTS_DIR,
    )

    trending = fetch_trending_projects(limit=5)
    completed = list_completed_projects(RESULTS_DIR)
    return templates.TemplateResponse(
        "index.html",
        {
            "request": request,
            "trending": trending,
            "completed": completed,
            "message": f"任务已启动：{project_label}，请稍后刷新页面查看结果。",
        },
    )


@app.get("/download/{filename}")
def download_result(filename: str):
    target = RESULTS_DIR / filename
    if not target.exists():
        raise HTTPException(status_code=404, detail="File not found")
    return FileResponse(
        path=str(target),
        media_type="application/jsonl",
        filename=filename,
    )


def fetch_trending_projects(limit: int = 5) -> List[Tuple[str, int, str]]:
    """
    返回 [(name, stars_this_month, url), ...]
    """
    url = "https://github.com/trending/python?since=monthly"
    resp = requests.get(url, timeout=10)
    resp.raise_for_status()
    soup = BeautifulSoup(resp.text, "html.parser")

    results: List[Tuple[str, int, str]] = []
    for repo_item in soup.select("article.Box-row")[:limit]:
        a = repo_item.select_one("h2 a")
        if not a:
            continue
        repo_path = a.get("href", "").strip("/")
        full_name = repo_path  # e.g. "psf/requests"
        link = f"https://github.com/{full_name}"

        stars_text = "0"
        for el in repo_item.select("span.d-inline-block.float-sm-right"):
            stars_text = el.text.strip().split(" ")[0].replace(",", "")
            break
        try:
            stars = int(stars_text)
        except ValueError:
            stars = 0

        results.append((full_name, stars, link))

    return results


def clone_repository(url: str, dest: Path) -> None:
    completed = subprocess.run(
        ["git", "clone", "--depth", "1", url, str(dest)],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        raise HTTPException(
            status_code=400,
            detail=f"Git clone 失败: {completed.stderr.strip()}",
        )


@app.get("/health")
def health():
    return {"status": "ok"}

