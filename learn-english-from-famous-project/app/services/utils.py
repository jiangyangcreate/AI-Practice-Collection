from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import List, Tuple


class InputKind(Enum):
    URL = auto()
    LOCAL_PATH = auto()


@dataclass
class ParsedInput:
    kind: InputKind
    url: str | None = None
    path: Path | None = None
    repo_name: str | None = None


def parse_user_input(raw: str) -> ParsedInput:
    raw = raw.strip()
    if not raw:
        raise ValueError("输入不能为空")

    if raw.startswith("http://") or raw.startswith("https://"):
        # 只支持完整的 GitHub 仓库 URL，例如 https://github.com/psf/requests
        from urllib.parse import urlparse

        parsed = urlparse(raw)
        if parsed.netloc != "github.com":
            raise ValueError("目前只支持 github.com 的完整仓库 URL")
        parts = [p for p in parsed.path.split("/") if p]
        if len(parts) < 2:
            raise ValueError("请输入完整的仓库 URL，例如 https://github.com/psf/requests")
        owner, repo = parts[0], parts[1]
        repo_name = repo
        return ParsedInput(kind=InputKind.URL, url=raw, repo_name=repo_name)

    path = Path(raw)
    if path.is_absolute() and path.exists() and path.is_dir():
        return ParsedInput(kind=InputKind.LOCAL_PATH, path=path)

    raise ValueError("请输入完整的 GitHub 仓库 URL 或本机绝对路径目录")


def list_completed_projects(results_dir: Path) -> List[Tuple[str, str, str, str]]:
    """
    扫描 data/results，找出 *_translate.jsonl 文件。
    返回 [(project_label, jsonl_name, txt_name, csv_name), ...]，
    project_label 供前端展示，后面三个用于前端下载链接。
    """
    items: List[Tuple[str, str, str, str]] = []
    for path in sorted(results_dir.glob("*_translate.jsonl")):
        label = path.stem.replace("_translate", "")
        base = f"{label}_translate"
        jsonl_name = f"{base}.jsonl"
        txt_name = f"{base}.txt"
        csv_name = f"{base}.csv"
        items.append((label, jsonl_name, txt_name, csv_name))
    return items

