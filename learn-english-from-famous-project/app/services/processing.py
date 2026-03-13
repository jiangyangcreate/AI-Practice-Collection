from __future__ import annotations

import ast
import json
import re
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable

from .translation import translate_word_list


CAMEL_CASE_BOUNDARY_RE = re.compile(r"(?<!^)(?=[A-Z])")


def process_project(project_path: Path, project_label: str, results_dir: Path) -> None:
    """
    后台任务：遍历项目，提取标识符，写出 ast.jsonl / words.jsonl / translate.jsonl。
    支持按已有文件增量执行：
      - 若 *_ast.jsonl 不存在：重新遍历项目并写出；
      - 若 *_words.jsonl 不存在：基于 ast 结果生成并写出；
      - 若 *_translate.jsonl 不存在：基于 words 结果调用翻译服务并写出。
    """
    ast_file = results_dir / f"{project_label}_ast.jsonl"
    words_file = results_dir / f"{project_label}_words.jsonl"
    translate_file = results_dir / f"{project_label}_translate.jsonl"
    txt_file = results_dir / f"{project_label}_translate.txt"
    csv_file = results_dir / f"{project_label}_translate.csv"

    # 1) AST 统计
    if ast_file.exists():
        ast_counts = read_counter_jsonl(ast_file, key_field="name")
    else:
        ast_counts = collect_ast_names(project_path)
        write_counter_jsonl(ast_counts, ast_file, key_field="name")

    # 2) 单词统计
    if words_file.exists():
        word_counts = read_counter_jsonl(words_file, key_field="word")
    else:
        word_counts = split_words_from_identifiers(ast_counts)
        write_counter_jsonl(word_counts, words_file, key_field="word")

    # 3) 翻译结果（JSONL）
    if not translate_file.exists():
        translations = translate_word_list(word_counts.keys())
        with translate_file.open("w", encoding="utf-8") as f:
            for word, line in translations:
                obj = {"word": word, "translation": line}
                f.write(json.dumps(obj, ensure_ascii=False) + "\n")

    # 4 / 5) 文本和 CSV 视图（基于 translate.jsonl 派生）
    ensure_txt_and_csv_from_translate(
        translate_file=translate_file,
        txt_file=txt_file,
        csv_file=csv_file,
    )


def write_counter_jsonl(counter: Counter, path: Path, key_field: str) -> None:
    with path.open("w", encoding="utf-8") as f:
        for key, count in sorted(counter.items(), key=lambda kv: (-kv[1], kv[0])):
            obj = {key_field: key, "count": count}
            f.write(json.dumps(obj, ensure_ascii=False) + "\n")


def read_counter_jsonl(path: Path, key_field: str) -> Counter:
    counter: Counter = Counter()
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            key = obj.get(key_field)
            count = obj.get("count", 0)
            if isinstance(key, str) and isinstance(count, int):
                counter[key] += count
    return counter


def ensure_txt_and_csv_from_translate(
    translate_file: Path,
    txt_file: Path,
    csv_file: Path,
) -> None:
    """
    基于 *_translate.jsonl 生成：
      - *_translate.txt
      - *_translate.csv
    若目标文件已存在则跳过。
    """
    if not translate_file.exists():
        return

    pairs: list[tuple[str, str]] = []
    with translate_file.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            word = obj.get("word")
            translation = obj.get("translation", "")
            if isinstance(word, str):
                pairs.append((word, str(translation)))

    if pairs and not txt_file.exists():
        with txt_file.open("w", encoding="utf-8") as f_txt:
            for word, translation in pairs:
                f_txt.write(f"{word}\n")
                f_txt.write(f"{translation}\n")

    if pairs and not csv_file.exists():
        import csv

        with csv_file.open("w", encoding="utf-8", newline="") as f_csv:
            writer = csv.writer(f_csv)
            # 不写表头，直接：单词,翻译
            for word, translation in pairs:
                writer.writerow([word, translation])


def collect_ast_names(root: Path) -> Counter:
    counter: Counter = Counter()

    for py_file in root.rglob("*.py"):
        if any(part == ".venv" for part in py_file.parts):
            continue
        try:
            text = py_file.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        try:
            tree = ast.parse(text, filename=str(py_file))
        except SyntaxError:
            continue

        visitor = NameCollector(counter)
        visitor.visit(tree)

    return counter


class NameCollector(ast.NodeVisitor):
    def __init__(self, counter: Counter):
        self.counter = counter

    def _add(self, name: str) -> None:
        if not name:
            return
        self.counter[name] += 1

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:  # type: ignore[override]
        self._add(node.name)
        self.generic_visit(node)

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:  # type: ignore[override]
        self._add(node.name)
        self.generic_visit(node)

    def visit_ClassDef(self, node: ast.ClassDef) -> None:  # type: ignore[override]
        self._add(node.name)
        self.generic_visit(node)

    def visit_Name(self, node: ast.Name) -> None:  # type: ignore[override]
        self._add(node.id)
        self.generic_visit(node)

    def visit_Attribute(self, node: ast.Attribute) -> None:  # type: ignore[override]
        parts = []
        cur: ast.AST | None = node
        while isinstance(cur, ast.Attribute):
            parts.append(cur.attr)
            cur = cur.value
        if isinstance(cur, ast.Name):
            parts.append(cur.id)
        dotted = ".".join(reversed(parts))
        if dotted:
            self._add(dotted)
        self.generic_visit(node)

    def visit_arg(self, node: ast.arg) -> None:  # type: ignore[override]
        if node.arg:
            self._add(node.arg)
        self.generic_visit(node)

    def visit_alias(self, node: ast.alias) -> None:  # type: ignore[override]
        if node.asname:
            self._add(node.asname)
        if node.name:
            self._add(node.name)
        self.generic_visit(node)


def split_words_from_identifiers(counts: Counter) -> Counter:
    """
    - os.path.join -> os path join
    - __init__ -> init
    - _test_task -> test task
    - camelCase / ZoneInfo -> camel Case / Zone Info
    同一个“词”如果有不同大小写形式（如 zoneinfo / ZoneInfo），优先保留包含大写的形式。
    """
    word_counter: Counter = Counter()
    # 先按原始 token 统计
    raw_word_map: Dict[str, int] = {}
    for name, count in counts.items():
        for token in explode_identifier(name):
            raw_word_map[token] = raw_word_map.get(token, 0) + count

    # 规范化大小写，合并 zoneinfo / ZoneInfo，优先带大写
    chosen_form: Dict[str, str] = {}
    total_counts: Dict[str, int] = {}
    for token, cnt in raw_word_map.items():
        norm = token.lower()
        existing = chosen_form.get(norm)
        if existing is None:
            chosen_form[norm] = token
        else:
            # 如果当前 token 含有大写且已有的不含大写，则替换
            if any(ch.isupper() for ch in token) and not any(
                ch.isupper() for ch in existing
            ):
                chosen_form[norm] = token
        total_counts[norm] = total_counts.get(norm, 0) + cnt

    for norm, total in total_counts.items():
        form = chosen_form[norm]
        word_counter[form] = total

    return word_counter


def explode_identifier(name: str) -> Iterable[str]:
    """
    将标识符拆成单词：
    - 按 "." 分段
    - 去掉前后下划线，再按 "_" 分段
    - 对每一段按 CamelCase 再拆
    """
    # os.path.join -> ["os", "path", "join"]
    for dotted_part in name.split("."):
        part = dotted_part.strip("_")
        if not part:
            continue
        # _test_task -> ["test", "task"]
        for underscore_part in part.split("_"):
            seg = underscore_part.strip("_")
            if not seg:
                continue
            # ZoneInfo -> ["Zone", "Info"]
            camel_parts = CAMEL_CASE_BOUNDARY_RE.split(seg)
            for c in camel_parts:
                c = c.strip("_")
                if c:
                    yield c

