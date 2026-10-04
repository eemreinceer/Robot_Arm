#!/usr/bin/env python3
"""Validate local Markdown links without network access or third-party modules."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote


INLINE_LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
REFERENCE_LINK_RE = re.compile(r"^\s*\[[^\]]+\]:\s*(\S+)")
REMOTE_PREFIXES = ("http://", "https://", "mailto:", "tel:")


def tracked_markdown(repo_root: Path) -> list[Path]:
    result = subprocess.run(
        ["git", "ls-files", "*.md"],
        cwd=repo_root,
        check=True,
        text=True,
        capture_output=True,
    )
    return [repo_root / line for line in result.stdout.splitlines() if line]


def normalize_target(raw_target: str) -> str:
    target = raw_target.strip()
    if target.startswith("<") and ">" in target:
        target = target[1 : target.index(">")]
    else:
        target = target.split(maxsplit=1)[0]
    return unquote(target)


def local_targets(markdown: Path) -> list[tuple[int, str]]:
    targets: list[tuple[int, str]] = []
    in_fence = False
    for line_number, line in enumerate(markdown.read_text(encoding="utf-8").splitlines(), 1):
        stripped = line.lstrip()
        if stripped.startswith("```") or stripped.startswith("~~~"):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        for match in INLINE_LINK_RE.finditer(line):
            targets.append((line_number, normalize_target(match.group(1))))
        reference = REFERENCE_LINK_RE.match(line)
        if reference:
            targets.append((line_number, normalize_target(reference.group(1))))
    return targets


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="*", type=Path, help="Markdown files; default is all tracked files")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    markdown_files = args.files or tracked_markdown(repo_root)
    errors: list[str] = []

    for item in markdown_files:
        markdown = item if item.is_absolute() else repo_root / item
        if not markdown.is_file():
            errors.append(f"missing Markdown file: {item}")
            continue
        for line_number, target in local_targets(markdown):
            if not target or target.startswith("#") or target.startswith(REMOTE_PREFIXES):
                continue
            path_part = target.split("#", 1)[0].split("?", 1)[0]
            if not path_part:
                continue
            if path_part.startswith("/"):
                errors.append(f"{markdown.relative_to(repo_root)}:{line_number}: absolute local link: {target}")
                continue
            resolved = (markdown.parent / path_part).resolve()
            if not resolved.exists():
                errors.append(f"{markdown.relative_to(repo_root)}:{line_number}: missing target: {target}")

    if errors:
        print("Markdown link check FAILED", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print(f"Markdown link check PASS ({len(markdown_files)} files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
