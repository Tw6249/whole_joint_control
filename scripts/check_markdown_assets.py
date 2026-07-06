from __future__ import annotations

import argparse
import re
from pathlib import Path
from urllib.parse import unquote, urlparse


MARKDOWN_IMAGE_RE = re.compile(r"!\[[^\]]*\]\(([^)]+)\)")
HTML_IMG_RE = re.compile(r"<img\s+[^>]*src=[\"']([^\"']+)[\"']", re.IGNORECASE)


def is_local_asset(raw: str) -> bool:
    target = raw.strip().split()[0].strip("\"'")
    parsed = urlparse(target)
    return not parsed.scheme and not target.startswith("#")


def normalize_target(raw: str) -> str:
    target = raw.strip().split()[0].strip("\"'")
    target = target.split("#", 1)[0]
    return unquote(target)


def iter_asset_refs(markdown_path: Path):
    text = markdown_path.read_text(encoding="utf-8", errors="ignore")
    for pattern in (MARKDOWN_IMAGE_RE, HTML_IMG_RE):
        for match in pattern.finditer(text):
            raw = match.group(1)
            if is_local_asset(raw):
                yield match.start(1), normalize_target(raw)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check local Markdown image references.")
    parser.add_argument("paths", nargs="*", default=["."], help="Files or directories to scan.")
    args = parser.parse_args()

    markdown_files: list[Path] = []
    for raw_path in args.paths:
        path = Path(raw_path)
        if path.is_dir():
            markdown_files.extend(path.rglob("*.md"))
        elif path.suffix.lower() == ".md":
            markdown_files.append(path)

    missing: list[tuple[Path, str]] = []
    for markdown_path in sorted(set(markdown_files)):
        for _, target in iter_asset_refs(markdown_path):
            if not target:
                continue
            asset_path = (markdown_path.parent / target).resolve()
            if not asset_path.exists():
                missing.append((markdown_path, target))

    if missing:
        print("Missing local Markdown assets:")
        for markdown_path, target in missing:
            print(f"- {markdown_path}: {target}")
        return 1

    print(f"Checked {len(set(markdown_files))} Markdown files; all local image assets exist.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
