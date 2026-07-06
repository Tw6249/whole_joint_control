from __future__ import annotations

import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ANALYSIS_ROOT = ROOT / "analysis_artifacts"
REPORT_ROOT = ROOT / "docs" / "reports" / "analysis"


def _absolute(path: Path) -> Path:
    return path.resolve() if path.is_absolute() else (ROOT / path).resolve()


def analysis_report_dir(out_dir: Path) -> Path:
    """Map an analysis output directory to the matching docs report directory."""
    resolved = _absolute(out_dir)
    try:
        rel = resolved.relative_to(ANALYSIS_ROOT.resolve())
    except ValueError:
        rel = Path(resolved.name)
    return REPORT_ROOT / rel


def repo_relpath(path: Path) -> str:
    """Return a repo-relative path suitable for Markdown text."""
    resolved = _absolute(path)
    try:
        return resolved.relative_to(ROOT).as_posix()
    except ValueError:
        return resolved.as_posix()


def markdown_relpath(target: Path, from_dir: Path) -> str:
    """Return a POSIX relative path for a Markdown link."""
    target_abs = _absolute(target)
    from_abs = _absolute(from_dir)
    return Path(os.path.relpath(target_abs, from_abs)).as_posix()
