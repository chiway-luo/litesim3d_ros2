#!/usr/bin/env python3
"""Locate source and external artifacts for a workspace or standalone checkout."""
import argparse
import os
from pathlib import Path
import re


REPO_ROOT = Path(__file__).resolve().parents[1]
DEVELOPMENT_DOCS_DIR = REPO_ROOT / "tmp" / "development-docs"
IN_WORKSPACE = REPO_ROOT.parent.name == "src"
WORKSPACE_ROOT = REPO_ROOT.parent.parent if IN_WORKSPACE else REPO_ROOT.parent
ARTIFACTS_DIR = Path(os.environ.get(
    "LITESIM3D_ARTIFACTS_DIR",
    str(WORKSPACE_ROOT / "artifacts" if IN_WORKSPACE
        else REPO_ROOT.with_name(REPO_ROOT.name + "-artifacts")),
)).expanduser().resolve()
EVIDENCE_DIR = ARTIFACTS_DIR / "evidence"
SOURCE_FOLDERS = ("litesim3d_core", "litesim3d_interfaces", "litesim3d_ros2",
                  "examples", "scripts", "standalone")


def relocated_document(path):
    """Resolve historical document references into the local development archive."""
    path = Path(path)
    if path.is_relative_to(REPO_ROOT):
        archived = DEVELOPMENT_DOCS_DIR / path.relative_to(REPO_ROOT)
        if archived.is_file():
            return archived
    if path.parent == REPO_ROOT / "docs" and path.suffix == ".md":
        matches = [relocated_document(REPO_ROOT / "docs" / folder / path.name)
                   for folder in ("usage", "testing", "design")]
        return next((candidate for candidate in matches if candidate.is_file()), path)
    return path


def resolve_reference(value):
    """Resolve logical evidence references retained in historical JSON indexes."""
    path = Path(value)
    if path.is_absolute():
        return path
    if path.parts and path.parts[0] == "evidence":
        return EVIDENCE_DIR.joinpath(*path.parts[1:])
    return relocated_document(REPO_ROOT / path)


def reference(path):
    """Store portable source paths or logical evidence/<name> references."""
    path = Path(path).resolve()
    if path.is_relative_to(EVIDENCE_DIR):
        return "evidence/" + path.relative_to(EVIDENCE_DIR).as_posix()
    if path.is_relative_to(REPO_ROOT):
        return path.relative_to(REPO_ROOT).as_posix()
    return str(path)


def source_files():
    for folder in SOURCE_FOLDERS:
        for path in sorted((REPO_ROOT / folder).rglob("*")):
            if path.is_file() and not {"__pycache__", ".pytest_cache"}.intersection(path.parts):
                yield path


def python_sources():
    return [path for path in source_files()
            if path.suffix == ".py" or path.name == "sim_session"]


def write_report(path, text):
    """Render legacy docs-root templates as portable, relocated reports.

    Evidence stays a logical identifier: a fresh clone has no historical logs.
    Source/document links remain clickable from the new report directory.
    """
    path = Path(path).resolve()
    if path.is_relative_to(REPO_ROOT / "docs/testing"):
        path = DEVELOPMENT_DOCS_DIR / path.relative_to(REPO_ROOT)
    path.parent.mkdir(parents=True, exist_ok=True)

    def rewrite(match):
        label, target = match.groups()
        if re.match(r"\w+://", target) or target.startswith(("#", "mailto:")):
            return match.group(0)
        local, separator, anchor = target.partition("#")
        if local.startswith("../evidence/"):
            return f"{label}（`{local[3:]}`）"
        destination = relocated_document((REPO_ROOT / "docs" / local).resolve())
        relative = os.path.relpath(destination, path.parent)
        return f"[{label}]({relative}{separator}{anchor})"

    text = re.sub(r"\[([^\]]*)\]\(([^)]+)\)", rewrite, text)
    note = ("\n> 证据审计报告：结论仅对应文中源码指纹与实际执行记录。"
            "原始 `evidence/...` 不随仓库提交，见 [测试入口](README.md)。\n")
    heading, separator, body = text.partition("\n")
    path.write_text(heading + separator + note + body)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=["repo", "workspace", "artifacts", "evidence", "install"])
    args = parser.parse_args()
    if args.kind == "install":
        record = EVIDENCE_DIR / "install-prefix.txt"
        if not record.is_file():
            parser.error("No install prefix recorded; run bash scripts/build.sh first.")
        install = Path(record.read_text().strip())
        if not (install / "setup.bash").is_file():
            parser.error("Recorded install prefix is unavailable; rebuild with scripts/build.sh.")
        print(install)
    else:
        print({"repo": REPO_ROOT, "workspace": WORKSPACE_ROOT,
               "artifacts": ARTIFACTS_DIR, "evidence": EVIDENCE_DIR}[args.kind])


if __name__ == "__main__":
    main()
