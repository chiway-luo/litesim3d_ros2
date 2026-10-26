#!/usr/bin/env python3
"""Check clone-local documentation without requiring historical test artifacts."""
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[2]

def prose(content):
    """Exclude fenced examples from link and heading inspection."""
    return re.sub(r"^(`{3,}|~{3,})[^\n]*\n.*?^\1[ \t]*$", "", content,
                  flags=re.M | re.S)


def anchors(content):
    body = prose(content)
    result = set(re.findall(r'<(?:a|span)\s+id=["\']([^"\']+)["\']', body))
    seen = {}
    for heading in re.findall(r"^#{1,6}\s+(.+?)\s*#*\s*$", body, re.M):
        heading = re.sub(r"\[([^]]+)\]\([^)]+\)", r"\1", heading)
        slug = "".join(c for c in heading.lower() if c.isalnum() or c in "_- ").replace(" ", "-")
        count = seen.get(slug, 0)
        seen[slug] = count + 1
        result.add(slug + (f"-{count}" if count else ""))
    return result


def check(root=ROOT):
    documents = sorted(root.glob('README*.md'))
    for folder in ('docs', 'examples', 'plugins', 'litesim3d_core/resources'):
        documents.extend(sorted((root / folder).rglob('*.md')))
    documents.extend(sorted(root.glob("litesim3d_*/README*.md")))
    documents = sorted(set(documents))
    failures = []
    links = blocks = fragments = 0
    for document in documents:
        relative = document.relative_to(root).as_posix()
        content = document.read_text()
        body = prose(content)
        targets = re.findall(r'\[[^\]]*\]\(([^)]+)\)', body)
        # README headers use HTML for centered logos and language links.
        targets.extend(re.findall(r"""(?:href|src)\s*=\s*["']([^"']+)["']""", body))
        for target in targets:
            if re.match(r'\w+://', target) or target.startswith('mailto:'):
                continue
            local, _, fragment = unquote(target.strip('<>')).partition('#')
            links += 1
            destination = (document.parent / local).resolve() if local else document
            if not destination.is_relative_to(root) or not destination.exists():
                failures.append(f'{relative}: unavailable in clone: {local}')
            elif destination.relative_to(root).as_posix().startswith("tmp/"):
                failures.append(f'{relative}: local development material in public documentation: {local}')
            elif fragment and destination.suffix == ".md":
                fragments += 1
                if fragment not in anchors(destination.read_text()):
                    failures.append(f"{relative}: missing heading/anchor: {target}")
        for language, code in re.findall(r'^```(bash|sh|zsh)\n(.*?)^```', content, re.M | re.S):
            blocks += 1
            result = subprocess.run(['bash', '-n'], input=code, text=True, capture_output=True)
            if result.returncode:
                failures.append(f'{relative}: invalid shell block: {result.stderr.strip()}')
        if document.parent == root or relative.startswith(('docs/usage/', 'docs/tutorials/', 'docs/api/', 'examples/')):
            if '/Users/' in content or re.search(r'\bros2h\b', content):
                failures.append(f'{relative}: personal path or shell alias in usage instructions')
        if relative.startswith(("docs/usage/", "docs/tutorials/", "docs/api/", "docs/concepts/")):
            if len(re.findall(r"^# ", body, re.M)) != 1:
                failures.append(f"{relative}: expected exactly one page title")
        if relative.startswith(("docs/tutorials/", "docs/usage/")) and document.name != "兼容配置.md":
            if re.search(r"\bm[23](?:_robot|_world)?\.yaml\b", content):
                failures.append(f"{relative}: historical configuration belongs in compatibility appendix")
    result = {'status': 'FAIL' if failures else 'PASS', 'documents': len(documents),
              'local_links': links, 'anchor_links': fragments, 'shell_blocks': blocks, 'failures': failures,
              'scope': 'Local links/anchors, page titles, reader-facing paths and shell syntax; commands not executed.'}
    return result


def main():
    result = check()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return int(bool(result["failures"]))


if __name__ == '__main__':
    raise SystemExit(main())
