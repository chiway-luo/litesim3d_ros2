#!/usr/bin/env python3
"""Check clone-local documentation without requiring historical test artifacts."""
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[2]


def main():
    documents = [ROOT / 'README.md']
    for folder in ('docs', 'examples', 'litesim3d_core/resources'):
        documents.extend(sorted((ROOT / folder).rglob('*.md')))
    failures = []
    links = blocks = 0
    for document in documents:
        relative = document.relative_to(ROOT).as_posix()
        content = document.read_text()
        for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)', content):
            if re.match(r'\w+://', target) or target.startswith(('#', 'mailto:')):
                continue
            local = unquote(target.split('#')[0].strip('<>'))
            if not local:
                continue
            links += 1
            destination = (document.parent / local).resolve()
            if not destination.is_relative_to(ROOT) or not destination.exists():
                failures.append(f'{relative}: unavailable in clone: {local}')
        for language, code in re.findall(r'^```(bash|sh|zsh)\n(.*?)^```', content, re.M | re.S):
            blocks += 1
            result = subprocess.run(['bash', '-n'], input=code, text=True, capture_output=True)
            if result.returncode:
                failures.append(f'{relative}: invalid shell block: {result.stderr.strip()}')
        if relative == 'README.md' or relative.startswith(('docs/usage/', 'examples/')):
            if '/Users/' in content or re.search(r'\bros2h\b', content):
                failures.append(f'{relative}: personal path or shell alias in usage instructions')
    result = {'status': 'FAIL' if failures else 'PASS', 'documents': len(documents),
              'local_links': links, 'shell_blocks': blocks, 'failures': failures,
              'scope': 'Repository-local links and shell syntax only; commands are not executed.'}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
