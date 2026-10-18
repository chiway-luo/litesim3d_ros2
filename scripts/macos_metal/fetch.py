#!/usr/bin/env python3
"""Fetch the pinned Ogre-Next Metal sources into an explicitly supplied build cache."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import sys
import urllib.request


def main():
    destination = Path(sys.argv[1]).resolve()
    manifest = json.loads(Path(__file__).with_name("sources.json").read_text())
    base = "https://raw.githubusercontent.com/OGRECave/ogre-next/" + manifest["version"] + "/"
    def fetch(item):
        name, digest = item
        target = destination / name
        if target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest() == digest:
            return
        with urllib.request.urlopen(base + name, timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != digest:
            raise RuntimeError("Ogre source checksum mismatch: " + name)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        list(pool.map(fetch, manifest["sha256"].items()))


if __name__ == "__main__":
    main()
