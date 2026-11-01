#!/usr/bin/env python3
"""Compatibility name; immediately replace this process with the native waiter."""
import os
import sys
from pathlib import Path


def main():
    executable = Path(__file__).resolve().with_name("spawn_when_ready")
    os.execv(str(executable), [str(executable), *sys.argv[1:]])


if __name__ == "__main__":
    main()
