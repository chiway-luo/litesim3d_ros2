"""Opt-in subprocess coverage; this directory is only added by the runner."""
import os

if os.environ.get("COVERAGE_PROCESS_START"):
    import coverage

    coverage.process_startup()

    from function_coverage import process_startup

    process_startup()
