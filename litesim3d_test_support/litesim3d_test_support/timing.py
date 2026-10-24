"""Simulation deadlines with a separate, bounded wall-clock liveness guard."""
import time


def until_simulated(predicate, simulation_time, spin, seconds, *, wall_timeout=45):
    beginning = simulation_time()
    deadline = time.monotonic() + wall_timeout
    while True:
        elapsed = simulation_time() - beginning
        if elapsed < 0:
            raise AssertionError("simulation clock moved backwards")
        if elapsed > seconds:
            raise AssertionError(f"condition missed simulation deadline: {elapsed:.3f}s > {seconds}s")
        # Require a fresh state, not a stale zero-velocity sample from startup.
        if elapsed > 0 and predicate():
            return elapsed
        if time.monotonic() >= deadline:
            raise AssertionError(f"simulation made insufficient progress in {wall_timeout}s: {elapsed:.3f}s")
        spin(.02)
