# Coverage contract

`scripts/test_ubuntu.py --profile coverage --output /tmp/litesim-quality --jobs 2`
runs the normal Ubuntu 22.04 / ROS 2 Humble build and quality workflow. Source
ROS Humble first. Use the runner's evidence directory from `latest.json` when
inspecting results; a failed or blocked stage is not a passing full workflow.

## Reproduce unit-only coverage

```bash
source /opt/ros/humble/setup.bash
python3 scripts/test_ubuntu.py --profile coverage --unit-only \
  --output /tmp/litesim-unit-coverage --jobs 2
```

`--unit-only` is valid **only** with `--profile coverage`. It builds and
instruments every normal runtime/GUI target, with the same source inventory,
static baseline and thresholds. It runs the explicit `native()` and
`python_units()` stages and the real coverage collector. Pure in-process
offscreen widget units remain included; physical-renderer, Xvfb/D-Bus and
launched-peer integration workflows are not started. Only their presence checks
are omitted; GUI compilation is **not** disabled.

Its tooling preflight runs `test_quality.py` and the sanitizer-diagnostic,
packaging-command and JUnit-encoding contract regressions, with the
existing unrelated ASan/LSan ELF control explicitly out of scope. No sanitizer detector is disabled
or reported as passing. The default/full runner still selects the entire
tooling suite, sanitizer controls and integration workflows unchanged.

`result.json`, `latest.json` and `summary.json` record `scope: "unit-only"` and
use `UNIT_ONLY_PASS` / `UNIT_ONLY_FAIL`, not a full-matrix `PASS`. Integration,
graphical integration, sanitizer controls, release packaging and the full
quality matrix are explicitly recorded as `NOT_RUN`. A successful unit-only
result does not close separately reported full-profile environment blockers.
Source-snapshot consistency and strict >90% gates remain mandatory.

The real Config/Model/Simulator/sensor CLI pipeline cases are retained as
`core_cli_integration_tests` in the full runner's integration stage. Only the
separate boundary-injected `core_cli_unit_tests` are counted as units; calling a
whole pipeline's entry point in-process does not by itself make it a unit test.
The real 140-cycle Config/Model/Simulator/sensor-worker `lifecycle_probe` stress
subprocess is likewise integration-only. Full profiles retain that check,
including TSAN; unit-only coverage neither runs it nor uses its profiles. Its
compiled executable and runtime source mappings remain in the static baseline.

## Gates and scope

- **Unit-only aggregate lines, branches and functions must each be strictly
  greater than 90%.** Exactly 90% fails. Gates compare integer numerators and
  denominators, not rounded display percentages or averages of file percentages.
- Combined unit + integration coverage is reported and gated separately. It
  never supplies hits to the unit-only result.
- The six existing critical core implementation files additionally require
  greater than 90% for each nonempty metric.
- `inventory.py` still discovers the full installed runtime and example source
  inventory, including GUI code. This change adds no coverage exclusions.
- Every compiled native mapping enters a zero-hit baseline, even when its
  executable/plugin was never run. Missing source mappings and uninstrumented
  runtime translation units fail validation.

## Counting native code

Lines are LLVM executable source-line locations. Branches are source-condition
outcomes. Functions are canonical definition-body ranges. Template
instantiations, constructor ABI variants, duplicated DSOs and installed copies
of a header are unioned at their source definition, not counted repeatedly.
Uninstantiated inline/template bodies with no LLVM mapping cannot be measured.

LLVM14 can infer fallthrough counts after an opaque call throws. These source
metrics therefore do not prove that every semantic path or external transport
was exercised, even when a file reports 100%. Recording-spy assertions provide
separate behavioral evidence; real ROS delivery and physical rendering are
never certified merely by a source-coverage percentage.

The coverage build explicitly emits ELF build IDs. LLVM14's `%m` is a nonunique
layout signature (and `%b` is unsupported), so it can also collide between two
different DSOs in one process. One **uninstrumented** quality-only initializer
per ELF target calls compiler-rt's module-local filename API to prefix that
target's output. It adds no measured functions or counters. CMake emits
`litesim-coverage-<target>.json` beside the build output to record the expected
target/prefix association. Reconfigure/relink and collect fresh profiles when
upgrading to this contract; legacy unqualified profiles are rejected even if
they retain one plausible binary ID after a collision.

Before merging, the collector
matches each raw profile's embedded binary ID to its exact executable or DSO.
Each object is exported independently, then source locations are unioned. A
global merge is unsafe: two different executables can have identical `main`
names and control-flow hashes, falsely covering one with the other's counters.
Profiles with absent/ambiguous IDs, missing target prefixes or ambiguous module
manifests fail closed. Attribution is saved
as `coverage/{unit,combined}-native-attribution.json`; per-object raw LLVM exports
and indexed profiles are retained in `coverage/native/`.

## Counting Python code

Coverage.py measures executable statements and branch destinations, including
unexecuted inventoried files. The opt-in subprocess startup hook separately
records actual function-body entries. Executing a `def` statement, constructing
an inner function, or closing an unstarted coroutine is not a covered call.
New Python threads inherit the entry profiler.

The static denominator includes declared functions, methods and lambdas in every
inventoried source, even if it is never imported or the compiler drops/coalesces
its body. Class/module bodies and generated comprehensions are not functions.
Stable code-tree paths distinguish same-line callables where possible. Python
3.10 can share one code object for identical same-line lambdas; such aliases
retain every definition in the denominator but receive at most one covered
body, a conservative lower bound rather than invented hits. Installed Python
copies are mapped back to their original source identity.

Per-process entries are retained as `profiles/{unit,integration}/python/.functions.*.json`.
Missing entry profiles, unsupported profile versions and stale source identities
fail closed. Coverage processes must use the runner's startup hook and both
`COVERAGE_PROCESS_START` and `LITESIM_FUNCTION_COVERAGE_ROOTS` settings.

## Results and regression controls

`coverage/unit-coverage.json` and `combined-coverage.json` contain file, package
and weighted aggregate counts. `coverage-gate.json` contains thresholds, scope,
failures and final status. `summary.md` displays **unit-only** package metrics.

The collector's regression tests include an actual LLVM14 two-executable
same-`main` collision, zero-hit denominator preservation, exact-90 rejection,
installed-source subprocess aliases, unit/integration separation, never-called
definitions, compiler-coalesced lambdas, unstarted generators/coroutines,
decorators, optimized-away functions and new-thread entries:

```bash
python3 -m pytest -q scripts/quality/tests/test_quality.py
```

That complete tooling module also contains an existing ASan/LSan ELF regression;
it requires the separately documented sanitizer toolchain/runtime capability.
An explicitly selected coverage-only subset is not a full quality-matrix pass.
