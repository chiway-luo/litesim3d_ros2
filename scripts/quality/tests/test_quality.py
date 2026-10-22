import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import unittest
from argparse import Namespace

QUALITY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(QUALITY))
from audit import canonical_source, compilation_errors, has_function_body
from check_lifecycle import evaluate
from check_sanitizers import findings, suppression_usage
from collect_coverage import counts, gate, native_metrics, python_report, fixed_denominators
from inventory import inventory, python_aliases, coverage_config
from sanitizer_control import asan_options, asan_link_options


class QualityTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name) / "source"
        self.root.mkdir()

    def write(self, path, content):
        destination = self.root / path
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(content)
        return destination

    def package(self, name="new_plugin"):
        self.write(f"{name}/package.xml", f"<package><name>{name}</name></package>")

    def test_new_package_and_unexecuted_sources_enter_denominator(self):
        self.package()
        self.write("new_plugin/src/runtime.cpp", "int answer(){return 42;}")
        self.write("new_plugin/scripts/control", "#!/usr/bin/python3\nraise ValueError()\n")
        self.write("new_plugin/test/test.cpp", "int main(){}")
        sources = inventory(self.root)
        self.assertEqual(len(sources["sources"]), 2)
        _, failures = gate(sources, {})
        self.assertTrue(any("runtime.cpp" in failure for failure in failures))
        self.assertTrue(any("scripts/control" in failure for failure in failures))
        self.assertTrue(any("Total lines" in failure for failure in failures))

    def test_ignored_external_package_is_explicitly_recorded(self):
        self.package()
        self.write("new_plugin/COLCON_IGNORE", "")
        self.write("new_plugin/src/runtime.cpp", "int answer(){return 42;}")
        item = inventory(self.root)
        self.assertEqual(item["sources"], {})
        self.assertIn("new_plugin/", item["exclusions"])

    def test_installed_headers_and_python_aliases(self):
        self.package()
        header = "new_plugin/include/new_plugin/api.hpp"
        script = "new_plugin/launch/robot.launch.py"
        self.write(header, "int answer(){return 42;}")
        self.write(script, "print(42)")
        prefix = self.root.parent / "install"
        item = inventory(self.root)
        self.assertEqual(canonical_source(prefix / "include/new_plugin/api.hpp", self.root, prefix, item), header)
        self.assertIn(str(prefix / "share/new_plugin/launch/robot.launch.py"), python_aliases(self.root, prefix, item)[script])
        self.assertIsNone(canonical_source("/usr/include/vector", self.root, prefix, item))
        module = "new_plugin/new_plugin/model.py"
        self.write(module, "answer = 42\n")
        installed = prefix / "local/lib/python3.10/dist-packages/new_plugin/model.py"
        installed.parent.mkdir(parents=True)
        installed.write_text("answer = 42\n")
        self.assertIn(str(installed), python_aliases(self.root, prefix, inventory(self.root))[module])

    def test_all_compilations_must_be_instrumented(self):
        self.package()
        source = self.write("new_plugin/src/runtime.cpp", "int answer(){return 42;}")
        build = self.root.parent / "build/new_plugin"
        build.mkdir(parents=True)
        item = inventory(self.root)
        self.assertTrue(compilation_errors(self.root, build.parent, item, "-fcoverage-mapping"))
        command = {"file": str(source), "directory": str(build), "command": "clang++ -fcoverage-mapping -c " + str(source)}
        database = build / "compile_commands.json"
        database.write_text(json.dumps([command]))
        self.assertFalse(compilation_errors(self.root, build.parent, item, "-fcoverage-mapping"))
        database.write_text(json.dumps([command, {**command, "command": "clang++ -c file"}]))
        self.assertTrue(compilation_errors(self.root, build.parent, item, "-fcoverage-mapping"))

    def test_native_union_does_not_double_count_installed_headers(self):
        lcov = "SF:/src/api.hpp\nDA:1,2\nDA:2,0\nend_of_record\nSF:/install/api.hpp\nDA:1,3\nDA:2,1\nend_of_record\nSF:/src/missed.cpp\nDA:1,0\nend_of_record"
        exported = {"data": [{"files": [
            {"filename": "/src/api.hpp", "branches": [[1, 3, 1, 9, 5, 0, 0, 0, 4]]},
            {"filename": "/install/api.hpp", "branches": [[1, 3, 1, 9, 0, 5, 0, 0, 4]]}]}]}
        result = native_metrics(lcov, exported, lambda path: Path(path).name)
        self.assertEqual(result["api.hpp"]["lines"], counts(2, 2))
        self.assertEqual(result["api.hpp"]["branches"], counts(2, 2))
        self.assertEqual(result["missed.cpp"]["lines"], counts(0, 1))

    def test_total_and_critical_thresholds_reject_low_coverage(self):
        path = "litesim3d_core/src/geometry.cpp"
        item = {"sources": {path: {"declarations_only": False}}}
        _, failures = gate(item, {path: {"lines": counts(80, 100), "branches": counts(70, 100)}})
        self.assertEqual(len(failures), 2)
        self.assertTrue(all(path in failure for failure in failures))
        _, failures = gate(item, {path: {"lines": counts(79, 100), "branches": counts(69, 100)}})
        self.assertEqual(len(failures), 4)

    def test_declaration_exemption_cannot_hide_inline_function(self):
        self.assertFalse(has_function_body("struct A { int value=1; void run(); }; // void fake() {}"))
        self.assertTrue(has_function_body("bool ready() const noexcept { return true; }"))

    def test_unit_gate_cannot_borrow_integration_hits(self):
        path = "litesim3d_core/src/geometry.cpp"
        item = {"sources": {path: {"declarations_only": False}}}
        _, failures = gate(item, {path: {"lines": counts(80, 100), "branches": counts(70, 100)}}, unit=True)
        self.assertEqual(len(failures), 2)
        _, failures = gate(item, {path: {"lines": counts(0, 0), "branches": counts(0, 0)}}, unit=True)
        self.assertEqual(failures, ["No executable coverage mapping: " + path])

    def test_missing_function_profile_cannot_shrink_denominator(self):
        observed = {"main.cpp": {"lines": counts(1, 1), "branches": counts(0, 0)}}
        baseline = {"main.cpp": {"lines": counts(0, 47), "branches": counts(0, 26)},
                    "missed.cpp": {"lines": counts(0, 8), "branches": counts(0, 2)}}
        fixed = fixed_denominators(observed, baseline)
        self.assertEqual(fixed["main.cpp"]["lines"], counts(1, 47))
        self.assertEqual(fixed["missed.cpp"]["lines"], counts(0, 8))
        with self.assertRaises(RuntimeError):
            fixed_denominators({"unknown.cpp": observed["main.cpp"]}, baseline)

    def test_lifecycle_requires_same_pid_and_all_cycles(self):
        samples = [{"cycle": i, "pid": 10, "rss_bytes": 1000000, "fds": 5, "threads": 1} for i in range(140)]
        self.assertEqual(evaluate(samples)[0], [])
        self.assertTrue(evaluate(samples[:-1])[0])
        samples[-1]["pid"] = 20
        self.assertTrue(evaluate(samples)[0])

    def test_rss_growth_and_threads_are_checked_in_child(self):
        samples = [{"cycle": i, "pid": 10, "rss_bytes": i * 1024 * 1024, "fds": 5, "threads": 1} for i in range(140)]
        self.assertTrue(evaluate(samples)[0])
        self.assertFalse(evaluate(samples, instrumented=True)[0])
        samples[-1]["threads"] = 10
        self.assertTrue(evaluate(samples, instrumented=True)[0])

    def test_child_sanitizer_report_fails_even_without_parent_failure(self):
        child = self.write("logs/node.log", "all good\n==40==ERROR: LeakSanitizer: detected memory leaks\n")
        detected = findings(self.root)
        self.assertEqual(detected[0]["file"], str(child))
        self.assertEqual(detected[0]["line"], 2)
        self.write("positive-control/leak.log", "ERROR: LeakSanitizer: expected fault\n")
        self.assertEqual(len(findings(self.root)), 1)
        self.write("ubsan.45", "file.cpp:14: runtime error: overflow\n")
        self.assertEqual(len(findings(self.root)), 2)

    def test_dependency_exceptions_remain_visible_alongside_real_findings(self):
        self.write("ubsan.48", "Suppressions used:\n  7 352 pluginlib::ClassLoader<litesim3d::plugins::SensorPlugin>::loadLibraryForClass\n"
                   "  1 256 FcInit\nERROR: LeakSanitizer: another leak\n")
        usage = suppression_usage(self.root)
        self.assertEqual(sum(row["bytes"] for row in usage), 608)
        self.assertEqual(len(findings(self.root)), 1)

    def test_ci_uses_the_same_five_profiles(self):
        workflow = (QUALITY.parents[1] / ".github/workflows/m1.yml").read_text()
        self.assertIn("profile: [functional, coverage, asan, tsan, release]", workflow)
        self.assertIn("python3 scripts/test_ubuntu.py --profile", workflow)
        self.assertNotIn("bash scripts/test_m3.sh", workflow)

    def test_asan_executable_mapping_keeps_plugins_shared(self):
        compiler = shutil.which("clang++-14")
        if platform.system() != "Linux" or not compiler or not shutil.which("cmake"):
            self.skipTest("Linux with Clang 14 and CMake is required for the ELF regression")
        self.write("CMakeLists.txt", "cmake_minimum_required(VERSION 3.20)\n"
                   "project(litesim3d_gui LANGUAGES CXX)\n"
                   "add_library(plugin SHARED plugin.cpp)\n"
                   "add_executable(probe main.cpp)\n"
                   "target_link_libraries(probe PRIVATE plugin ${CMAKE_DL_LIBS})\n")
        self.write("plugin.cpp", "extern \"C\" int answer(){return 42;}\n")
        self.write("main.cpp", "#include <dlfcn.h>\n#include <unwind.h>\n#include <cstdio>\n"
                   "extern \"C\" int answer(); int main(){\n"
                   "Dl_info info{}; if(!dladdr(reinterpret_cast<void*>(&_Unwind_Backtrace), &info)) return 2;\n"
                   "std::puts(info.dli_fname); return answer()!=42;}\n")
        # Exercise an unpacked tool prefix, without replacing the machine's
        # libunwind packages (which may be required by Nav2/SLAM dependencies).
        library = Path(next(flag[2:] for flag in asan_link_options(True) if flag.startswith("-L")))
        isolated = self.root / "unwind"
        isolated.mkdir()
        for name in ("libunwind.so", "libunwind.so.1"):
            (isolated / name).symlink_to((library / name).resolve())
        env = dict(os.environ, LITESIM_LLVM_UNWIND_DIR=str(isolated),
                   LD_LIBRARY_PATH=str(isolated) + os.pathsep + os.environ.get("LD_LIBRARY_PATH", ""))
        build = self.root.parent / "build"
        for command in (["cmake", "-S", str(self.root), "-B", str(build),
                         f"-DCMAKE_CXX_COMPILER={compiler}", "-DLITESIM_QUALITY_MODE=asan",
                         f"-DCMAKE_PROJECT_INCLUDE={QUALITY / 'instrument.cmake'}"],
                        ["cmake", "--build", str(build), "--parallel", "1"]):
            result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f"-L{isolated}", (build / "CMakeFiles/probe.dir/link.txt").read_text())
        def elf_type(path):
            header = path.read_bytes()[:20]
            self.assertEqual(header[:4], b"\x7fELF")
            return int.from_bytes(header[16:18], "little" if header[5] == 1 else "big")
        self.assertEqual(elf_type(build / "libplugin.so"), 3)  # ET_DYN: still a shared plugin.
        if platform.machine().lower() in ("x86_64", "amd64"):
            self.assertEqual(elf_type(build / "probe"), 2)  # ET_EXEC: cannot overlap ASan's allocator.
        env["ASAN_OPTIONS"] = asan_options(shutil.which("llvm-symbolizer-14") or "llvm-symbolizer")
        result = subprocess.run([str(build / "probe")], env=env, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("libunwind.so.1", result.stdout)  # LLVM unwinds malloc stacks, not libgcc_s.

    def test_python_subprocess_files_merge_and_unexecuted_file_is_counted(self):
        try:
            import coverage
        except ImportError:
            self.skipTest("python3-coverage is required and installed in the Ubuntu quality environment")
        self.package()
        used = self.write("new_plugin/new_plugin/used.py", "def choose(value):\n    if value:\n        return 1\n    return 2\n")
        missed = self.write("new_plugin/new_plugin/missed.py", "def unused():\n    return 3\n")
        prefix = self.root.parent / "install"
        installed = prefix / "local/lib/python3.10/dist-packages/new_plugin/used.py"
        installed.parent.mkdir(parents=True)
        installed.write_text(used.read_text())
        evidence = self.root.parent / "evidence"
        raw = evidence / "profiles/unit/python"
        raw.mkdir(parents=True)
        cov = coverage.Coverage(config_file=False, data_file=str(raw / ".coverage"), data_suffix=True, branch=True)
        namespace = {}
        cov.start()
        exec(compile(installed.read_text(), str(installed), "exec"), namespace)
        self.assertEqual(namespace["choose"](True), 1)
        cov.stop()
        cov.save()
        (evidence / "coveragerc").write_text(coverage_config(self.root, prefix, inventory(self.root)))
        report = evidence / "coverage"
        report.mkdir()
        result = python_report(Namespace(root=self.root, prefix=prefix, evidence=evidence), "unit", inventory(self.root), report)
        self.assertGreater(result[used.relative_to(self.root).as_posix()]["lines"]["covered"], 0)
        self.assertEqual(result[missed.relative_to(self.root).as_posix()]["lines"], counts(0, 2))


if __name__ == "__main__":
    unittest.main()
