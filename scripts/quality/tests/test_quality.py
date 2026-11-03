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
from collect_coverage import counts, gate, native_metrics, native_report, profile_binary_id, python_functions, python_report, fixed_denominators
from inventory import inventory, python_aliases, coverage_config
from python_startup.function_coverage import FunctionCoverage, source_functions
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
        self.assertEqual(result["missed.cpp"]["functions"], counts(0, 0))

    def test_native_functions_union_body_locations_not_abi_names(self):
        def function(filename, name, region, hits):
            return {"filenames": [filename], "name": name, "count": hits,
                    "regions": [[*region, hits, 0, 0, 0]]}
        exported = {"data": [{"files": [], "functions": [
            function("/src/api.hpp", "template<int>", [1, 8, 3, 2], 0),
            function("/install/api.hpp", "template<float>", [1, 8, 3, 2], 4),
            function("/src/api.hpp", "C1", [5, 10, 5, 12], 0),
            function("/install/api.hpp", "C2", [5, 10, 5, 12], 1),
            function("/src/api.hpp", "unused_overload", [5, 30, 5, 32], 0),
            function("/dependency/header.hpp", "dependency", [1, 8, 3, 2], 8)]}]}
        result = native_metrics("SF:/src/api.hpp\nDA:1,4\n", exported,
                                lambda path: None if "dependency" in path else Path(path).name)
        self.assertEqual(result["api.hpp"]["functions"], counts(2, 3))

    @unittest.skipUnless(shutil.which("clang++-14") and shutil.which("llvm-profdata-14")
                         and shutil.which("llvm-cov-14"), "LLVM14 toolchain required")
    def test_native_identical_main_cannot_borrow_another_executables_hits(self):
        self.package()
        evidence = self.root.parent / "evidence"
        raw = evidence / "profiles/unit"
        raw.mkdir(parents=True)
        report = evidence / "coverage"
        report.mkdir()
        binaries = []
        # Deliberately identical names AND control-flow hashes, but different
        # definition files/build IDs. Only the first executable is ever run.
        for name in ("first", "second"):
            source = self.write(f"new_plugin/src/{name}.cpp", "int main(){return 0;}\n")
            binary = self.root.parent / name
            helper = self.root.parent / f"{name}-filename.o"
            prefix = f"native_control--{name}--"
            subprocess.run([shutil.which("clang++-14"), "-c", str(QUALITY / "profile_filename.cpp"),
                f'-DLITESIM_COVERAGE_MODULE="{prefix}"', "-fno-profile-instr-generate",
                "-fno-coverage-mapping", "-o", str(helper)], check=True)
            subprocess.run([shutil.which("clang++-14"), "-O1", "-g",
                "-fprofile-instr-generate", "-fcoverage-mapping", "-Wl,--build-id=sha1",
                str(source), str(helper), "-o", str(binary)], check=True)
            (binary.parent / f"litesim-coverage-{name}.json").write_text(json.dumps({
                "binary": str(binary), "filename_prefix": prefix}))
            binaries.append(binary)
        subprocess.run([str(binaries[0])], check=True,
                       env=dict(os.environ, LLVM_PROFILE_FILE=str(raw / "%m-%p.profraw")))
        args = Namespace(root=self.root, prefix=self.root.parent / "install", evidence=evidence)
        baseline = native_report(args, "baseline", binaries, report)
        observed = native_report(args, "unit", binaries, report)
        fixed = fixed_denominators(observed, baseline)
        self.assertEqual(fixed["new_plugin/src/first.cpp"]["functions"], counts(1, 1))
        self.assertEqual(fixed["new_plugin/src/second.cpp"]["functions"], counts(0, 1))
        self.assertGreater(fixed["new_plugin/src/second.cpp"]["lines"]["count"], 0)
        self.assertEqual(fixed["new_plugin/src/second.cpp"]["lines"]["covered"], 0)
        attribution = json.loads((report / "unit-native-attribution.json").read_text())
        self.assertTrue(attribution[str(binaries[0])]["raw_profiles"])
        self.assertEqual(attribution[str(binaries[1])]["raw_profiles"], [])
        # A legacy collision profile may carry one plausible ID. Filename
        # qualification is still required, so renaming cannot hide that gap.
        qualified = next(raw.glob("*.profraw"))
        qualified.rename(raw / "legacy-%m.profraw")
        with self.assertRaisesRegex(RuntimeError, "Unqualified native profile"):
            native_report(args, "unit", binaries, report)
        second_manifest = binaries[1].parent / "litesim-coverage-second.json"
        data = json.loads(second_manifest.read_text())
        data["filename_prefix"] = "native_control--first--"
        second_manifest.write_text(json.dumps(data))
        with self.assertRaisesRegex(RuntimeError, "Ambiguous native coverage-module"):
            native_report(args, "baseline", binaries, report)

    @unittest.skipUnless(shutil.which("clang++-14") and shutil.which("cmake")
                         and shutil.which("llvm-profdata-14") and shutil.which("llvm-cov-14"),
                         "LLVM14 and CMake toolchain required")
    def test_same_signature_plugin_profiles_are_target_qualified_and_unmixed(self):
        self.package()
        package = self.root / "new_plugin"
        self.write("new_plugin/CMakeLists.txt", "cmake_minimum_required(VERSION 3.20)\n"
            "project(collision_control LANGUAGES CXX)\n"
            "add_library(first SHARED src/first.cpp)\n"
            "add_library(second SHARED src/second.cpp)\n"
            "add_executable(host src/main.cpp)\ntarget_link_libraries(host PRIVATE dl)\n")
        for name in ("first", "second"):
            self.write(f"new_plugin/src/{name}.cpp",
                       'extern "C" int choose(bool value){if(value)return 1;return 2;}\n')
        self.write("new_plugin/src/main.cpp", '#include <dlfcn.h>\n'
            'int main(int argc,char**){\n'
            'void* a=dlopen("./libfirst.so",RTLD_NOW|RTLD_LOCAL);\n'
            'void* b=dlopen("./libsecond.so",RTLD_NOW|RTLD_LOCAL);\n'
            'if(!a||!b)return 8;\n'
            'auto first=reinterpret_cast<int(*)(bool)>(dlsym(a,"choose"));\n'
            'auto second=reinterpret_cast<int(*)(bool)>(dlsym(b,"choose"));\n'
            'int result=first(true)+(argc>1?0:second(false));\n'
            'dlclose(b);dlclose(a);return result!=(argc>1?1:3);\n}\n')
        build = self.root.parent / "build"
        for command in (["cmake", "-S", str(package), "-B", str(build),
                f"-DCMAKE_CXX_COMPILER={shutil.which('clang++-14')}",
                f"-DCMAKE_PROJECT_INCLUDE={QUALITY / 'instrument.cmake'}",
                "-DLITESIM_QUALITY_MODE=coverage"],
                ["cmake", "--build", str(build), "--parallel", "2"]):
            process = subprocess.run(command, capture_output=True, text=True, timeout=60)
            self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
        binaries = [build / name for name in ("host", "libfirst.so", "libsecond.so")]
        for label, arguments, second_hits in (("both", [], 1), ("only-first", ["skip"], 0)):
            evidence = self.root.parent / label
            raw = evidence / "profiles/unit"
            raw.mkdir(parents=True)
            env = dict(os.environ, LLVM_PROFILE_FILE=str(raw / "%m-%p.profraw"))
            # Two simultaneous processes must each retain independent DSO files.
            processes = [subprocess.Popen([str(binaries[0]), *arguments], cwd=build, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) for _ in range(2)]
            for process in processes:
                stdout, stderr = process.communicate(timeout=10)
                self.assertEqual(process.returncode, 0, stdout + stderr)
                self.assertNotIn("LLVM Profile Error", stderr)
            self.assertEqual(len([path for path in raw.glob("*.profraw") if path.stat().st_size]), 6)
            report = evidence / "coverage"
            report.mkdir()
            args = Namespace(root=self.root, prefix=self.root.parent / "install", evidence=evidence)
            observed = native_report(args, "unit", binaries, report)
            self.assertEqual(observed["new_plugin/src/first.cpp"]["functions"], counts(1, 1))
            self.assertEqual(observed["new_plugin/src/second.cpp"]["functions"], counts(second_hits, 1))
            self.assertEqual(observed["new_plugin/src/first.cpp"]["branches"], counts(1, 2))
            self.assertEqual(observed["new_plugin/src/second.cpp"]["branches"], counts(second_hits, 2))
            # The helper must not create fake measured functions of its own.
            for export in (report / "native/unit").glob("*.json"):
                functions = [function for data in json.loads(export.read_text())["data"]
                             for function in data["functions"]]
                self.assertEqual(len(functions), 1)

    def test_total_and_critical_thresholds_reject_low_coverage(self):
        path = "litesim3d_core/src/geometry.cpp"
        item = {"sources": {path: {"declarations_only": False}}}
        _, failures = gate(item, {path: {kind: counts(90, 100) for kind in ("lines", "branches", "functions")}})
        self.assertEqual(len(failures), 6)
        _, failures = gate(item, {path: {kind: counts(96 if kind == "branches" else 91, 100) for kind in ("lines", "branches", "functions")}})
        self.assertEqual(failures, [])

    def test_strict_total_gate_covers_noncritical_runtime_and_integer_boundary(self):
        path = "gui/src/viewer.cpp"
        item = {"sources": {path: {"declarations_only": False}}}
        for unit in (False, True):
            _, failures = gate(item, {path: {kind: counts(90, 100)
                for kind in ("lines", "branches", "functions")}}, unit=unit)
            self.assertEqual(len(failures), 3)
            _, failures = gate(item, {path: {kind: counts(950001 if kind == "branches" else 900001, 1000000)
                for kind in ("lines", "branches", "functions")}}, unit=unit)
            self.assertEqual(failures, [])

    def test_branch_95_limit_is_strict_and_does_not_raise_other_metrics(self):
        path = "runtime/src/client.cpp"
        item = {"sources": {path: {"declarations_only": False}}}
        for unit in (False, True):
            for hit, passes in ((949999, False), (950000, False), (950001, True)):
                metrics = {path: {"lines": counts(900001, 1000000),
                                  "functions": counts(900001, 1000000),
                                  "branches": counts(hit, 1000000)}}
                _, failures = gate(item, metrics, unit=unit)
                self.assertEqual(not failures, passes)
                if not passes:
                    self.assertEqual(len(failures), 1)
                    self.assertIn("branches", failures[0])
                    self.assertIn("<= 95%", failures[0])

    def test_declaration_exemption_cannot_hide_inline_function(self):
        self.assertFalse(has_function_body("struct A { int value=1; void run(); }; // void fake() {}"))
        self.assertTrue(has_function_body("bool ready() const noexcept { return true; }"))

    def test_model_geometry_declarations_remain_in_inventory_and_future_body_is_audited(self):
        relative = "litesim3d_plugin_api/include/litesim3d_plugin_api/model_geometry.hpp"
        source = (QUALITY.parents[1] / relative).read_text()
        self.package("litesim3d_plugin_api")
        path = self.write(relative, source)
        item = inventory(self.root)
        self.assertIn(relative, item["sources"])
        self.assertTrue(item["sources"][relative]["declarations_only"])
        self.assertNotIn(relative, item["exclusions"])
        self.assertFalse(has_function_body(source))
        _, failures = gate(item, {}, unit=True)
        self.assertNotIn("Missing source coverage: " + relative, failures)
        # The declaration-only marker is not permission to add executable code.
        path.write_text(source.replace("virtual ~ModelParameters();",
                                       "virtual ~ModelParameters() {}"))
        self.assertTrue(has_function_body(path.read_text()))

    def test_session_protocol_declaration_classification_preserves_inventory_and_rejects_future_bodies(self):
        from unittest.mock import patch
        import collect_coverage as collector
        relative = "litesim3d_ros2/src/session_protocol.hpp"
        source = (QUALITY.parents[1] / relative).read_text()
        self.package("litesim3d_ros2")
        path = self.write(relative, source)
        initial = inventory(self.root)
        self.assertIn(relative, initial["sources"])
        self.assertTrue(initial["sources"][relative]["declarations_only"])
        self.assertNotIn(relative, initial["exclusions"])
        self.assertFalse(has_function_body(source))
        evidence = self.root.parent / "declaration-evidence"
        evidence.mkdir(exist_ok=True)
        args = Namespace(root=self.root, build=self.root.parent / "build",
                         prefix=self.root.parent / "install", evidence=evidence)
        for addition in ("bool healthy() const { return !failed; }",
                         "bool computed = [] { return true; }();",
                         "ResetCoordinator() : run_id{} {}",
                         "int check() noexcept(true) { return 1; }",
                         'static constexpr const char *endpoint = "https://example.test"; bool healthy() const { return !failed; }',
                         "// even a comment change requires a fresh review"):
            path.write_text(source.replace(" public:", " public:\n  " + addition, 1))
            current = inventory(self.root)
            self.assertEqual(set(current["sources"]), set(initial["sources"]))
            # The exact-source guard must fail closed even if the legacy
            # text heuristic misses a URL/lambda/constructor body entirely.
            with (patch.object(collector, "has_function_body", return_value=False),
                  patch.object(collector, "compilation_errors", return_value=[]),
                  patch.object(collector, "objects", return_value=[]),
                  patch.object(collector, "native_report", return_value={}),
                  patch.object(collector, "python_report", return_value={})):
                self.assertTrue(collector.collect(args))
            gate_result = json.loads((evidence / "coverage/coverage-gate.json").read_text())
            self.assertIn("Declaration-only source changed; re-review required: " + relative,
                          gate_result["failures"])

    def test_unit_gate_cannot_borrow_integration_hits(self):
        path = "litesim3d_core/src/geometry.cpp"
        item = {"sources": {path: {"declarations_only": False}}}
        _, failures = gate(item, {path: {kind: counts(80, 100)
            for kind in ("lines", "branches", "functions")}}, unit=True)
        self.assertEqual(len(failures), 6)
        _, failures = gate(item, {path: {kind: counts(0, 0)
            for kind in ("lines", "branches", "functions")}}, unit=True)
        self.assertIn("No executable coverage mapping: " + path, failures)
        self.assertEqual(len(failures), 4)

    def test_missing_function_profile_cannot_shrink_denominator(self):
        observed = {"main.cpp": {"lines": counts(1, 1), "branches": counts(0, 0), "functions": counts(1, 1)}}
        baseline = {"main.cpp": {"lines": counts(0, 47), "branches": counts(0, 26), "functions": counts(0, 4)},
                    "missed.cpp": {"lines": counts(0, 8), "branches": counts(0, 2), "functions": counts(0, 2)}}
        fixed = fixed_denominators(observed, baseline)
        self.assertEqual(fixed["main.cpp"]["lines"], counts(1, 47))
        self.assertEqual(fixed["missed.cpp"]["lines"], counts(0, 8))
        self.assertEqual(fixed["main.cpp"]["functions"], counts(1, 4))
        self.assertEqual(fixed["missed.cpp"]["functions"], counts(0, 2))
        with self.assertRaises(RuntimeError):
            fixed_denominators({"unknown.cpp": observed["main.cpp"]}, baseline)
        with self.assertRaisesRegex(RuntimeError, "functions exceed"):
            fixed_denominators({"main.cpp": {**observed["main.cpp"], "functions": counts(5, 5)}}, baseline)

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
        config = evidence / "coveragerc"
        config.write_text(coverage_config(self.root, prefix, inventory(self.root)))
        env = dict(os.environ, COVERAGE_PROCESS_START=str(config),
                   COVERAGE_FILE=str(raw / ".coverage"),
                   LITESIM_FUNCTION_COVERAGE_ROOTS=json.dumps([str(self.root), str(prefix)]),
                   PYTHONPATH=str(QUALITY / "python_startup"))
        run = subprocess.run([sys.executable, "-c",
            f"import runpy; namespace=runpy.run_path({str(installed)!r}); assert namespace['choose'](True)==1"],
            env=env, capture_output=True, text=True, timeout=30)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        report = evidence / "coverage"
        report.mkdir()
        result = python_report(Namespace(root=self.root, prefix=prefix, evidence=evidence), "unit", inventory(self.root), report)
        self.assertGreater(result[used.relative_to(self.root).as_posix()]["lines"]["covered"], 0)
        self.assertEqual(result[missed.relative_to(self.root).as_posix()]["lines"], counts(0, 2))
        self.assertEqual(result[used.relative_to(self.root).as_posix()]["functions"], counts(1, 1))
        self.assertEqual(result[missed.relative_to(self.root).as_posix()]["functions"], counts(0, 1))

    def test_python_entry_counts_calls_not_definitions_and_same_line_lambdas(self):
        path = self.write("runtime.py",
            "first, second = (lambda: 42), (lambda: 42)\n"
            "def unused():\n    return 99\n"
            "def outer():\n    def inner():\n        return 2\n    return inner\n"
            "class Item:\n    def method(self):\n        return 7\n"
            "async def later():\n    return 3\n")
        namespace = {}
        tracer = FunctionCoverage([self.root], self.root / ".coverage").start()
        try:
            exec(compile(path.read_bytes(), str(path), "exec"), namespace)
            self.assertEqual(namespace["first"](), 42)
            namespace["outer"]()  # Creating inner is not calling inner.
            namespace["later"]().close()  # Nor is creating an unstarted coroutine.
        finally:
            tracer.stop()
        definitions = source_functions(path)
        hits = tracer.hits[str(path)]
        self.assertEqual(len(definitions), 7)
        self.assertEqual(len(hits), 2)
        self.assertEqual(len([name for name in definitions if ":<lambda>:" in name]), 2)
        self.assertEqual(len([name for name in hits if ":<lambda>:" in name]), 1)
        self.assertEqual(json.loads(tracer.save().read_text())["files"][str(path)], sorted(hits))

    def test_unstarted_async_and_generator_bodies_are_not_credited(self):
        path = self.write("runtime.py",
            "async def dormant():\n    return 9\n"
            "async def active():\n    return 7\n"
            "def dormant_generator():\n    yield 9\n"
            "def active_generator():\n    yield 7\n")
        namespace = {}
        tracer = FunctionCoverage([self.root], self.root / ".coverage").start()
        try:
            exec(compile(path.read_bytes(), str(path), "exec"), namespace)
            namespace["dormant"]().close()
            namespace["dormant_generator"]().close()
            coroutine = namespace["active"]()
            with self.assertRaises(StopIteration):
                coroutine.send(None)
            generator = namespace["active_generator"]()
            self.assertEqual(next(generator), 7)
            generator.close()
        finally:
            tracer.stop()
        self.assertEqual(len(source_functions(path)), 4)
        self.assertEqual(len(tracer.hits[str(path)]), 2)
        self.assertFalse(any(":dormant" in name for name in tracer.hits[str(path)]))

    def test_decorated_function_entry_and_optimized_away_definition_remain_accounted(self):
        path = self.write("runtime.py",
            "def identity(fn):\n    return fn\n"
            "@identity\ndef decorated():\n    return 7\n"
            "if False:\n    def unreachable():\n        return 9\n")
        namespace = {}
        tracer = FunctionCoverage([self.root], self.root / ".coverage").start()
        try:
            exec(compile(path.read_bytes(), str(path), "exec"), namespace)
            self.assertEqual(namespace["decorated"](), 7)
        finally:
            tracer.stop()
        self.assertEqual(len(source_functions(path)), 3)
        self.assertEqual(len(tracer.hits[str(path)]), 2)
        self.assertTrue(tracer.hits[str(path)] <= source_functions(path))

    def test_missing_and_stale_function_entry_profiles_fail_closed(self):
        self.package()
        self.write("new_plugin/runtime.py", "def unused():\n    return 7\n")
        directory = self.root.parent / "profiles"
        directory.mkdir()
        args = Namespace(root=self.root, prefix=self.root.parent / "install")
        item = inventory(self.root)
        with self.assertRaisesRegex(RuntimeError, "No Python function-entry"):
            python_functions(args, [directory], item)
        profile = directory / ".functions.1.json"
        profile.write_text(json.dumps({"version": 1, "files": {}}))
        self.assertEqual(python_functions(args, [directory], item)["new_plugin/runtime.py"], counts(0, 1))
        profile.write_text(json.dumps({"version": 1, "files": {
            str(self.root / "new_plugin/runtime.py"): ["unknown:stale:999"]}}))
        with self.assertRaisesRegex(RuntimeError, "do not match the current source"):
            python_functions(args, [directory], item)
        profile.write_text(json.dumps({"version": 2, "files": {}}))
        with self.assertRaisesRegex(RuntimeError, "Unsupported"):
            python_functions(args, [directory], item)

    def test_ambiguous_native_module_profiles_are_rejected_not_credited_twice(self):
        from unittest.mock import patch
        for output in ("Binary IDs:\n", "Binary IDs:\nabc123\ndef456\n"):
            with patch("collect_coverage.subprocess.check_output", return_value=output):
                with self.assertRaisesRegex(RuntimeError, "unambiguous binary ID"):
                    profile_binary_id("llvm-profdata-14", Path("ambiguous.profraw"))

    def test_python_entries_include_calls_on_new_threads(self):
        import threading
        path = self.write("runtime.py", "def work():\n    return 7\n")
        namespace = {}
        tracer = FunctionCoverage([self.root], self.root / ".coverage").start()
        try:
            exec(compile(path.read_bytes(), str(path), "exec"), namespace)
            thread = threading.Thread(target=namespace["work"])
            thread.start()
            thread.join()
        finally:
            tracer.stop()
        self.assertEqual(tracer.hits[str(path)], source_functions(path))

    def test_runner_keeps_unit_and_integration_python_calls_separate(self):
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        self.package()
        source = self.write("new_plugin/scripts/runtime.py",
            "import sys\ndef first():\n    return 1\n"
            "def second():\n    return 2\n"
            "print(first() if sys.argv[1] == 'unit' else second())\n")
        args = Namespace(output=self.root.parent / "quality", jobs=1,
                         ros_prefix=self.root.parent / "underlay", rmw="rmw_fastrtps_cpp")
        with patch.object(test_ubuntu, "ROOT", self.root):
            profile = test_ubuntu.Profile(args, "coverage")
            profile.prefix.mkdir(parents=True)
            (profile.prefix / "setup.bash").write_text("")
            profile.configure_coverage()
            profile.env["PYTHONPATH"] = str(QUALITY / "python_startup")
            self.assertTrue(profile.stage("isolated-unit", [sys.executable, source, "unit"], unit=True))
            self.assertTrue(profile.stage("real-integration", [sys.executable, source, "integration"]))
        report = profile.evidence / "coverage"
        report.mkdir()
        options = Namespace(root=self.root, prefix=profile.prefix, evidence=profile.evidence)
        item = inventory(self.root)
        unit = python_report(options, "unit", item, report)
        combined = python_report(options, "combined", item, report)
        name = source.relative_to(self.root).as_posix()
        self.assertEqual(unit[name]["functions"], counts(1, 2))
        self.assertEqual(combined[name]["functions"], counts(2, 2))
        self.assertEqual(len(list((profile.evidence / "profiles/unit/python").glob(".functions.*.json"))), 1)
        self.assertEqual(len(list((profile.evidence / "profiles/integration/python").glob(".functions.*.json"))), 1)

    def test_runner_classifies_only_isolated_native_control_and_gui_as_unit(self):
        sys.path.insert(0, str(QUALITY.parent))
        from test_ubuntu import Profile
        stages = []
        profile = Namespace(runtime=self.root.parent / "runtime", evidence=self.root.parent / "evidence",
                            prefix=self.root.parent / "runtime/install", env={}, name="coverage", stage=lambda *args, **kwargs: stages.append((args, kwargs)))
        Profile.native(profile)
        Profile.python_units(profile)
        isolated = {args[0]: (args, kwargs) for args, kwargs in stages}
        for name in ("runtime_unit", "runtime_process_unit", "native_model_unit", "native_cli_unit",
                     "session_protocol_unit", "ros_client_unit", "sim_session_unit", "spawn_when_ready_unit", "core_cli_unit_tests", "livox_outputs_unit", "test_sdf_import_units",
                     "ros_event_loop_test", "render_helpers_test", "render_orchestration_unit", "viewer_unit", "camera_view_unit",
                     "point_adapter_unit", "service_responses_unit", "joint_control_unit",
                     "lidar_outputs_unit", "cloud_tools_unit", "world_node_unit", "sim_node_unit",
                     "control-unit"):
            self.assertTrue(isolated[name][1]["unit"])
        control = isolated["control-unit"][0][1]
        self.assertIn("^(body_configuration|omni_configuration|control_loop_sleep|controller_units|system_units|spawner_unit|readiness_unit)$", control)
        self.assertFalse(any("test_integration.py" in str(arg) or "test_shutdown.py" in str(arg)
                             or "test_controller_parameters.py" in str(arg)
                             for _, (args, _) in isolated.items() for arg in args))
        self.assertTrue(isolated["bringup-unit"][1]["unit"])
        self.assertTrue(isolated["python-runtime-unit"][1]["unit"])
        self.assertNotIn("lifecycle", isolated)
        self.assertNotIn("runtime-process-integration", isolated)
        for name, package in (("runtime_process_unit", "litesim3d_runtime"),
                              ("native_cli_unit", "litesim3d_bringup")):
            command = isolated[name][0][1]
            self.assertEqual(command[0], profile.runtime / "build" / package / name)
            self.assertIn(f"--gtest_output=xml:{profile.evidence / (name + '-cases.xml')}", command)
        Profile.lifecycle(profile)
        lifecycle = next((args, kwargs) for args, kwargs in stages if args[0] == "lifecycle")
        self.assertFalse(lifecycle[1].get("unit", False))
        Profile.regression(profile)
        integration = next((args, kwargs) for args, kwargs in stages
                           if args[0] == "controller-parameters-integration")
        self.assertFalse(integration[1].get("unit", False))
        cli = next((args, kwargs) for args, kwargs in stages if args[0] == "core-cli-integration")
        self.assertFalse(cli[1].get("unit", False))
        self.assertTrue(any("core_cli_integration_tests" in str(part) for part in cli[0][1]))
        process = next((args, kwargs) for args, kwargs in stages
                       if args[0] == "runtime-process-integration")
        self.assertFalse(process[1].get("unit", False))
        self.assertEqual(process[0][1][0], profile.runtime / "build/litesim3d_runtime/runtime_process_integration")

    def test_unit_only_cli_rejects_every_other_profile_before_execution(self):
        from contextlib import redirect_stderr
        import io
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        for name in ("all", "functional", "asan", "tsan", "release"):
            output = self.root.parent / ("rejected-" + name)
            command = ["test_ubuntu.py", "--unit-only", "--profile", name, "--output", str(output)]
            with patch.object(sys, "argv", command), patch.object(test_ubuntu, "Profile") as profile:
                with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    test_ubuntu.main()
                self.assertEqual(error.exception.code, 2)
                profile.assert_not_called()
            self.assertFalse(output.exists())
        args = Namespace(output=self.root.parent / "invalid", unit_only=True)
        with self.assertRaisesRegex(ValueError, "only for the coverage"):
            test_ubuntu.Profile(args, "functional")

    def test_unit_only_pipeline_runs_all_unit_stages_without_full_matrix_claim(self):
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        args = Namespace(output=self.root.parent / "quality", jobs=1, unit_only=True,
                         ros_prefix=self.root.parent / "underlay", rmw="rmw_fastrtps_cpp")
        with patch.object(test_ubuntu, "ROOT", self.root):
            profile = test_ubuntu.Profile(args, "coverage")
            calls = []
            def stage(name, command, *unused, **options):
                calls.append((name, list(map(str, command)), options))
                profile.record["stages"].append({"name": name, "status": "PASS"})
                return True
            with patch.object(profile, "stage", side_effect=stage), \
                    patch.object(profile, "build", return_value=True) as build, \
                    patch.object(profile, "regression", side_effect=AssertionError("integration attempted")):
                self.assertTrue(profile.execute())
            build.assert_called_once()
        stages = {name: (command, options) for name, command, options in calls}
        self.assertIn("sim_core_tests", stages)
        self.assertIn("sim_node_unit", stages)
        self.assertIn("world_node_unit", stages)
        self.assertIn("camera_view_unit", stages)
        self.assertIn("bringup-unit", stages)
        self.assertIn("python-runtime-unit", stages)
        self.assertIn("coverage-report", stages)
        self.assertFalse(stages["coverage-report"][1]["installed"])
        self.assertFalse(any(options.get("graphical") for _, _, options in calls))
        self.assertFalse({"regression", "desktop", "sanitizer-report", "controller-parameters-integration",
                          "core-cli-integration", "livox-output-plugin-integration", "lifecycle"} & stages.keys())
        tooling = stages["coverage-tooling-unit"][0]
        for module in ("test_quality.py", "test_sanitizer_controls.py", "test_install_contract.py",
                       "test_run_checked_xml.py"):
            self.assertIn(str(self.root / "scripts/quality/tests" / module), tooling)
        self.assertIn("not test_asan_executable_mapping_keeps_plugins_shared", tooling)
        self.assertEqual(profile.env["LWS_BUILD_GUI"], "1")
        self.assertEqual(profile.record["scope"], "unit-only")
        self.assertEqual(profile.record["status"], "UNIT_ONLY_PASS")
        for category in ("integration", "graphical_integration", "sanitizer_controls", "full_quality_matrix"):
            self.assertEqual(profile.record["categories"][category], "NOT_RUN")

    def test_unit_only_build_keeps_gui_and_instrumentation_without_display_preflight(self):
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        def tool(name, versioned=None):
            return None if name in {"xvfb-run", "dbus-run-session"} else "/usr/bin/" + (versioned or name)
        with patch.object(test_ubuntu, "ROOT", self.root), patch.object(test_ubuntu, "tool", side_effect=tool):
            args = Namespace(output=self.root.parent / "unit", jobs=2, unit_only=True,
                             ros_prefix=self.root.parent / "underlay", rmw="rmw_fastrtps_cpp")
            profile = test_ubuntu.Profile(args, "coverage")
            with patch.object(profile, "stage", return_value=False) as stage:
                self.assertFalse(profile.build())
            self.assertEqual(stage.call_args.args[0], "build")
            command = list(map(str, stage.call_args.args[1]))
            self.assertIn(str(self.root / "scripts/build.sh"), command)
            self.assertIn("-DLITESIM_QUALITY_MODE=coverage", command)
            self.assertIn("-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O1 -g -DNDEBUG", command)
            self.assertEqual(profile.env["LWS_BUILD_GUI"], "1")
            self.assertFalse(any("packages-select" in item or "BUILD_GUI=0" in item for item in command))
            default = test_ubuntu.Profile(Namespace(output=self.root.parent / "full", jobs=2,
                ros_prefix=self.root.parent / "underlay", rmw="rmw_fastrtps_cpp"), "coverage")
            self.assertFalse(default.unit_only)  # Older fixtures omit the flag.
            with self.assertRaisesRegex(RuntimeError, "Install xvfb"):
                default.build()

    def test_unit_only_rejects_accidental_integration_and_graphical_stages(self):
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        args = Namespace(output=self.root.parent / "unit", jobs=1, unit_only=True,
                         ros_prefix=self.root.parent / "underlay", rmw="rmw_fastrtps_cpp")
        with patch.object(test_ubuntu, "ROOT", self.root):
            profile = test_ubuntu.Profile(args, "coverage")
            with patch.object(test_ubuntu, "run") as run:
                with self.assertRaisesRegex(RuntimeError, "Graphical integration"):
                    profile.stage("bad-display", ["never-run"], installed=False, graphical=True)
                with self.assertRaisesRegex(RuntimeError, "Integration is outside"):
                    profile.stage("bad-peer", ["never-run"])
                run.assert_not_called()
            self.assertEqual(profile.record["stages"], [])
            self.assertFalse((profile.evidence / "profiles/integration").exists())

    def test_default_profile_retains_full_tooling_and_integration_path(self):
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        args = Namespace(output=self.root.parent / "full", jobs=1,
                         ros_prefix=self.root.parent / "underlay", rmw="rmw_fastrtps_cpp")
        with patch.object(test_ubuntu, "ROOT", self.root):
            profile = test_ubuntu.Profile(args, "coverage")
            calls = []
            def stage(name, command, *unused, **options):
                calls.append((name, list(map(str, command)), options))
                profile.record["stages"].append({"name": name, "status": "PASS"})
                return True
            with patch.object(profile, "stage", side_effect=stage), \
                    patch.object(profile, "build", return_value=True), \
                    patch.object(profile, "native"), patch.object(profile, "python_units"), \
                    patch.object(profile, "regression") as regression:
                self.assertTrue(profile.execute())
            regression.assert_called_once()
        stages = {name: (command, options) for name, command, options in calls}
        self.assertIn(str(self.root / "scripts/quality/tests"), stages["quality-tests"][0])
        self.assertNotIn("-k", stages["quality-tests"][0])
        self.assertTrue(stages["coverage-report"][1]["installed"])
        self.assertIn("lifecycle", stages)
        self.assertFalse(stages["lifecycle"][1].get("unit", False))
        self.assertEqual(profile.record["status"], "PASS")
        self.assertEqual(profile.record["scope"], "full-profile")

    def test_livox_real_plugin_load_is_integration_not_unit_coverage(self):
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        for name in ("functional", "coverage", "asan"):
            args = Namespace(output=self.root.parent / name, jobs=1,
                             ros_prefix=self.root.parent / "underlay", rmw="rmw_fastrtps_cpp")
            with patch.object(test_ubuntu, "ROOT", self.root):
                profile = test_ubuntu.Profile(args, name)
                with patch.object(profile, "stage", return_value=True) as stage, \
                        patch.object(test_ubuntu.subprocess, "check_output", return_value="100"):
                    profile.regression()
                calls = [call for call in stage.call_args_list
                         if call.args[0] == "livox-output-plugin-integration"]
                self.assertEqual(len(calls), 1)
                command = calls[0].args[1]
                self.assertEqual(command[0], profile.runtime / "build/litesim3d_mid360/livox_output_plugin_integration")
                self.assertEqual(command[1], f"--gtest_output=xml:{profile.evidence / 'livox-output-plugin-integration-cases.xml'}")
                self.assertEqual(calls[0].args[2], 60)
                self.assertFalse(calls[0].kwargs.get("unit", False))
                self.assertFalse(calls[0].kwargs.get("graphical", False))
                with patch.object(profile, "stage", return_value=True) as stage:
                    profile.native()
                self.assertFalse(any("livox_output_plugin_integration" in str(call)
                                     for call in stage.call_args_list))

    def test_unit_only_summary_is_scope_specific_and_never_full_matrix_pass(self):
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        output = self.root.parent / "unit-summary"
        command = ["test_ubuntu.py", "--profile", "coverage", "--unit-only", "--output", str(output)]
        def execute(profile):
            profile.record.update(test_ubuntu.scope_details(True, "PASS"))
            profile.record["status"] = "UNIT_ONLY_PASS"
            profile.save()
            return True
        cwd = Path.cwd()
        try:
            with patch.object(test_ubuntu, "ROOT", self.root), patch.object(sys, "argv", command), \
                    patch.dict(os.environ, ROS_DISTRO="humble"), \
                    patch.object(test_ubuntu.platform, "system", return_value="Linux"), \
                    patch.object(test_ubuntu.Profile, "execute", autospec=True, side_effect=execute):
                self.assertEqual(test_ubuntu.main(), 0)
        finally:
            os.chdir(cwd)
        summary = json.loads((output / "summary.json").read_text())
        self.assertEqual(summary["status"], "UNIT_ONLY_PASS")
        self.assertEqual(summary["scope"], "unit-only")
        self.assertEqual(summary["categories"]["full_quality_matrix"], "NOT_RUN")
        self.assertEqual(summary["categories"]["sanitizer_controls"], "NOT_RUN")
        self.assertTrue(summary["same_source_across_profiles"])

    def test_unit_only_report_failure_and_source_changes_cannot_pass(self):
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        for scenario in ("threshold-failure", "changed-source"):
            args = Namespace(output=self.root.parent / scenario, jobs=1, unit_only=True,
                             ros_prefix=self.root.parent / "underlay", rmw="rmw_fastrtps_cpp")
            with patch.object(test_ubuntu, "ROOT", self.root):
                profile = test_ubuntu.Profile(args, "coverage")
                def stage(name, command, *unused, **options):
                    failed = name == "coverage-report" and scenario == "threshold-failure"
                    if name == "coverage-report" and scenario == "changed-source":
                        (self.root / "changed.cpp").write_text("int changed(){return 1;}\n")
                    profile.record["stages"].append({"name": name, "status": "FAIL" if failed else "PASS"})
                    return not failed
                with patch.object(profile, "stage", side_effect=stage), \
                        patch.object(profile, "build", return_value=True), \
                        patch.object(profile, "regression", side_effect=AssertionError("integration attempted")):
                    self.assertFalse(profile.execute())
            self.assertEqual(profile.record["status"], "UNIT_ONLY_FAIL")
            self.assertEqual(profile.record["categories"]["unit_coverage"], "FAIL")
            self.assertEqual(profile.record["categories"]["full_quality_matrix"], "NOT_RUN")
            if scenario == "changed-source":
                self.assertIn("Source changed", profile.record["error"])

    def test_unit_only_cancellation_is_not_a_matrix_pass(self):
        import signal
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        output = self.root.parent / "unit-cancelled"
        command = ["test_ubuntu.py", "--profile", "coverage", "--unit-only", "--output", str(output)]
        cwd = Path.cwd()
        try:
            with patch.object(test_ubuntu, "ROOT", self.root), patch.object(sys, "argv", command), \
                    patch.dict(os.environ, ROS_DISTRO="humble"), \
                    patch.object(test_ubuntu.platform, "system", return_value="Linux"), \
                    patch.object(test_ubuntu.Profile, "execute", side_effect=test_ubuntu.Interrupted(signal.SIGTERM)):
                self.assertEqual(test_ubuntu.main(), 128 + signal.SIGTERM)
        finally:
            os.chdir(cwd)
        summary = json.loads((output / "summary.json").read_text())
        self.assertEqual(summary["status"], "UNIT_ONLY_CANCELLED")
        self.assertEqual(summary["scope"], "unit-only")
        self.assertEqual(summary["categories"]["unit_coverage"], "CANCELLED")
        self.assertEqual(summary["categories"]["full_quality_matrix"], "NOT_RUN")

    def test_full_tsan_retains_lifecycle_even_when_regression_is_not_run(self):
        from unittest.mock import patch
        sys.path.insert(0, str(QUALITY.parent))
        import test_ubuntu
        args = Namespace(output=self.root.parent / "tsan", jobs=1,
                         ros_prefix=self.root.parent / "underlay", rmw="rmw_fastrtps_cpp")
        with patch.object(test_ubuntu, "ROOT", self.root):
            profile = test_ubuntu.Profile(args, "tsan")
            calls = []
            def stage(name, command, *unused, **options):
                calls.append((name, command, options))
                profile.record["stages"].append({"name": name, "status": "PASS"})
                return True
            with patch.object(profile, "stage", side_effect=stage), \
                    patch.object(profile, "build", return_value=True), \
                    patch.object(profile, "native"), patch.object(profile, "python_units") as python_units, \
                    patch.object(profile, "regression") as regression:
                self.assertTrue(profile.execute())
            python_units.assert_not_called()
            regression.assert_not_called()
        lifecycle = next((command, options) for name, command, options in calls if name == "lifecycle")
        self.assertFalse(lifecycle[1].get("unit", False))
        self.assertIn("--instrumented", lifecycle[0])
        self.assertEqual(profile.record["status"], "PASS")


if __name__ == "__main__":
    unittest.main()
