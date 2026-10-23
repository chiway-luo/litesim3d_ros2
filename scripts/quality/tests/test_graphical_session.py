"""Exercise the real shared wrapper without relying on a logged-in desktop."""
from argparse import Namespace
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

SCRIPTS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCRIPTS))
from test_ubuntu import Profile
from quality.sanitizer_control import asan_link_options, asan_options


class GraphicalSessionTests(unittest.TestCase):
    @unittest.skipUnless(platform.system() == "Linux", "Ubuntu graphical test environment required")
    def test_private_bus_is_usable_reaped_and_preserves_exit_status(self):
        tools = {name: shutil.which(name) for name in
                 ("clang++-14", "llvm-symbolizer-14", "pkg-config", "dbus-run-session", "xvfb-run")}
        self.assertTrue(all(tools.values()), f"Missing Ubuntu quality tools: {tools}")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            setup = root / "underlay"
            setup.mkdir()
            (setup / "setup.bash").write_text("")
            binary = root / "qt-session"
            qt_flags = shlex.split(subprocess.check_output(
                [tools["pkg-config"], "--cflags", "--libs", "Qt5Core", "Qt5DBus"], text=True))
            build = subprocess.run([
                tools["clang++-14"], "-g", "-fPIC", "-fsanitize=address", "-fno-omit-frame-pointer",
                *asan_link_options(True), str(SCRIPTS / "fixtures/qt_session/main.cpp"),
                "-o", str(binary), *qt_flags], capture_output=True, text=True, timeout=60)
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
            profile = Profile(Namespace(output=root / "checks", jobs=1, ros_prefix=setup,
                                        rmw="rmw_fastrtps_cpp"), "functional")
            # A stale inherited bus must not be used. Also reproduce the lack
            # of a login session in the CI container instead of borrowing SSH's.
            stale_bus = "unix:path=" + str(root / "nonexistent-bus")
            profile.env["DBUS_SESSION_BUS_ADDRESS"] = stale_bus
            profile.env.pop("XDG_RUNTIME_DIR", None)
            profile.env.pop("LSAN_OPTIONS", None)
            profile.env["ASAN_OPTIONS"] = asan_options(tools["llvm-symbolizer-14"])
            addresses = []
            for name, arguments, expected in (("normal", [], 0), ("failure", ["fail"], 17)):
                with self.subTest(exit_code=expected):
                    passed = profile.stage(name, [binary, *arguments], timeout=30,
                                           installed=False, graphical=True)
                    log = (profile.evidence / f"{name}.log").read_text()
                    self.assertEqual(profile.record["stages"][-1]["exit_code"], expected, log)
                    self.assertEqual(passed, expected == 0)
                    self.assertNotIn("Sanitizer", log)
                    address = re.search(r"^BUS_ADDRESS=(.+)$", log, re.MULTILINE)
                    daemon = re.search(r"^BUS_PID=(\d+)$", log, re.MULTILINE)
                    self.assertIsNotNone(address, log)
                    self.assertIsNotNone(daemon, log)
                    self.assertNotEqual(address.group(1), stale_bus)
                    addresses.append(address.group(1))
                    process = Path("/proc") / daemon.group(1)
                    deadline = time.monotonic() + 3
                    while process.exists() and time.monotonic() < deadline:
                        time.sleep(0.05)
                    self.assertFalse(process.exists(),
                                     "Private D-Bus daemon was not reaped; containers require an init process (docker --init)")
            self.assertEqual(len(set(addresses)), 2, "Graphical stages shared a bus")


if __name__ == "__main__":
    unittest.main()
