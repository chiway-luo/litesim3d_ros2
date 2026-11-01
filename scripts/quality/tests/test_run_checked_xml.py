"""JUnit must remain parseable without changing the original command evidence."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

SCRIPTS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCRIPTS))
from checks import run_checked


# Retained from the ASan desktop runner failure: literal terminal escapes made
# runner.xml invalid even though the command's failure was correctly detected.
CI_ANSI_LINE = (
    "E       [spawn_when_ready.py-5] [INFO] [1793531373.181967802] "
    "[spawner_body_velocity_controller]: "
    "\x1b[94mLoaded \x1b[1mbody_velocity_controller\x1b[0m\n"
)
INVALID_CONTROLS = "".join(chr(code) for code in range(32) if code not in (9, 10, 13))
VALID_BOUNDARIES = "\t\n\r \x7f\x85\ud7ff\ue000\ufffd\U00010000\U0001fffe\U0001ffff\U0010fffe\U0010ffff"


def test_nonzero_log_keeps_raw_bytes_and_xml_replaces_only_invalid_characters(tmp_path):
    text = CI_ANSI_LINE + INVALID_CONTROLS + "\ufffe\uffff" + "\nUnicode: 中文 café 🛰\n"
    raw = text.encode("utf-8")
    command = [sys.executable, "-c", f"import os; os.write(2, {raw!r}); raise SystemExit(7)"]
    xml, log = tmp_path / "result.xml", tmp_path / "command.log"
    assert run_checked.run(command, 5, xml, log) == 7
    assert log.read_bytes() == raw
    suite = ET.parse(xml).getroot()
    assert suite.attrib == {"name": "required_command", "tests": "1", "failures": "1"}
    failure = suite.find("testcase/failure")
    assert failure is not None and failure.get("message") == "command exited 7"
    expected = CI_ANSI_LINE.replace("\x1b", "\ufffd")
    expected += "\ufffd" * (len(INVALID_CONTROLS) + 2) + "\nUnicode: 中文 café 🛰\n"
    assert failure.text == expected


def test_command_arguments_are_xml_safe_without_changing_child_argv(tmp_path):
    argument = "argument:\x01\x1b\x1f\ufffe\uffff:中文🛰"
    command = [sys.executable, "-c", "import os,sys; os.write(1,sys.argv[1].encode('utf-8'))", argument]
    xml, log = tmp_path / "result.xml", tmp_path / "command.log"
    assert run_checked.run(command, 5, xml, log) == 0
    assert log.read_bytes() == argument.encode("utf-8")
    suite = ET.parse(xml).getroot()
    assert suite.get("failures") == "0"
    case = suite.find("testcase")
    assert case is not None and case.find("failure") is None
    assert case.get("name") == " ".join(command[:-1]) + " argument:" + "\ufffd" * 5 + ":中文🛰"


def test_invalid_utf8_diagnostic_does_not_mask_child_failure_or_change_log(tmp_path):
    raw = b"\xff\x1b[31mfailure\x1b[0m\x00\n"
    command = [sys.executable, "-c", f"import os; os.write(2, {raw!r}); raise SystemExit(9)"]
    xml, log = tmp_path / "result.xml", tmp_path / "command.log"
    assert run_checked.run(command, 5, xml, log) == 9
    assert log.read_bytes() == raw
    failure = ET.parse(xml).find("testcase/failure")
    assert failure is not None and failure.get("message") == "command exited 9"
    assert failure.text == "\ufffd\ufffd[31mfailure\ufffd[0m\ufffd\n"


def test_xml_text_preserves_xml_10_valid_unicode_boundaries():
    assert run_checked.xml_text(VALID_BOUNDARIES) == VALID_BOUNDARIES
    element = ET.Element("testcase", name=run_checked.xml_text(VALID_BOUNDARIES))
    # Attribute escaping must retain even tab/newline/CR at the XML boundary.
    assert ET.fromstring(ET.tostring(element, encoding="utf-8")).get("name") == VALID_BOUNDARIES


def test_xml_text_replaces_surrogates_and_invalid_unicode_boundaries():
    invalid = INVALID_CONTROLS + "\ud800\udbff\udc00\udfff\ufffe\uffff"
    expected = "\ufffd" * len(invalid)
    assert run_checked.xml_text(invalid) == expected
    element = ET.Element("failure", message=run_checked.xml_text(invalid))
    element.text = run_checked.xml_text(invalid)
    parsed = ET.fromstring(ET.tostring(element, encoding="utf-8"))
    assert parsed.get("message") == expected and parsed.text == expected
