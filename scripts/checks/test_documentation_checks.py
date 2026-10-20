"""Regression checks for documentation gate failures, using isolated fixtures."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("doc_check", Path(__file__).with_name("check_repository_docs.py"))
DOC = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DOC)


class DocumentationGateTests(unittest.TestCase):
    def check_readme(self, text):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            (root / "README.md").write_text(text)
            return DOC.check(root)

    def test_existing_file_with_missing_fragment_is_rejected(self):
        result = self.check_readme("# Example\n[broken](README.md#removed-section)\n")
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("missing heading/anchor", result["failures"][0])

    def test_duplicate_heading_and_explicit_anchor_resolve(self):
        result = self.check_readme(
            '# Example\n## 使用\n## 使用\n<a id="stable"></a>\n'
            '[second](#使用-1) [stable](README.md#stable)\n')
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["anchor_links"], 2)

    def test_example_code_is_not_mistaken_for_document_links(self):
        result = self.check_readme('# Example\n```text\n[fake](absent.md)\n## not-a-heading\n```\n')
        self.assertEqual(result["status"], "PASS")
        self.assertNotIn("not-a-heading", DOC.anchors("```text\n## not-a-heading\n```"))

    def test_invalid_shell_is_rejected(self):
        result = self.check_readme("# Example\n```bash\nif true; then\n```\n")
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("invalid shell block", result["failures"][0])


if __name__ == "__main__":
    unittest.main()
