"""Evidence gates reject empty/skipped/failed, stale and shortened runs."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pytest
from reports.report_completion import b3_passed, suite, suite_passed


@pytest.mark.parametrize('body, count, passed', [
    ('', 1, False), ('', 0, False), ('<testcase name="ok"/>', 1, True),
    ('<testcase><skipped/></testcase>', 1, False),
    ('<testcase><failure/></testcase>', 1, False),
    ('<testcase><error/></testcase>', 1, False),
    ('<testcase name="ok"/>', 2, False),
])
def test_xml_rejects_unexecuted_or_failed(tmp_path, body, count, passed):
    path = tmp_path/'suite.xml'
    path.write_text('<testsuite>'+body+'</testsuite>')
    assert suite_passed(suite(path), count) == passed
    assert not suite_passed(suite(tmp_path/'missing.xml'), 1)


@pytest.mark.parametrize('change', ['none', 'short', 'failed', 'stale_bridge', 'stale_core',
                                  'running', 'missing_robot', 'bad_exit', 'slow_bridge'])
def test_performance_requires_complete_current_evidence(change):
    result = {'status': 'PASS', 'rtf_target': 1, 'gui': False, 'warmup_seconds': 30,
              'duration_seconds': 120, 'wall_seconds': 120.1, 'rtf': 1.,
              'internal_missed_scans': 0, 'peak_reported_compute_p99_ms': 10.,
              'peak_reported_bridge_p99_ms': 1., 'resources': {'sim': {'rss_peak': 300*1024**2}},
              'exits': {'sim': 0, 'r0': 0, 'r1': 0, 'r2': 0, 'r3': 0}}
    manifest = {'exit_reason': 'shutdown', 'bridge_source_sha256': 'bridge',
                'robots': [{'core_source_sha256': 'core', 'exit_reason': 'shutdown'} for _ in range(4)]}
    if change == 'short': result['wall_seconds'] = 119.9
    if change == 'failed': result['status'] = 'FAIL'
    if change == 'stale_bridge': manifest['bridge_source_sha256'] = 'old'
    if change == 'stale_core': manifest['robots'][1]['core_source_sha256'] = 'old'
    if change == 'running': manifest['robots'][0]['exit_reason'] = 'running'
    if change == 'missing_robot': manifest['robots'].pop()
    if change == 'bad_exit': result['exits']['r2'] = -9
    if change == 'slow_bridge': result['peak_reported_bridge_p99_ms'] = 20.1
    assert b3_passed(result, manifest, 'core', 'bridge', 1) == (change == 'none')
    assert not b3_passed({}, {}, 'core', 'bridge', 1)
