"""Real ROS services with deterministic send failures, without replaying requests."""
import json
from pathlib import Path
import signal
import subprocess
import sys

import pytest
from litesim3d_interfaces.srv import Step
from test_ros import h, ns
from test_m3_ros import start_world


@pytest.fixture(scope='session')
def response_fault_library(tmp_path_factory):
    if sys.platform != 'linux':
        pytest.skip('Linux LD_PRELOAD fault injection; ordinary ROS regressions still run')
    build = tmp_path_factory.mktemp('response-fault-build')
    source = Path(__file__).with_name('response_fault')
    with (build/'build.log').open('w') as log:
        subprocess.run(['cmake', '-S', str(source), '-B', str(build)],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
        subprocess.run(['cmake', '--build', str(build), '--parallel', '2'],
                       stdout=log, stderr=subprocess.STDOUT, check=True)
    return build/'libresponse_fault.so'


def inject(h, library, mode):
    control = h.directory/'response-fault.txt'
    control.write_text(mode)
    h.env.update(LD_PRELOAD=str(library), LWS_TEST_RESPONSE_SERVICE='/sim/step',
                 LWS_TEST_RESPONSE_FAULT=str(control),
                 LWS_TEST_RESPONSE_TRACE=str(h.directory/'response-sends.jsonl'))
    return control


def sends(h):
    path = h.directory/'response-sends.jsonl'
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def start_single(h):
    h.rsp()
    process = h.sim(start_paused=True)
    h.ready()
    return process


def request(ident, steps=1):
    value = Step.Request()
    value.request_id = ident
    value.steps = steps
    return value


@pytest.mark.parametrize('world', [False, True], ids=['single', 'world'])
def test_response_timeout_recovers_without_repeating_step(h, response_fault_library, world):
    inject(h, response_fault_library, '3')
    start_world(h, 2) if world else start_single(h)
    result = h.step(20, 'once')  # Original six-second client deadline, one request.
    assert result.success and result.time_ns == 100_000_001
    # DDS may deliver the result before the interposer has flushed its trace.
    h.until(lambda: sends(h) and sends(h)[-1]['result'] == 0)
    records = sends(h)
    # Three injected failures are mandatory; real discovery can also time out.
    assert len(records) >= 4 and all(r['result'] == 2 for r in records[:-1])
    assert records[-1]['result'] == 0  # RCL_RET_TIMEOUT, then OK.
    assert len({(r['sequence'], r['guid']) for r in records}) == 1
    assert h.step(20, 'once') == result  # Cached application result remains unchanged.
    following = h.step(1, 'following')
    assert following.success and following.time_ns == 105_000_001
    h.until(lambda: h.messages['clock'] and ns(h.messages['clock'][-1].clock) == following.time_ns)


def test_response_timeout_expires_and_does_not_report_success(h, response_fault_library):
    control = inject(h, response_fault_library, 'always')
    process = start_single(h)
    client = h.node.create_client(Step, '/sim/step')
    try:
        assert client.wait_for_service(timeout_sec=4)
        future = client.call_async(request('expired'))
        h.until(lambda: 'response expired' in (h.directory/'sim.log').read_text(), timeout=4)
        assert not future.done() and process.poll() is None
        count = len(sends(h))
        h.spin(.3)
        assert len(sends(h)) == count  # Deadline stopped the retries and released the result.
        assert ns(h.messages['clock'][-1].clock) == 5_000_001
        h.pause()  # Other services and the executor remain responsive.
        control.unlink()
        assert h.step(1, 'after-expiry').time_ns == 10_000_001
        assert not future.done()  # An expired response cannot leak into a later request.
    finally:
        h.node.destroy_client(client)


def test_response_queue_refuses_overflow_before_executing(h, response_fault_library):
    control = inject(h, response_fault_library, 'always')
    start_single(h)
    client = h.node.create_client(Step, '/sim/step')
    try:
        assert client.wait_for_service(timeout_sec=4)
        futures = []
        for index in range(17):
            futures.append(client.call_async(request(f'queued-{index}')))
            # Deliver each request without overflowing the middleware's request history.
            h.until(lambda: len({r['sequence'] for r in sends(h)}) == index + 1
                    or 'response queue full' in (h.directory/'sim.log').read_text())
        assert 'response queue full' in (h.directory/'sim.log').read_text()
        control.unlink()
        h.until(lambda: all(f.done() for f in futures[:16]))
        assert [f.result().time_ns for f in futures[:16]] == [1 + (i + 1)*5_000_000 for i in range(16)]
        assert not futures[16].done()
        assert h.step(1, 'after-overflow').time_ns == 85_000_001
    finally:
        h.node.destroy_client(client)


def test_response_non_timeout_error_is_not_retried(h, response_fault_library):
    inject(h, response_fault_library, 'error')
    process = start_single(h)
    client = h.node.create_client(Step, '/sim/step')
    try:
        assert client.wait_for_service(timeout_sec=4)
        future = client.call_async(request('fatal'))
        h.until(lambda: process.poll() is not None)
        assert process.returncode != 0 and not future.done()
        assert len(sends(h)) == 1 and sends(h)[0]['result'] == 1
        assert 'injected permanent response failure' in (h.directory/'sim.log').read_text()
    finally:
        h.node.destroy_client(client)


def test_response_pending_does_not_delay_shutdown(h, response_fault_library):
    inject(h, response_fault_library, 'always')
    process = start_single(h)
    client = h.node.create_client(Step, '/sim/step')
    try:
        assert client.wait_for_service(timeout_sec=4)
        future = client.call_async(request('shutdown'))
        h.until(lambda: len(sends(h)) >= 2)
        process.send_signal(signal.SIGINT)
        assert h.wait(process, timeout=2) == 0
        assert not future.done()
    finally:
        h.node.destroy_client(client)
