import importlib.util
from pathlib import Path
import sys
import subprocess
import pytest
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
def load(name):
    group='benchmarks' if name=='evaluate_m2' else 'checks'
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/group/f'{name}.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def test_E2E08_evaluator_keeps_failures_and_analytic_offsets():
    e=load('evaluate_m2');truth=[{'stamp_ns':1+i*20_000_000,'xyz':[i*.01,0,0]} for i in range(100)]
    zero=e.trajectory_score(truth,truth);assert zero['pass'] and zero['rmse_m']==0
    shifted=[{**p,'xyz':[p['xyz'][0]+.3,.4,0]} for p in truth]
    offset=e.trajectory_score(truth,shifted);assert not offset['pass'] and abs(offset['rmse_m']-.5)<1e-12
    assert e.trajectory_score(truth,shifted,alignment=[-.3,-.4,0])['pass']
    assert not e.trajectory_score(truth,truth[:98])['pass']
    assert not e.trajectory_score(truth,truth,timed_out=True)['pass']
    divergent=[{**p,'xyz':[float('nan'),0,0]} for p in truth]
    assert not e.trajectory_score(truth,divergent)['pass']
    result=e.summarize([{'truth':truth,'estimate':truth,'completed':True},
                        {'truth':truth,'estimate':truth[:98],'completed':True},
                        {'truth':truth,'estimate':divergent,'completed':True},
                        {'truth':truth,'estimate':truth,'completed':True,'timed_out':True}])
    assert result['total']==4 and result['passed']==1 and result['success_rate']==.25


def test_DEL04_runner_propagates_assertions_and_timeout(tmp_path):
    r=load('run_checked')
    for name,code,timeout in [('assertion','assert False',5),('timeout','import time;time.sleep(10)',.1),('pass','pass',5)]:
        xml=tmp_path/(name+'.xml');log=tmp_path/(name+'.log')
        result=r.run([sys.executable,'-c',code],timeout,xml,log)
        assert (result==0)==(name=='pass')
        assert bool(ET.parse(xml).find('.//failure') is not None)==(name!='pass')


def test_T_URD_18_xacro(tmp_path):
    spec=importlib.util.spec_from_file_location("prepare_xacro",ROOT/"scripts/prepare_xacro.py")
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    share=Path(get_package_share_directory("litesim3d_core"))
    a=module.expand(share/"urdf/r04.xacro",tmp_path/"a.urdf",["length:=0.8"])
    b=module.expand(share/"urdf/r04.xacro",tmp_path/"b.urdf",["length:=1.2"])
    assert a["expanded_sha256"]!=b["expanded_sha256"] and a["includes"]
    assert ET.parse(tmp_path/"a.urdf").find(".//box").get("size").startswith("0.8")
    assert ET.parse(tmp_path/"b.urdf").find(".//box").get("size").startswith("1.2")
    bad=tmp_path/"bad.xacro"
    bad.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:include filename="missing.xacro"/></robot>')
    with pytest.raises(subprocess.CalledProcessError):module.expand(bad,tmp_path/"bad.urdf")
    assert not (tmp_path/"bad.urdf").exists()


