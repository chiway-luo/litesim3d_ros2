#!/usr/bin/env python3
"""Static M2 documentation, schema, source-fingerprint and evidence audit."""
import ast
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR, resolve_reference, reference, python_sources
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT = REPO_ROOT;OUT=EVIDENCE_DIR / 'm2'

def main():
    checks=[]
    def check(name,passed):checks.append({'check':name,'passed':bool(passed)})
    for document in [ROOT/'README.md',*sorted((ROOT/'docs').rglob('*.md'))]:
        for link in re.findall(r'\[[^\]]*\]\(([^)]+)\)',document.read_text()):
            if re.match(r'\w+://',link) or link.startswith(('#','mailto:')):continue
            local=link.split('#')[0].strip('<>')
            if local:check(f'local link {document.name}: {local}',(document.parent/local).exists())
    for source in python_sources():
        try:ast.parse(source.read_text());check('Python syntax '+reference(source),True)
        except SyntaxError:check('Python syntax '+reference(source),False)
    for script in ROOT.glob('scripts/*.sh'):
        check('shell syntax '+script.name,subprocess.run(['bash','-n',str(script)],capture_output=True).returncode==0)
    index=json.loads((OUT/'test-index.json').read_text())
    check('52 unique M2 IDs',len(index['cases'])==len({c['id'] for c in index['cases']})==52)
    for case in index['cases']:
        check(case['id']+' requirement mapping',bool(case['requirements']))
        check(case['id']+' executable/evidence mapping',bool(case['runs']))
        if case['status']=='PASS':
            for run in case['runs']:check(case['id']+' evidence exists: '+run['evidence'],(resolve_reference(run['evidence'])).exists())
        if case['status']=='DEFERRED':
            allowed=case['id'].startswith('T-E2E-') or (case['id']=='T-ROB-01' and index['platform_decision'].get('eight_hour_soak_deferred'))
            check(case['id']+' not counted PASS',allowed and bool(case['note']))
    manifest=json.loads((OUT/'B1-soak-8h/rtf-1-gui-0-0/manifest.json').read_text())
    check('running/tested core matches current source',index['current_core_source_sha256']==manifest['core_source_sha256'])
    check('M2 platform is native Humble',index['platform']=='macOS / ROS 2 Humble' and not index['platform_decision']['linux_required'])
    config=(ROOT/'litesim3d_core/resources/config/m2.yaml').read_text()
    for field in ['scan_mode: rolling','rate_hz: 200','topic: imu/data_raw','threads: 4','max_slope_deg: 20']:
        check('M2 documented default '+field,field in config)
    check('reset IDL installed','srv/Reset.srv' in (ROOT/'litesim3d_interfaces/CMakeLists.txt').read_text())
    check('supervisor installed','configure_file(scripts/sim_session sim_session @ONLY)' in (ROOT/'litesim3d_ros2/CMakeLists.txt').read_text())
    failed=[c['check'] for c in checks if not c['passed']]
    result={'status':'FAIL' if failed else 'PASS','checks':checks,'failures':failed,
            'scope':'Static consistency only; performance/soak must independently finish and pass.'}
    (OUT/'docs-check.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    suite=ET.Element('testsuite',name='M2Docs',tests='1',failures=str(int(bool(failed))))
    case=ET.SubElement(suite,'testcase',name='documentation_and_evidence')
    if failed:ET.SubElement(case,'failure',message='; '.join(failed))
    ET.ElementTree(suite).write(OUT/'docs.xml',encoding='utf-8',xml_declaration=True)
    print(json.dumps({'status':result['status'],'checks':len(checks),'failures':failed},ensure_ascii=False))
    return int(bool(failed))

if __name__=='__main__':raise SystemExit(main())
