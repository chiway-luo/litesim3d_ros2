#!/usr/bin/env python3
"""Collect current Clang/Python profiles and apply the unchanged project thresholds."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT
import shutil
import subprocess
import coverage
from checks.check_coverage import check, expected_sources

ROOT = REPO_ROOT


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--prefix',type=Path,required=True)
    parser.add_argument('--python-profile-source',type=Path,action='append',default=[])
    args=parser.parse_args();directory=args.directory.resolve();prefix=args.prefix.resolve()
    def tool(name):
        return subprocess.check_output(['xcrun','--find',name],text=True).strip() if sys.platform=='darwin' else shutil.which(name)
    profdata,cov=tool('llvm-profdata'),tool('llvm-cov')
    def merge(name,files):
        assert files,f'no real profiles for {name}'
        output=directory/(name+'.profdata')
        subprocess.run([profdata,'merge','-sparse',*map(str,files),'-o',str(output)],check=True)
        return output
    core=merge('core',sorted(directory.glob('core-*.profraw')))
    bridge=merge('bridge',sorted(p for p in directory.glob('*.profraw') if not p.name.startswith('core-')))
    inventory=expected_sources('M3')
    core_sources=[str(ROOT/path) for path in inventory['core']]
    test=prefix.parent/'build/litesim3d_core/sim_core_tests'
    commands={
        'core-coverage.json':[cov,'export',str(test),f'-instr-profile={core}',*core_sources],
        'bridge-coverage.json':[cov,'export',str(prefix/'lib/litesim3d_ros2/sim_node'),
            *[f'-object={prefix}/lib/litesim3d_ros2/{name}' for name in ['sim_world_node','sim_ouster_adapter','sim_cloud_probe']],
            f'-object={prefix}/bin/sim_headless',f'-instr-profile={bridge}',
            *[str(ROOT/path) for path in inventory['bridge']]]}
    with (directory/'llvm-diagnostics.txt').open('w') as errors:
        for name,command in commands.items():
            with (directory/name).open('w') as output:subprocess.run(command,stdout=output,stderr=errors,check=True)
    config=directory/'coveragerc'
    # Coverage 7.15 reporting commands auto-combine and can delete raw inputs.
    # Use the API to preserve inputs and append to previously collected data.
    python=coverage.Coverage(config_file=str(config))
    python.load()
    python.combine(data_paths=[str(directory),*map(str,args.python_profile_source)],strict=False,keep=True)
    python.save()
    python.json_report(outfile=str(directory/'python-coverage.json'),pretty_print=True)
    result=check(directory,'M3')
    print(json.dumps({'status':result['status'],'totals':result['totals'],'failures':result['failures']},indent=2))
    return int(result['status']!='PASS')


if __name__=='__main__':raise SystemExit(main())
