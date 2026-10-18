#!/usr/bin/env python3
"""Deterministic shared F09 room for the frozen four-robot B3 workload."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import yaml
from fixtures.generate_m2_fixtures import mesh


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--resources',type=Path,required=True)
    args=parser.parse_args();directory=args.output.resolve();directory.mkdir(parents=True,exist_ok=True)
    info=mesh(directory/'F09.stl',100,100)
    config=yaml.safe_load((args.resources/'config/m3_robot.yaml').read_text())
    config['robot']['urdf']='package://litesim3d_core/urdf/r01.urdf'
    config['entities']=[{'id':'room','geometry':{'type':'mesh','uri':str(directory/'F09.stl'),
                                              'scale':[1,1,1]},'support':True}]
    (directory/'B3-robot.yaml').write_text(yaml.safe_dump(config))
    world={'schema_version':1,'config':'B3-robot.yaml','robots':[
        {'id':f'r{i}','xyz':[x,y,0]} for i,(x,y) in enumerate([(-3,-3),(3,-3),(-3,3),(3,3)])]}
    (directory/'B3-world.yaml').write_text(yaml.safe_dump(world))
    info['config_sha256']=hashlib.sha256((directory/'B3-world.yaml').read_bytes()).hexdigest()
    (directory/'fixture.json').write_text(json.dumps(info,indent=2))


if __name__=='__main__':main()
