#!/usr/bin/env python3
"""Deterministic triangulated open rooms, F09=100k and F10=1M triangles."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import struct
import yaml


def mesh(path,nx,ny):
    count=10*nx*ny
    with path.open('wb') as out:
        out.write(b'LWS M2 F09/F10 deterministic room v1'.ljust(80,b'\0'))
        out.write(struct.pack('<I',count))
        faces=[((-10,-10,0),(20,0,0),(0,20,0)),
               ((-10,-10,0),(20,0,0),(0,0,4)),
               ((10,10,0),(-20,0,0),(0,0,4)),
               ((-10,10,0),(0,-20,0),(0,0,4)),
               ((10,-10,0),(0,20,0),(0,0,4))]
        for origin,u,v in faces:
            def point(i,j):return tuple(origin[k]+u[k]*i/nx+v[k]*j/ny for k in range(3))
            for i in range(nx):
                for j in range(ny):
                    a,b,c,d=point(i,j),point(i+1,j),point(i+1,j+1),point(i,j+1)
                    for triangle in ((a,b,c),(a,c,d)):
                        out.write(struct.pack('<12fH',0,0,0,*triangle[0],*triangle[1],*triangle[2],0))
    return {'triangles':count,'bytes':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--resources',type=Path,required=True);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    info={}
    for name,nx,ny,rings,columns,rate in [('F09',100,100,32,1024,10),('F10',500,200,64,2048,20)]:
        path=args.output/(name+'.stl');info[name]=mesh(path,nx,ny)
        data=yaml.safe_load((args.resources/'config/m2.yaml').read_text())
        data['robot']['urdf']=str((args.resources/'urdf/r01.urdf').resolve())
        data['entities']=[{'id':'room','geometry':{'type':'mesh','uri':str(path.resolve()),'scale':[1,1,1]},'support':True}]
        data['lidars'][0].update(rings=rings,columns=columns,rate_hz=rate,vertical_angles_deg=[-15+30*i/(rings-1) for i in range(rings)])
        config=args.output/(name+'.yaml');config.write_text(yaml.safe_dump(data))
        info[name]['config_sha256']=hashlib.sha256(config.read_bytes()).hexdigest()
    (args.output/'fixtures.json').write_text(json.dumps(info,indent=2))

if __name__=='__main__':main()
