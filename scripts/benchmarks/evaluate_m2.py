#!/usr/bin/env python3
"""Offline evaluation only. Never publishes TF, commands or ground truth to algorithms.
Input: JSON runs, each with truth/estimate arrays of {stamp_ns, xyz}, completed and timed_out.
Alignment is a frozen initial offset, not a fitted trajectory or scale.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import reference


def trajectory_score(truth, estimate, *, alignment=(0,0,0), tolerance_ns=5_000_000,
                     completed=True, timed_out=False, rmse_limit=.1):
    if not truth:
        return {'pass':False,'reason':'missing ground truth','matched':0,'matching_ratio':0,'rmse_m':None}
    truth_times=[int(p['stamp_ns']) for p in truth]
    if truth_times != sorted(set(truth_times)):
        raise ValueError('truth timestamps must be strictly increasing')
    estimates={int(p['stamp_ns']):p['xyz'] for p in estimate}
    if len(estimates)!=len(estimate):raise ValueError('duplicate estimate timestamp')
    if len(alignment)!=3 or not all(math.isfinite(x) for x in alignment):raise ValueError('invalid alignment')
    errors=[];matched=0;invalid=0
    # Match every required reference sample; missing estimates remain in the denominator.
    times=sorted(estimates)
    used=set()
    for reference in truth:
        t=int(reference['stamp_ns']);index=bisect.bisect_left(times,t)
        candidates=[j for j in (index-1,index) if 0<=j<len(times) and j not in used]
        if not candidates:continue
        near=min(candidates,key=lambda j:abs(times[j]-t))
        if abs(times[near]-t)>tolerance_ns:continue
        used.add(near);value=estimates[times[near]]
        if len(value)!=3 or not all(math.isfinite(v) for v in value):invalid+=1;continue
        target=reference['xyz']
        if len(target)!=3 or not all(math.isfinite(v) for v in target):raise ValueError('invalid truth')
        errors.append(sum((value[i]+alignment[i]-target[i])**2 for i in range(3)))
        matched+=1
    ratio=matched/len(truth)
    rmse=math.sqrt(sum(errors)/matched) if matched else None
    passed=completed and not timed_out and not invalid and ratio>=.99 and rmse is not None and rmse<=rmse_limit
    return {'pass':bool(passed),'matched':matched,'required':len(truth),'matching_ratio':ratio,
            'rmse_m':rmse,'invalid':invalid,'completed':completed,'timed_out':timed_out}


def summarize(runs):
    scores=[trajectory_score(run.get('truth',[]),run.get('estimate',[]),alignment=run.get('alignment',[0,0,0]),
                             completed=run.get('completed',False),timed_out=run.get('timed_out',False)) for run in runs]
    return {'runs':scores,'total':len(scores),'passed':sum(s['pass'] for s in scores),
            'success_rate':sum(s['pass'] for s in scores)/len(scores) if scores else 0}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('input',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();result=summarize(json.loads(args.input.read_text()))
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['total'] and result['passed']==result['total'] else 1

if __name__=='__main__':raise SystemExit(main())
