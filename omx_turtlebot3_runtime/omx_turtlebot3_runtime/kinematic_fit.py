"""Offline provisional motor-sign/zero fit; never writes an execution config.

Shoulder zero and camera mount remain assumptions, not commissioned facts.
Input is the explicit stationary-marker, empty-arm calibration capture.
"""
import argparse
import itertools
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def camera(q, signs, offsets):
    yaw=q[0];a,b,c=np.asarray(q[1:])*signs+np.r_[0.,offsets]
    rz=Rotation.from_rotvec([0.,0.,yaw]).as_matrix()
    ry=lambda v:Rotation.from_rotvec([0.,v,0.]).as_matrix()
    mount=np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
    p=rz@(ry(a)@np.array([.024,0.,.128])+ry(a+b)@np.array([.124,0.,0.])+
           ry(a+b+c)@np.array([.020,0.,.065]))
    return p,rz@ry(a+b+c)@mount


def fit(path, marker):
    groups={}
    for line in path.read_text().splitlines():
        x=json.loads(line)
        if x['event']!='SAMPLE':continue
        detection=next((d for d in x['detections'] if d['id']==marker),None)
        if detection is None:continue
        stage=int(x['stage'].split()[1]);groups.setdefault(stage,[]).append((x,detection))
    if not all(len(groups.get(i,[]))>=4 for i in range(7)):
        raise ValueError('Need at least four valid stationary samples at all seven stages: '+str({k:len(v) for k,v in groups.items()}))
    samples=[]
    for k,rows in sorted(groups.items()):
        q=np.median([x['q'] for x,d in rows],axis=0)
        t=np.median([d['t'] for x,d in rows],axis=0)
        r=Rotation.from_matrix([d['r'] for x,d in rows]).mean().as_matrix()
        samples.append((q,t,r))
    def poses(signs,offsets):
        position=[];rotation=[]
        for q,t,r in samples:
            p,cam=camera(q,signs,offsets);position.append(p+cam@t);rotation.append(cam@r)
        return np.asarray(position),Rotation.from_matrix(rotation)
    def residual(signs,offsets):
        p,r=poses(signs,offsets)
        return np.r_[(p-p.mean(axis=0)).ravel(),(r.mean().inv()*r).as_rotvec().ravel()*.08]
    candidates=[]
    for signs in itertools.product([-1.,1.],repeat=3):
        best=None
        for guess in itertools.product([-3.,0.,3.],repeat=2):
            result=least_squares(lambda z:residual(signs,z),guess,
                bounds=([-np.pi,-np.pi],[np.pi,np.pi]),loss='soft_l1',f_scale=.002,max_nfev=100)
            score=float(np.mean(residual(signs,result.x)**2))
            if best is None or score<best[0]:best=score,result
        score,result=best;p,r=poses(signs,result.x)
        candidates.append(dict(signs_joint2_3_4=list(signs),offsets_joint3_4_rad=result.x.tolist(),
            score=score,translation_rms_m=float(np.sqrt(np.mean(np.sum((p-p.mean(axis=0))**2,axis=1)))),
            orientation_rms_rad=float(np.sqrt(np.mean(np.sum((r.mean().inv()*r).as_rotvec()**2,axis=1))))))
    candidates.sort(key=lambda x:x['score'])
    return dict(provisional=True,automatically_applied=False,reference_id=marker,
        assumptions=['shoulder_zero_rad=0','configured_camera_mount','marker_stationary'],
        counts={k:len(v) for k,v in groups.items()},candidates=candidates)


def main():
    p=argparse.ArgumentParser();p.add_argument('samples',type=Path);p.add_argument('--reference-id',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    report=fit(args.samples,args.reference_id);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['candidates'][:2],indent=2))

if __name__=='__main__':main()
