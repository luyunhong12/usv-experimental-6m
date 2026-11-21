"""Static displacement/trim from design sections, without a simulation engine."""
import ast,json,math
from pathlib import Path
import numpy as np
from scipy.optimize import root

base=Path(__file__).resolve().parents[1]/'04_依据与数据'
raw=json.loads((base/'艉盖重量复算底数.json').read_text())
def arr(v):return np.array(v,dtype=float)
before=arr(raw['before']['dry'])
original=arr(raw['original']['dry'])
def ledger(key,plug):
    return {'挖去原甲板':-arr(raw['before'][plug]),**{n:arr(v) for n,v in raw[key]['items'].items()}}
oldparts=ledger('first','plug260');parts=ledger('now','plug280')
delta=sum(parts.values());first=sum(oldparts.values())
states=dict(旧预算=original,开孔前=before,第一版=before+first,当前=before+delta)
def loaded(state,L):
    v=state+L*.84*arr([1,.08,0,.23+.125*L/60])
    return v[0],v[1:]/v[0]
source=ast.parse((Path(__file__).with_name('build_hull_full.py')).read_text())
stations=arr(next(ast.literal_eval(n.value) for n in source.body if isinstance(n,ast.Assign) and getattr(n.targets[0],'id','')=='stations_mm'))/1000
def equilibrium(m,cg,dx=.001):
    x=np.arange(-3+dx/2,3,dx)
    k,b,c,w,h=[np.interp(x,stations[:,0],v) for v in stations.T[1:]]
    def residual(guess):
        d,t,s=guess
        level=d+t*x
        area=np.zeros_like(x);ym=area.copy();vm=area.copy()
        # Integrate submerged height and its Y/Z moments along the four straight bottom/side edges.
        for y1,z1,y2,z2 in [(-w,h,-b,c),(-b,c,0,k),(0,k,b,c),(b,c,w,h)]:
            v1=z1-level-s*y1;v2=z2-level-s*y2
            f=np.divide(v1,v1-v2,out=np.zeros_like(x),where=abs(v1-v2)>1e-15).clip(0,1)
            yl=y1+np.where(v1>0,f,0)*(y2-y1);yr=y2-np.where(v2>0,1-f,0)*(y2-y1)
            a=np.minimum(v1,0);bb=np.minimum(v2,0);dy=np.where((v1<0)|(v2<0),yr-yl,0)
            area-=dy*(a+bb)/2
            ym-=dy*(2*yl*a+yl*bb+yr*a+2*yr*bb)/6
            vm-=dy*(a*a+a*bb+bb*bb)/6
        zm=vm+level*area+s*ym; dz=(zm-cg[2]*area).sum()
        return 1000*dx*arr([area.sum(),((x-cg[0])*area).sum()+t*dz,(ym-cg[1]*area).sum()+s*dz])-arr([m,0,0])
    res=root(residual,[.21,.014,-.02],tol=1e-10)
    assert res.success and max(abs(res.fun))<1e-6,res.message
    d,t,s=res.x
    return arr([1000*(d-3*t-.040),1000*d,1000*(d+2.1*t-.080),math.degrees(math.atan(t)),math.degrees(math.atan(s))])
