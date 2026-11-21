import json,sys,inspect,math
from pathlib import Path
import numpy as np
from scipy.optimize import root
sys.path.insert(0,str(Path(__file__).parent))
from stern_trim_budget import equilibrium,stations,arr
b=Path(__file__).resolve().parents[1]/'04_依据与数据'
# Expose the same section integration's residual for local static stiffness only.
s=inspect.getsource(equilibrium).replace('def equilibrium(', 'def static_detail(')
s=s[:s.index('    return arr(')]+'''    eps=math.radians(.25)
    moments=[]
    for phi in [math.atan(s)-eps,math.atan(s)+eps]:
        slope=math.tan(phi)
        v=root(lambda v:residual([v[0],v[1],slope])[:2],[d,t])
        assert v.success
        moments.append(residual([*v.x,slope])[2])
    return dict(waterplane_m=[d,t,s],transverse_local_stiffness_kg_m_per_rad=(moments[1]-moments[0])/(2*eps),effective_initial_GM_m=(moments[1]-moments[0])/(2*eps)/m)
'''
exec(s)
r=json.loads((b/'冷却联锁重量与重心.json').read_text());old=json.loads((b/'集中收尾重量与重心.json').read_text())
for k,st in r['states'].items():
 st['static']=static_detail(st['mass_kg'],st['cg_m']);st['draft_trim_heel']=equilibrium(st['mass_kg'],st['cg_m']).tolist();st['previous_draft_trim_heel']=equilibrium(old['states'][k]['mass_kg'],old['states'][k]['cg_m']).tolist()
r['static_note']='仅静水船壳截面积分及小角度恢复力斜率，无仿真。GM未扣液舱自由液面、甲板积水，不作为极限风浪或不翻船保证。'
(b/'冷却联锁重量与重心.json').write_text(json.dumps(r,ensure_ascii=False,indent=2))
print(json.dumps(r['states'],ensure_ascii=False,indent=2))
