#!/usr/bin/env python3
"""Export auditable component data, power modes, and aggregate uncertainty bounds."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import argparse,csv,json
from pathlib import Path
from equipment_ledger import load,rows,dry_links,auxiliary_w
from mass_properties import aggregate
from fuel_mass import configuration,tank_properties,combine
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
data=load();(a.output/'component_mass_ledger.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
fields=['id','name','link','mass_kg','mass_range_kg','cg_m','inertia_at_cg_kg_m2','profiles','quality','basis','source','power']
with (a.output/'component_ledger.csv').open('w',newline='',encoding='utf-8-sig') as f:
 writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
 for row in data['parts']:writer.writerow({k:json.dumps(row[k],ensure_ascii=False) if isinstance(row.get(k),(list,dict)) else row.get(k,'') for k in fields})
summary={}
for profile in ['baseline','speed15_concept']:
 summary[profile]={}
 for mass_case in ['light','nominal','heavy']:
  dry=aggregate(rows(profile,mass_case));dry['inertia_kg_m2']=dry.pop('inertia_at_cg_kg_m2')
  summary[profile][mass_case]={'dry':dry,'fuel_states_level_tank':{str(l):combine(dry,tank_properties(configuration(),l)) for l in [0,6,30,60]}}
summary['aux_output_w']={m:auxiliary_w(m) for m in ['normal','survey','peak']};summary['aux_bus_input_w']={m:v/.87 for m,v in summary['aux_output_w'].items()}
summary['uncertainty_note']='All component mass endpoints applied simultaneously; conservative parameter bounds, not statistical confidence intervals. CG installation uncertainty remains an estimate, not a surveyed layout.'
(a.output/'ledger_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
md=['# 逐件质量与辅助用电台账','', '此表从配置自动导出；共 69 条，基础方案使用 60 条。结构件、线束等仍按明确命名的组件组计量，并非逐螺钉制造 BOM。燃油不在干质量条目中重复加入。','', '质量、重心坐标为工程输入；来源与惯性张量完整见同目录 JSON/CSV。单位：kg、m、W；坐标为模型 FLU。质量范围不是统计置信区间。','', '| 条目 | 所属 link | 质量（范围）kg | 重心 m | 正常/采集/峰值辅助 W | 依据 |','|---|---|---|---|---|---|']
for r in data['parts']:
 q=r['power'];power='/'.join(f"{q['idle_w']+(q['rated_w']-q['idle_w'])*q['duty'][m]:.2f}" for m in ['normal','survey','peak']) if q['class']=='auxiliary' else q['class']
 md.append(f"| {r['name']} | {r['link']} | {r['mass_kg']:.3f} ({r['mass_range_kg'][0]:.3f}–{r['mass_range_kg'][1]:.3f}) | {', '.join(f'{v:.3f}' for v in r['cg_m'])} | {power} | {r['basis'].replace('|','/')} |")
md+=['','## 总量与功耗','', '| 配置 | 干质量 kg | 满油质量 kg | 全轻/全重满油 kg |','|---|---:|---:|---|']
for profile in ['baseline','speed15_concept']:
 s=summary[profile];md.append(f"| {profile} | {s['nominal']['dry']['mass_kg']:.3f} | {s['nominal']['fuel_states_level_tank']['60']['mass_kg']:.3f} | {s['light']['fuel_states_level_tank']['60']['mass_kg']:.3f} / {s['heavy']['fuel_states_level_tank']['60']['mass_kg']:.3f} |")
md+=['',f"辅助输出侧功耗：{summary['aux_output_w']}。按 87% DC/DC 效率计入母线需求：{summary['aux_bus_input_w']}。转舵与推进由运行时模型单独计算，启动电池及发电充电另行记账。",'', '功耗是各设备 idle + (rated − idle) × duty 的平均需求；峰值工况按全部额定开启。当前没有逐台设备开关或泵周期模型。', '', '升级电池和电机是仿真方案质量及安装位置假设，不是已采购选型或舱内装配适配证明。相机位置采用传感器参考点附近的估计重心，内部机构、支架壁厚和设备布置仍有估算误差。']
(a.output/'逐件质量与功耗台账.md').write_text('\n'.join(md)+'\n')
print(json.dumps({'mass_full_upgrade':summary['speed15_concept']['nominal']['fuel_states_level_tank']['60']['mass_kg'],'aux_output_w':summary['aux_output_w']},indent=2))
