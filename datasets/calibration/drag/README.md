# 实船阻力标定入口

目前没有实船数据；模板只有列名，不填入伪造测量值。

采集平静水面近似直航工况，记录时间（秒）、相对水流的航速（m/s）、已知纵向外力（N）、转角（rad）与偏航角速度（rad/s）。稳态拖曳试验更适合区分船体阻力和未知推进器推力。记录实测排水质量、吃水、风流条件与仪器校准。

`python3 scripts/fit_drag.py calibration/drag/实测.csv --mass-kg 实测质量 --data-kind measured --output calibration/drag/拟合.json`

结果需要审核和复现实验后才能替换 `trial_load.json` 的线性阻尼、`propulsion.json` 的二次阻尼。不提供实测数据时，只能验证算法和仿真一致性，不能完成实船标定。动态辨识还受附加质量影响，不能把误差全部归入阻力。
