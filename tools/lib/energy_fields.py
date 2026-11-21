"""定义能源与质量遥测字段索引。

Version 1 Gazebo Double_V schemas; SI except documented Ah/Wh/litre fields.
"""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys

# 项目脚本可从任意工作目录启动；建模工具保持在共享库之前。
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/lib"
    ),
)
sys.path.insert(
    0,
    str(
        next(
            p
            for p in _ProjectPath(__file__).resolve().parents
            if (p / "project.toml").is_file()
        )
        / "tools/modeling"
    ),
)

TRUTH = "time,soc,voltage_v,current_a,temperature_c,fuel_l,fuel_l_h,motor_w,aux_w,charger_dc_w,generator_ac_w,generator_state,starter_soc,discharge_open,low_soc,low_fuel,fault,battery_wh,motor_wh,charger_wh,resistive_loss_wh,balance_w,removed_fuel_kg,unserved_w,motor_budget_w".split(
    ","
)
SENSORS = (
    "time,sample_time,valid,voltage_v,current_a,soc,fuel_l,temperature_c,age_s".split(
        ","
    )
)
GENERATOR = {0: "OFF", 1: "STARTING", 2: "WARMING", 3: "RUNNING", 4: "FAULT"}

# Native DART readback plus link-composite properties in the hull FLU frame.
MASS = "time,fuel_l,hull_mass,hull_cgx,hull_cgy,hull_cgz,hull_ixx,hull_iyy,hull_izz,hull_ixy,hull_ixz,hull_iyz,ship_mass,ship_cgx,ship_cgy,ship_cgz,ship_ixx,ship_iyy,ship_izz,ship_ixy,ship_ixz,ship_iyz".split(
    ","
)
