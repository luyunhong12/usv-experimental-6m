"""为船舶场景设置显式的刚体接触数值策略，不改变实体几何或材料质量。"""

import xml.etree.ElementTree as E


def configure_contact_physics(world):
    """原地统一生产场景的接触检测、步长和误差修正设置。

    Args:
        world: 含 physics 和原生 Physics 系统插件的 SDF world 元素。

    Raises:
        ValueError: 场景缺少物理配置或系统插件。
    """
    physics = world.find("physics")
    plugin = world.find("plugin[@name='gz::sim::systems::Physics']")
    if physics is None or plugin is None:
        raise ValueError("World requires physics settings and Physics plugin")
    physics.set("name", "1ms")
    physics.find("max_step_size").text = "0.001"
    dart = physics.find("dart")
    if dart is None:
        dart = E.SubElement(physics, "dart")
    detector = dart.find("collision_detector")
    if detector is None:
        detector = E.SubElement(dart, "collision_detector")
    detector.text = "ode"
    plugin.set("filename", "libUsvPhysics.so")
    # DART 6.13.2 不支持逐接触 ERP；此配置作用于该进程的 DART 接触。
    for key, value in {
        "rigid_contact_erp": "0.2",
        "rigid_contact_max_correction_velocity": "0.1",
    }.items():
        element = plugin.find(key)
        if element is None:
            element = E.SubElement(plugin, key)
        element.text = value
