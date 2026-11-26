"""由静水浸没横剖面估算横向流体惯性；不替代自由面势流或实测标定。"""

import numpy as np


def wet_section(triangles, x, water_z):
    """截取纵坐标处的湿剖面，返回吃水和湿宽。

    Args:
        triangles: 船体FLU三角网格，形状(N,3,3)，单位m。
        x: 截面纵坐标，m；应避开网格端点。
        water_z: 船体坐标水平静水面的高度，m。

    Returns:
        (吃水, 湿宽)，单位m；干剖面返回两个零。

    Raises:
        ValueError: 输入非有限、网格形状不正确或切面与面片共面。
    """
    mesh = np.asarray(triangles, dtype=float)
    if (
        mesh.ndim != 3
        or mesh.shape[1:] != (3, 3)
        or not np.isfinite(mesh).all()
        or not np.isfinite([x, water_z]).all()
    ):
        raise ValueError("需要有限的(N,3,3)网格和剖面位置")
    points = []
    for triangle in mesh:
        distance = triangle[:, 0] - x
        if np.all(np.abs(distance) < 1e-12):
            raise ValueError("剖面与面片共面，请改变剖分数量或位置")
        intersections = []
        for i, a in enumerate(triangle):
            b = triangle[(i + 1) % 3]
            da, db = distance[i], distance[(i + 1) % 3]
            if abs(da) < 1e-12:
                intersections.append(a)
            if da * db < 0:
                intersections.append(a + (b - a) * da / (da - db))
        if len(intersections) < 2:
            continue
        unique = np.unique(np.round(intersections, 12), axis=0)
        if len(unique) != 2:
            continue
        a, b = unique
        points.extend(p for p in (a, b) if p[2] <= water_z)
        if (a[2] < water_z) != (b[2] < water_z):
            points.append(a + (b - a) * (water_z - a[2]) / (b[2] - a[2]))
    if not points:
        return 0.0, 0.0
    points = np.asarray(points)
    return max(0.0, min(water_z, points[:, 2].max()) - points[:, 2].min()), np.ptp(
        points[:, 1]
    )


def lateral_inertia(triangles, water_z, reference_x, density=1000.0, strips=240):
    """按半椭圆镜像近似积分横移/偏航附加惯性，保留非对角项。

    截面单位长度横向附加质量采用0.5*rho*pi*T²；是静水几何
    工程近似，不计算频率依赖、波浪辐射或纵向附加质量。

    Args:
        triangles: 船体FLU网格，形状(N,3,3)，单位m。
        water_z: 水平水面在船体坐标的高度，m。
        reference_x: 矩阵参考点纵坐标，m。
        density: 正的水密度，kg/m³。
        strips: 正整数等宽中点剖分数量。

    Returns:
        含剖面数据、横移/偏航2×2矩阵和惯性中心的字典。

    Raises:
        ValueError: 输入非法、无浸水剖面或船长为零。
    """
    mesh = np.asarray(triangles, dtype=float)
    if (
        mesh.ndim != 3
        or mesh.shape[1:] != (3, 3)
        or mesh.size == 0
        or not np.isfinite(mesh).all()
        or not np.isfinite([water_z, reference_x, density]).all()
        or density <= 0
        or not isinstance(strips, int)
        or isinstance(strips, bool)
        or strips < 1
    ):
        raise ValueError("非法剖面积分输入")
    start, end = mesh[:, :, 0].min(), mesh[:, :, 0].max()
    if end <= start:
        raise ValueError("船长必须为正")
    if water_z >= mesh[:, :, 2].max():
        raise ValueError("半剖面近似只适用于穿过船体的水面")
    dx = (end - start) / strips
    sections = np.array(
        [
            [x, *wet_section(mesh, x, water_z)]
            for x in start + (np.arange(strips) + 0.5) * dx
        ]
    )
    masses = 0.5 * density * np.pi * sections[:, 1] ** 2 * dx
    total = masses.sum()
    if total <= 0:
        raise ValueError("无浸水剖面")
    arms = sections[:, 0] - reference_x
    coupling = float(masses @ arms)
    yaw = float(masses @ (arms**2))
    return {
        "sections_x_draft_beam_m": sections.tolist(),
        "strip_width_m": float(dx),
        "sway_yaw_matrix": [[float(total), coupling], [coupling, yaw]],
        "inertia_center_x_m": float(masses @ sections[:, 0] / total),
        "side_area_m2": float(sections[:, 1].sum() * dx),
    }
