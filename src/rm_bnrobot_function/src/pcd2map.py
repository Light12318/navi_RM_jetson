#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import cv2
import yaml
import argparse
from pathlib import Path

def pcd2occupancy_grid(pcd_path, save_dir, resolution=0.05, z_min=-0.5, z_max=1.5):
    """
    基于Open3D将PCD点云转换为Nav2兼容的栅格地图（PGM+YAML）
    :param pcd_path: 输入PCD文件路径
    :param save_dir: 输出地图保存目录
    :param resolution: 栅格分辨率（米/像素）
    :param z_min/z_max: 过滤点云的Z轴范围（只保留地面以上的障碍物）
    """
    # 1. 加载PCD点云
    pcd = o3d.io.read_point_cloud(pcd_path)
    if len(pcd.points) == 0:
        raise ValueError("PCD文件无点云数据！")
    points = np.asarray(pcd.points)  # N×3 数组（x,y,z）

    # 2. 过滤点云（Z轴范围）
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    filtered_points = points[mask]
    if len(filtered_points) == 0:
        raise ValueError(f"Z轴范围[{z_min}, {z_max}]内无点云，请调整！")

    # 3. 计算点云边界（确定栅格地图尺寸）
    x_min, y_min = filtered_points[:, 0].min(), filtered_points[:, 1].min()
    x_max, y_max = filtered_points[:, 0].max(), filtered_points[:, 1].max()

    # 4. 栅格化（点云→占据栅格）
    width = int((x_max - x_min) / resolution) + 1
    height = int((y_max - y_min) / resolution) + 1
    # Nav2默认：205=自由空间，0=障碍物，127=未知
    grid = np.ones((height, width), dtype=np.uint8) * 205  

    # 遍历点云，标记障碍物
    for (x, y, _) in filtered_points:
        # 转换为栅格坐标（PGM原点在左上角，翻转Y轴）
        col = int((x - x_min) / resolution)
        row = int((y_max - y) / resolution)
        if 0 <= row < height and 0 <= col < width:
            grid[row, col] = 0  # 障碍物标记为0

    # 5. 保存PGM文件（OpenCV自动处理格式）
    save_dir = Path(save_dir)
    save_dir.mkdir(exist_ok=True, parents=True)
    pgm_path = save_dir / "map.pgm"
    cv2.imwrite(str(pgm_path), grid)

    # 6. 生成Nav2兼容的YAML配置
    yaml_data = {
        "image": pgm_path.name,
        "resolution": resolution,
        "origin": [float(x_min), float(y_min), 0.0],  # 地图原点（x,y,yaw）
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196
    }
    yaml_path = save_dir / "map.yaml"
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.dump(yaml_data, f, sort_keys=False, indent=2)

    # 打印结果
    print(f"✅ 转换完成！")
    print(f"  - 输入PCD：{pcd_path}（点云数量：{len(points)} → 过滤后：{len(filtered_points)}）")
    print(f"  - 输出PGM：{pgm_path}（尺寸：{width}×{height} 像素）")
    print(f"  - 输出YAML：{yaml_path}")
    print(f"  - 地图范围：X[{x_min:.2f}, {x_max:.2f}]m，Y[{y_min:.2f}, {y_max:.2f}]m")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="基于Open3D的PCD转Nav2栅格地图")
    # 新增：支持加载参数文件
    parser.add_argument("--params", type=str, help="参数文件路径（YAML格式，优先级低于命令行参数）")
    # 原有命令行参数（保留，可覆盖参数文件中的值）
    parser.add_argument("--pcd", type=str, help="输入PCD文件路径（覆盖参数文件）")
    parser.add_argument("--save-dir", type=str, help="输出地图保存目录（覆盖参数文件）")
    parser.add_argument("--res", type=float, help="栅格分辨率（覆盖参数文件）")
    parser.add_argument("--z-min", type=float, help="点云Z轴最小值（覆盖参数文件）")
    parser.add_argument("--z-max", type=float, help="点云Z轴最大值（覆盖参数文件）")

    args = parser.parse_args()

    # 加载参数文件（若指定）
    params = {}
    if args.params:
        with open(args.params, "r", encoding="utf-8") as f:
            params = yaml.safe_load(f)
        print(f"📌 加载参数文件：{args.params}")

    # 优先级：命令行参数 > 参数文件 > 默认值
    pcd_path = args.pcd or params.get("pcd_path")
    save_dir = args.save_dir or params.get("save_dir")
    resolution = args.res or params.get("resolution", 0.05)
    z_min = args.z_min or params.get("z_min", -0.5)
    z_max = args.z_max or params.get("z_max", 1.5)

    # 检查必选参数
    if not pcd_path or not save_dir:
        raise ValueError("必须指定 --pcd 和 --save-dir（或通过 --params 参数文件提供）！")

    # 执行转换
    pcd2occupancy_grid(pcd_path, save_dir, resolution, z_min, z_max)
