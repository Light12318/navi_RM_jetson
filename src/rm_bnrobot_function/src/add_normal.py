#!/usr/bin/env python3
import open3d as o3d
import os

# 1. 填入你原来的地图名字（确保这个文件和本脚本在同一个文件夹）
input_map = "/home/mage/navigation/nav_RM_4/test.pcd"  
output_map = "map_with_normals.pcd"
path_input=os.path.join("/home/mage/navigation/nav_RM_4",input_map)
print(f"正在读取地图: {input_map} ...")
pcd = o3d.io.read_point_cloud(input_map)

# 检查地图是不是空的（排查之前的 No Data 报错）
if pcd.is_empty():
    print("致命错误：你的地图是空的！没有任何点！请检查建图或降采样过程！")
    exit()

print(f"地图读取成功！包含 {len(pcd.points)} 个点。")
print("正在拼命计算法向量...")

# 2. 计算法向量 (Radius参数代表搜索半径，max_nn代表最多参考周围的几个点)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

# 3. 保存新地图
o3d.io.write_point_cloud(output_map, pcd)
print(f"带法向量的新地图已保存为: {output_map}")