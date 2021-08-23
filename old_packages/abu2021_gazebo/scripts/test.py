import open3d as o3d
import sys
sys.path.insert(0, '.')

mesh = o3d.io.read_triangle_mesh('../meshes/arrow/arrow.obj')
o3d.visualization.draw_geometries([mesh])
