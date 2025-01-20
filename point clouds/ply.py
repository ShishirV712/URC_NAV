import open3d as o3d

file_path = "/home/shishir/urc_ws/src/point clouds/cloud.ply"

try:
    # Attempt to load with Open3D
    mesh = o3d.io.read_triangle_mesh(file_path)
    if mesh.is_empty():
        print("Mesh is empty or invalid")
    else:
        o3d.visualization.draw_geometries([mesh])
        print("Mesh successfully loaded and visualized")
except Exception as e:
    print(f"Error loading file with Open3D: {e}")

