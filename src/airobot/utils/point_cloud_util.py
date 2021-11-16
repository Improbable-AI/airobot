TORCH_AVAILABLE = True
try:
    import torch
except ImportError:
    TORCH_AVAILABLE = False


def plot_pointcloud(points=None, color=None, coord_size=0.3):
    import open3d
    if TORCH_AVAILABLE and isinstance(points, torch.Tensor):
        points = points.detach().cpu()
    if TORCH_AVAILABLE and isinstance(color, torch.Tensor):
        color = color.detach().cpu()
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(points)
    if color is not None:
        if color.max() > 1.01:
            color = color / 255.0
        pcd.colors = open3d.utility.Vector3dVector(color)
    coor = open3d.geometry.TriangleMesh.create_coordinate_frame(coord_size)
    open3d.visualization.draw_geometries([pcd, coor])
