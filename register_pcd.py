import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import argparse

def trim_by_radius(pcd: o3d.geometry.PointCloud, r=0.2, origin=(0.0, 0.0, 0.0)):
    if len(pcd.points) == 0:
        return o3d.geometry.PointCloud()  # nothing to do

    P = np.asarray(pcd.points, dtype=np.float64)           # (N,3)
    finite = np.isfinite(P).all(axis=1)                    # drop NaN/Inf rows

    o = np.asarray(origin, dtype=np.float64).reshape(1,3)
    d2 = np.sum((P - o)**2, axis=1)                        # squared distance
    keep = finite & (d2 <= (r * r))

    idx = np.flatnonzero(keep)
    if idx.size == 0:
        # helpful diagnostics
        with np.errstate(invalid='ignore'):
            norms = np.linalg.norm(P, axis=1)
        print(f"[trim_by_radius] kept 0; min|p|={np.nanmin(norms):.3f} m, "
              f"max|p|={np.nanmax(norms):.3f} m, r={r:.3f} m")
        return o3d.geometry.PointCloud()

    return pcd.select_by_index(idx)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s","--src", required=True, help="source .pcd path")
    ap.add_argument("-t", "--tgt", required=True, help="target .pcd path")
    ap.add_argument("-d", "--display", action="store_true", help="display point clouds")
    args = ap.parse_args()

    src_in = o3d.io.read_point_cloud(args.src)
    tgt_in = o3d.io.read_point_cloud(args.tgt)

    src = trim_by_radius(src_in,r=0.3,origin=(0.25,0,0))
    tgt = trim_by_radius(tgt_in,r=0.3,origin=(0.25,0,0))

    #src = src.voxel_down_sample(0.02)
    #tgt = tgt.voxel_down_sample(0.02)
    src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.1, 30))
    tgt.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.1, 30))

    T_init = np.eye(4)  # your initial guess sensor->scene
    reg = o3d.pipelines.registration.registration_icp(
        src, tgt, max_correspondence_distance=0.1, init=T_init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80)
    )
    # reg = o3d.pipelines.registration.registration_icp(
    #     src, tgt, max_correspondence_distance=0.02, init=T_init,
    #     estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    #     criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=5000)
    # )
    T = reg.transformation
    print("ICP fitness:", reg.fitness, "RMSE:", reg.inlier_rmse)
    print("T_sensor_to_scene:\n", T)
    Tc = np.copy(T)
    print("xyz/rpy:", Tc[:3, 3], R.from_matrix(Tc[:3, :3]).as_euler("xyz", False))

    if args.display:
        # Give distinct colors (skip if you want to keep original RGB)
        src.paint_uniform_color([1, 0, 0])      # red
        tgt.paint_uniform_color([0, 0.6, 1])    # blue

        o3d.visualization.draw_geometries([
            src, tgt,
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        ])

        srcx = src.transform(T)
        o3d.visualization.draw_geometries([
            srcx, tgt,
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        ])



if __name__ == "__main__":
    main()