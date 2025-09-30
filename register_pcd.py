import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s","--src", required=True, help="source .pcd path")
    ap.add_argument("-t", "--tgt", required=True, help="target .pcd path")
    args = ap.parse_args()

    src = o3d.io.read_point_cloud(args.src)
    tgt = o3d.io.read_point_cloud(args.tgt)

    src = src.voxel_down_sample(0.02)
    tgt = tgt.voxel_down_sample(0.02)
    src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.1, 30))

    T_init = np.eye(4)  # your initial guess sensor->scene
    reg = o3d.pipelines.registration.registration_icp(
        src, tgt, max_correspondence_distance=0.08, init=T_init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80)
    )
    T = reg.transformation
    print("ICP fitness:", reg.fitness, "RMSE:", reg.inlier_rmse)
    print("T_sensor_to_scene:\n", T)
    print("xyz/rpy:", T[:3, 3], R.from_matrix(T[:3, :3]).as_euler("xyz", True))


if __name__ == "__main__":
    main()