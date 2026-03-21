import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree
import argparse

def format_xacro_origin(tx, ty, yaw,
                        tz=0.0, roll=0.0, pitch=0.0,
                        sig_digits=8):
    """
    Format an <origin .../> tag with consistent significant digits.

    Args:
        tx, ty, yaw : planar estimate
        tz, roll, pitch : optional (default 0)
        sig_digits : number of significant digits (default 8)

    Returns:
        string
    """

    def fmt(x):
        # format with significant digits, avoid scientific notation
        if abs(x) < 1e-8:
            return "0"
        return f"{x:.{sig_digits}g}"

    xyz = f"{fmt(tx)} {fmt(ty)} {fmt(tz)}"
    rpy = f"{fmt(roll)} {fmt(pitch)} {fmt(yaw)}"

    return f'<origin xyz="{xyz}" rpy="{rpy}"/>'

def trim_by_radius(pcd: o3d.geometry.PointCloud, r=0.2, origin=(0.0, 0.0, 0.0)):
    if len(pcd.points) == 0:
        return o3d.geometry.PointCloud()

    P = np.asarray(pcd.points, dtype=np.float64)
    finite = np.isfinite(P).all(axis=1)

    o = np.asarray(origin, dtype=np.float64).reshape(1, 3)
    d2 = np.sum((P - o) ** 2, axis=1)
    keep = finite & (d2 <= (r * r))

    idx = np.flatnonzero(keep)
    return pcd.select_by_index(idx)


def yaw_to_R2(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def make_T(tx, ty, yaw):
    T = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[0, 3] = tx
    T[1, 3] = ty
    return T


def apply_xy(P, tx, ty, yaw):
    R2 = yaw_to_R2(yaw)
    return (R2 @ P.T).T + np.array([tx, ty])


def best_fit_2d(A, B):
    ca = A.mean(axis=0)
    cb = B.mean(axis=0)
    A0 = A - ca
    B0 = B - cb

    H = A0.T @ B0
    U, _, Vt = np.linalg.svd(H)
    R2 = Vt.T @ U.T

    if np.linalg.det(R2) < 0:
        Vt[-1, :] *= -1
        R2 = Vt.T @ U.T

    t = cb - R2 @ ca
    yaw = np.arctan2(R2[1, 0], R2[0, 0])
    return R2, t, yaw


def icp_planar(src, tgt, max_corr=0.1, max_iter=80):
    src_xy = np.asarray(src.points)[:, :2]
    tgt_xy = np.asarray(tgt.points)[:, :2]

    tree = cKDTree(tgt_xy)

    tx = ty = yaw = 0.0

    for _ in range(max_iter):
        src_tf = apply_xy(src_xy, tx, ty, yaw)

        d, idx = tree.query(src_tf, k=1)
        mask = d < max_corr

        if np.count_nonzero(mask) < 3:
            break

        A = src_tf[mask]
        B = tgt_xy[idx[mask]]

        R2, t, yaw_inc = best_fit_2d(A, B)

        Rcur = yaw_to_R2(yaw)
        tcur = np.array([tx, ty])

        Rnew = R2 @ Rcur
        tnew = R2 @ tcur + t

        yaw = np.arctan2(Rnew[1, 0], Rnew[0, 0])
        tx, ty = tnew

    # final metrics
    src_tf = apply_xy(src_xy, tx, ty, yaw)
    d, _ = tree.query(src_tf, k=1)
    mask = d < max_corr

    rmse = np.sqrt(np.mean(d[mask] ** 2)) if np.any(mask) else np.inf
    fitness = np.sum(mask) / len(src_xy)

    return make_T(tx, ty, yaw), fitness, rmse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--src", nargs="+", required=True,
                    help="one or more source .pcd files")
    ap.add_argument("-t", "--tgt", required=True)
    ap.add_argument("-d", "--display", action="store_true")
    args = ap.parse_args()

    tgt_in = o3d.io.read_point_cloud(args.tgt)
    tgt = trim_by_radius(tgt_in, r=0.3, origin=(0.25, 0, 0))

    txs, tys, yaws = [], [], []
    fitnesses, rmses = [], []

    for src_path in args.src:
        print(f"\nProcessing: {src_path}")

        src_in = o3d.io.read_point_cloud(src_path)
        src = trim_by_radius(src_in, r=0.3, origin=(0.25, 0, 0))

        T, fit, rmse = icp_planar(src, tgt)

        tx, ty = T[0, 3], T[1, 3]
        yaw = np.arctan2(T[1, 0], T[0, 0])

        print("  tx ty yaw:", tx, ty, yaw)
        print("  fitness:", fit, "rmse:", rmse)

        txs.append(tx)
        tys.append(ty)
        yaws.append(yaw)
        fitnesses.append(fit)
        rmses.append(rmse)

    # ---- statistics ----

    tx_mean = np.mean(txs)
    ty_mean = np.mean(tys)

    yaw_mean = np.arctan2(
        np.mean(np.sin(yaws)),
        np.mean(np.cos(yaws))
    )

    if len(txs) == 1:
        tx_std = ty_std = yaw_std = np.inf
    else:
        tx_std = np.std(txs)
        ty_std = np.std(tys)

        # angular std (small-angle assumption is fine here)
        yaw_std = np.std(yaws)

    print("\n=== FINAL ESTIMATE ===")
    print("tx:", tx_mean, "±", tx_std)
    print("ty:", ty_mean, "±", ty_std)
    print("yaw:", yaw_mean, "±", yaw_std)

    print("\n=== FINAL ESTIMATE (xacro format) ===")
    print(format_xacro_origin(tx_mean, ty_mean, yaw_mean))

    print("\nICP stats:")
    print("fitness mean:", np.mean(fitnesses))
    print("rmse mean:", np.mean(rmses))

    # optional visualization (last one only)
    if args.display:
        src_disp = o3d.io.read_point_cloud(args.src[-1])
        src_disp.paint_uniform_color([1, 0, 0])
        tgt.paint_uniform_color([0, 0.6, 1])

        o3d.visualization.draw_geometries([
            src_disp, tgt,
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        ])

        T_final = make_T(tx_mean, ty_mean, yaw_mean)
        src_disp.transform(T_final)

        o3d.visualization.draw_geometries([
            src_disp, tgt,
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        ])


if __name__ == "__main__":
    main()