# save_pcd_tf.py  (ROS 2 Humble)

# usage:
# python3 save_pcd_tf.py -t /camera/cloud -F base_footprint -o cloud_sim_bin.pcd --binary --latest

import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.task import Future
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

def write_pcd_xyz_ascii(path, pts_xyz: np.ndarray):
    N = pts_xyz.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {N}\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {N}\nDATA ascii\n"
    )
    with open(path, "w", newline="\n") as f:
        f.write(header)
        np.savetxt(f, pts_xyz, fmt="%.6f")

def write_pcd_xyz_binary(path, pts_xyz: np.ndarray):
    N = pts_xyz.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {N}\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {N}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.asarray(pts_xyz, dtype=np.float32, order="C").tobytes())

def quat_to_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=np.float64)

class OneShotPCDSaver(Node):
    def __init__(self, topic, target_frame, outfile, timeout_s, use_latest, binary, fallback_target):
        super().__init__("pcd_oneshot_saver_tf")
        self.done = False
        self.topic = topic
        self.target = target_frame
        self.fallback_target = fallback_target
        self.outfile = outfile
        self.timeout = Duration(seconds=timeout_s)
        self.use_latest = use_latest
        self.binary = binary

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        self.msg = None
        #self.done = Future()
        self.sub = self.create_subscription(PointCloud2, topic, self._on_cloud, 10)
        # Try processing at 20 Hz until both the message and TF are ready
        self.timer = self.create_timer(0.05, self._try_process)

    def _on_cloud(self, msg: PointCloud2):
        self.msg = msg  # store the first one; we only save once

    def _frame_exists(self, frame: str):
        try:
            frames = self.tf_buffer.all_frames_as_string()
            return frame in frames
        except Exception:
            return True  # some distros don't implement listing; don't block on it

    def _lookup_transform(self, target: str, src: str, stamp: Time):
        # Prefer timestamped lookup unless --latest
        when = Time() if self.use_latest else stamp
        # Try exact/timeout
        if self.tf_buffer.can_transform(target, src, when, self.timeout):
            return self.tf_buffer.lookup_transform(target, src, when)
        # Fallback: latest if allowed
        if not self.use_latest and self.tf_buffer.can_transform(target, src, Time(), self.timeout):
            self.get_logger().warn("Timestamped TF unavailable; using latest transform.")
            return self.tf_buffer.lookup_transform(target, src, Time())
        raise RuntimeError(f"Transform {target} <- {src} not available yet")

    def _try_process(self):
        if self.msg is None:
            return  # no cloud yet

        src = self.msg.header.frame_id.lstrip('/')
        target = self.target

        # Optional fallback target (e.g., base_link) if requested
        if self.fallback_target and not self._frame_exists(target):
            self.get_logger().warn(f"Target frame '{target}' not in TF; falling back to '{self.fallback_target}'")
            target = self.fallback_target

        try:
            t = self._lookup_transform(target, src, Time.from_msg(self.msg.header.stamp))
        except Exception as e:
            # Still warming up TF; keep waiting
            self.get_logger().debug(f"TF not ready: {e}")
            return

        # Transform points
        pts_iter = point_cloud2.read_points(self.msg, field_names=("x","y","z"), skip_nans=True)
        P = np.array(pts_iter.tolist(), dtype=np.float64)
        if P.size == 0:
            self.get_logger().warn("Cloud has no valid xyz points; waiting for next message…")
            self.msg = None
            return

        R = quat_to_R(t.transform.rotation.x, t.transform.rotation.y,
                      t.transform.rotation.z, t.transform.rotation.w)
        txyz = np.array([t.transform.translation.x,
                         t.transform.translation.y,
                         t.transform.translation.z], dtype=np.float64)
        P_out = (R @ P.T).T + txyz

        # Write and exit
        if self.binary:
            write_pcd_xyz_binary(self.outfile, P_out)
        else:
            write_pcd_xyz_ascii(self.outfile, P_out)

        self.get_logger().info(
            f"Wrote {P_out.shape[0]} pts to '{self.outfile}' in frame '{target}' "
            f"({ 'binary' if self.binary else 'ascii' })"
        )

        # Clean shutdown: stop timers/subs, set future, then shutdown ROS
        self.destroy_subscription(self.sub)
        self.timer.cancel()
        # self.done.set_result(True)
        # # Let the executor unwind gracefully:
        # rclpy.get_global_executor().shutdown() if rclpy.get_global_executor() else None
        #rclpy.shutdown()
        self.done = True

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-t","--topic", required=True, help="PointCloud2 topic (e.g. /camera/depth/points)")
    ap.add_argument("-o","--outfile", required=True, help="Output .pcd")
    ap.add_argument("-F","--frame", default="base_footprint", help="Target frame (default: base_footprint)")
    ap.add_argument("--fallback-frame", default="", help="Fallback target if --frame not in TF (e.g. base_link)")
    ap.add_argument("--timeout", type=float, default=2.0, help="TF lookup timeout (s)")
    ap.add_argument("--latest", action="store_true", help="Use latest TF (time=0) instead of cloud timestamp")
    ap.add_argument("--binary", action="store_true", help="Write binary PCD")
    args = ap.parse_args()

    rclpy.init()
    node = OneShotPCDSaver(
        topic=args.topic,
        target_frame=args.frame,
        outfile=args.outfile,
        timeout_s=args.timeout,
        use_latest=args.latest,
        binary=args.binary,
        fallback_target=args.fallback_frame or None,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
