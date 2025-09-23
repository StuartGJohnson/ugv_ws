#!/usr/bin/env python3

# example usage:
# python3 snapshot_burst.py -o /tmp/snap_mcap \
#   -t /map -t /fusion/points -t /scan -t /camera/image_color\
#   -b /tf -b /odom -b /tf_static \
#   --burst-duration 3.0 --storage mcap

import argparse, time
from typing import Dict, Set
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

class SnapshotBurst(Node):
    def __init__(self, one_shots, bursts, burst_duration, transients, timeout, out_uri, storage_id):
        super().__init__('snapshot_burst')
        self.done = False
        self.one_shots = set(one_shots)
        self.bursts = set(bursts)
        self.transients = set(transients)
        self.burst_duration = float(burst_duration)
        self.timeout_ns = int(timeout * 1e9)
        self.start_ns = self.get_clock().now().nanoseconds

        self.msg_types: Dict[str,str] = {}
        self.got_one: Set[str] = set()
        self.have_any_from_burst: Set[str] = set()
        self.subs = []

        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=out_uri, storage_id=storage_id),
            rosbag2_py.ConverterOptions('', '')
        )

        # register topics + subscribe
        self._setup_topics(self.one_shots | self.bursts)

        # tick @ 50 Hz to decide when to quit
        self.create_timer(0.02, self._tick)

    def _qos_for(self, topic):
        qos = QoSProfile(
            depth=50, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        if topic in self.transients:
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            qos.depth = 1
        else:
            # try inherit from publisher if present
            infos = self.get_publishers_info_by_topic(topic)
            if infos:
                pq = infos[0].qos_profile
                qos.reliability = pq.reliability
                qos.durability = pq.durability
        return qos

    def _wait_type(self, topic, wait_s=1.0):
        t0 = time.time()
        while time.time() - t0 < wait_s:
            for name, types in self.get_topic_names_and_types():
                if name == topic and types:
                    return types[0]
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def _setup_topics(self, topics):
        for t in sorted(topics):
            tname = self._wait_type(t, 1.0) or self._wait_type(t, 2.0)
            if not tname:
                self.get_logger().warn(f"No publishers seen yet on {t}; skipping.")
                continue
            self.msg_types[t] = tname
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=t, type=tname, serialization_format='cdr', offered_qos_profiles=''
            ))
            msg_cls = get_message(tname)
            qos = self._qos_for(t)

            if t in self.one_shots:
                self.subs.append(self.create_subscription(
                    msg_cls, t, self._cb_one_shot(t), qos))
            else:
                self.subs.append(self.create_subscription(
                    msg_cls, t, self._cb_burst(t), qos))

    def _cb_one_shot(self, topic):
        def cb(msg):
            if topic in self.got_one:
                return
            self.got_one.add(topic)
            self.writer.write(topic, serialize_message(msg), self.get_clock().now().nanoseconds)
            self.get_logger().info(f"ONE-SHOT captured: {topic}")
        return cb

    def _cb_burst(self, topic):
        def cb(msg):
            self.have_any_from_burst.add(topic)
            self.writer.write(topic, serialize_message(msg), self.get_clock().now().nanoseconds)
        return cb

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        # all one-shots obtained?
        missing_ones = [t for t in self.one_shots if t in self.msg_types and t not in self.got_one]
        ones_done = len(missing_ones) == 0

        # burst window elapsed?
        burst_elapsed = (now - self.start_ns) >= int(self.burst_duration * 1e9)

        # global timeout?
        timed_out = (now - self.start_ns) >= self.timeout_ns if self.timeout_ns > 0 else False

        if ones_done and burst_elapsed:
            self.get_logger().info("SnapshotBurst done.")
            self.done = True
        elif timed_out:
            self.get_logger().warn(f"Timeout. Missing one-shots: {missing_ones}")
            self.done = True

def main():
    ap = argparse.ArgumentParser(description="Snapshot one-shot topics and burst-record others, then exit.")
    ap.add_argument('-o','--output', required=True, help='Bag directory (created)')
    ap.add_argument('--storage', default='mcap', help='rosbag2 storage id (mcap/sqlite3)')
    ap.add_argument('--timeout', type=float, default=8.0, help='Overall timeout seconds')
    ap.add_argument('--burst-duration', type=float, default=3.0, help='Seconds to record burst topics')
    ap.add_argument('-t','--topic', dest='ones', action='append', default=[],
                    help='One-shot topic (repeat)')
    ap.add_argument('-b','--burst', dest='bursts', action='append', default=[],
                    help='Burst topic (repeat)')
    ap.add_argument('--transient', nargs='*', default=['/tf_static','/map'],
                    help='Topics to subscribe with TRANSIENT_LOCAL (default: /tf_static /map)')
    args = ap.parse_args()

    if not args.ones and not args.bursts:
        print("Specify at least one -t or -b topic.")
        return

    rclpy.init()
    node = SnapshotBurst(
        one_shots=args.ones,
        bursts=args.bursts,
        burst_duration=args.burst_duration,
        transients=set(args.transient),
        timeout=args.timeout,
        out_uri=args.output,
        storage_id=args.storage
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
