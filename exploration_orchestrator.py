#!/usr/bin/env python3
# 2025 Stuart Johnson
#
# Run a frontier exploration to completion. Record desired data
# and package appropriately.

import os
import sys
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from ament_index_python.packages import get_package_share_directory
import yaml
import argparse
import shutil
import glob
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from snapshot_bag import SnapshotBurst

def wait_for_exploration_done(timeout=1000.0):
    print(f"Waiting for exploration done for {timeout} seconds...")
    node = rclpy.create_node('explore_waiter')
    exp_done = False

    def cb(msg):
        nonlocal exp_done
        exp_done = True

    sub = node.create_subscription(Bool, '/exploration_done', cb, 10)
    start_time = time.time()
    while not exp_done and (time.time() - start_time < timeout):
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    return exp_done

def startup_rviz_record(movie_file: str, use_sim_time: bool):
    print(f"Starting rviz recording to file: {movie_file}")
    msg_dict = {"filename": movie_file, "fps": 10.0, "scale": 1.0, "codec": 'h264', "use_sim_time": use_sim_time}
    subprocess.run(
        ["ros2",
        "service",
        "call",
        "/rviz_record/start",
        "rviz_record/srv/StartRecording",
        str(msg_dict)
        ]
    )

def stop_rviz_record():
    print("Stopping rviz recording...")
    msg_dict = {}
    subprocess.run(
        ["ros2",
        "service",
        "call",
        "/rviz_record/stop",
        "rviz_record/srv/StopRecording",
        str(msg_dict)
        ]
    )

def has_any_subscriber(topic: str) -> bool:
    node = rclpy.create_node('check_subscribers_graph')
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        num_sub = 0
        for _ in range(10):
            num_sub = node.count_subscribers(topic)
            if num_sub > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.1)  # lets graph updates process
        ok =  num_sub > 0
        return ok
    finally:
        node.destroy_node()


def startup_frontier_explorer():
    node = rclpy.create_node("bool_pub_once")
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE

    pub = node.create_publisher(Bool, "/explore", qos)
    pub.publish(Bool(data=True))

    # give DDS time to deliver (esp. without transient local)
    end = time.time() + 5.0
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()

def do_snapshot(bag_dir: str):
    print("Doing bag snapshot...")
    node = SnapshotBurst(
        one_shots=['/map', '/odom', '/octomap_occupied_space', '/octomap_grid', '/octomap_occupied_full'],
        bursts=['/tf', '/tf_static'],
        burst_duration=3.0,
        transients=set(["/map"]),
        timeout=5.0,
        out_uri=bag_dir,
        storage_id="mcap"
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Exploration orchestrator')
    parser.add_argument('--config', type=str, required=True, help='Path to config file')
    args = parser.parse_args()

    run_time = time.strftime("%Y%m%d_%H%M%S")
    time_dir = "explore_" + run_time
    cfg_file = args.config
    cfg_file_leaf = os.path.basename(cfg_file)
    with open(cfg_file, 'r') as file:
        cfg = yaml.safe_load(file)

    rclpy.init()
    if not cfg["dry_run"]:
        explorer_exists = has_any_subscriber("/explore")
        if not explorer_exists:
            print("frontier_explorer is not listening on /explore. Exiting...")
            rclpy.shutdown()
            sys.exit(0)

    home_dir = cfg["output_home_dir"]
    test_home_dir = os.path.join(home_dir, time_dir)
    os.makedirs(home_dir, exist_ok=True)
    os.makedirs(test_home_dir, exist_ok=False)
    cfg_file_out = os.path.join(test_home_dir, cfg_file_leaf)
    with open(cfg_file_out, 'w') as file:
        yaml.dump(cfg, file, sort_keys=False)
    movie_file = os.path.join(test_home_dir, "rviz.mp4")
    startup_rviz_record(movie_file, cfg["use_sim_time"])
    time.sleep(1)

    startup_frontier_explorer()
    time.sleep(1)

    wait_for_exploration_done(cfg["max_exploration_time"])

    time.sleep(1)

    stop_rviz_record()

    bag_dir = os.path.join(test_home_dir, "bag")
    do_snapshot(bag_dir)

    rclpy.shutdown()

if __name__ == '__main__':
    main()