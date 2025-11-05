import rclpy
import argparse
from rclpy.node import Node

import re
import os

def detect_bot_id(node: Node = None, timeout: int = 10):
    if node is None:
        node = Node("detect_bot_id")

    for _ in range(timeout):
        bot_id = parse_bot_id(node)
        if bot_id:
            return bot_id

        rclpy.spin_once(node, timeout_sec=1.0)    

def parse_bot_id(node: Node):
    topic_names_and_types = node.get_topic_names_and_types()
    for topic_name, _ in topic_names_and_types:
        match = re.match('/([^/]*)/ro1/hardware/joint_state', topic_name)
        if match:
            return match.group(1)
        
    return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Get bot ID from a robot')
    args = parser.parse_args()

    rclpy.init()

    try:
        # Spin a few times to allow topic discovery
        print('Waiting for topic discovery...')
        bot_id = detect_bot_id()

        if bot_id:
            print(f"Bot ID: {bot_id}")
        else:
            print("No bot ID found. Make sure the robot is publishing joint states.")

    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
