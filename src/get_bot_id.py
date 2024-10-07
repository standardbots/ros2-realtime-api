from rclpy.node import Node

import re

def get_bot_id(node: Node):
    topic_names_and_types = node.get_topic_names_and_types()
    for topic_name, _ in topic_names_and_types:
        match = re.match('/([^/]*)/ro1/hardware/joint_state', topic_name)
        if match:
            return match.group(1)
        
    return None
        