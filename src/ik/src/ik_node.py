import os, sys
this_file = os.path.abspath(__file__)
project_root = os.path.abspath(os.path.join(os.path.dirname(this_file), '../..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, 'proto/generate'))

import logging
import argparse
import threading

from multiprocessing_logging import install_mp_handler
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)s] %(filename)s:%(lineno)d - %(message)s' # 这里设置格式
)
install_mp_handler()

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from teleop.tele_pose_pb2 import TeleState
from teleop.src.common import TRACK_STATE_TOPIC
from ik.src.ik_processor_base import IKProcessor


class IkNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__('ik_node')
        self._subscription = self.create_subscription(
            UInt8MultiArray,
            TRACK_STATE_TOPIC,
            self._track_state_callback,
            10)
        self._subscription  # prevent unused variable warning
        self._state_lock = threading.Lock()
        self._state = TeleState()
        self._state_flush_cnt = 0

        self._ik_timer = self.create_timer(1.0 / args.frequency, self._ik_process_callback)
        from ik.src.unitree.g1_29_ik_processor import G129IkProcessor
        self._ik_processor: IKProcessor = G129IkProcessor(self)

    def _ik_process_callback(self):
        with self._state_lock:
            current_state = TeleState()
            current_state.CopyFrom(self._state)

        left_ee_pose = current_state.left_ee_pose
        right_ee_pose = current_state.right_ee_pose
        self._ik_processor.process(left_ee_pose, right_ee_pose)

    def _track_state_callback(self, msg: UInt8MultiArray):
        self.get_logger().info('I heard: "%s"' % msg.data)
        # 1. 创建一个空的 Protobuf 消息对象
        state = TeleState()
        try:
            # 2. 关键步骤：将 ROS 消息的 data 字段转换为 bytes
            # msg.data 在 rclpy 中通常是一个 array.array 或 list
            # Protobuf 的 ParseFromString 必须接收 bytes 对象
            binary_data = bytes(msg.data)

            # 3. 反序列化
            state.ParseFromString(binary_data)

            # from google.protobuf import json_format
            # json_string = json_format.MessageToJson(state,
            #     # always_print_fields_with_no_presence=True, # 强制包含默认值字段
            #     preserving_proto_field_name=True    # 建议同时开启，保持字段名和 .proto 一致
            # )
            # print(json_string)
            with self._state_lock:
                self._state.CopyFrom(state)
                self._state_flush_cnt += 1
        except Exception as e:
            self.get_logger().error(f'解析 Protobuf 失败: {e}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frequency", type=float, default=30.0, help="Publishing frequency in Hz")

    args, other_args = parser.parse_known_args()

    logging.getLogger().setLevel(args.log_level.upper())
    try:
        rclpy.init(args=other_args)
        node = IkNode(args)
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down ROS2 node...")
        node.destroy_node()

if __name__ == '__main__':
    main()