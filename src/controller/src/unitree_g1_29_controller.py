from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import ( LowCmd_  as hg_LowCmd, LowState_ as hg_LowState) # idl for g1, h1_2
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController

import os
import sys
import time
import logging
import threading
from functools import partial
from multiprocessing_logging import install_mp_handler
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


from base.controller_interface import ControllerInterface
from base.utils import build_arg_parser

this_file = os.path.abspath(__file__)
project_root = os.path.abspath(os.path.join(os.path.dirname(this_file), '../..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, '../proto/generate'))
from controller.state_pb2 import UnitTreeLowState
from ik.ik_sol_pb2 import UnitTreeIkSol

# from loguru import logger
# logger.debug(f"asdfasd {self.subscriber_list_}")
# logger.info(f"asdfasd {self.subscriber_list_}")
# logger.warning(f"asdfasd {self.subscriber_list_}")
# logger.error(f"asdfasd {self.subscriber_list_}")

# sub_topic
UNITREE_IK_SOL_TOPIC    = "/unitree/ik_sol"
TRACK_STATE_TOPIC       = '/teleop/track_state'
# pub topic
UNITREE_LOW_STATE_TOPIC = "/unitree/low_state"

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)s] %(filename)s:%(lineno)d - %(message)s' # 这里设置格式
)
install_mp_handler()


class UnitreeG129Controller(ControllerInterface):
    def __init__(self, config):
        super().__init__(config)
        self.motion_mode     = False
        self.simulation_mode = False
        self.msg_count       = 0
        self.publisher_control = {}
        self.fps               = 15
        self._pub_period = 1.0 / self.fps
        self._last_pub_ros_low_state_time     = 0.0
        self._last_pub_unitree_low_state_time = 0.0
        self.track_state = UnitTreeIkSol()
        self.low_state   = UnitTreeLowState()
        self._state_lock = threading.Lock()

    def Start(self):
        self._init_unitree()
        self._init_ros()
        self._running = True
        logging.info(f"Start Finish")


    def Stop(self):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self._running = False


    def SendMsg(self, ros_msg):
        # get unitree msg and ros send msg
        # self.arm_ctrl.ctrl_dual_arm(self.track_state.dual_arm_sol_q, self.track_state.dual_arm_sol_tauff)
        
        msg = UInt8MultiArray()
        current_lr_arm_q  = self.arm_ctrl.get_current_dual_arm_q()
        current_lr_arm_dq = self.arm_ctrl.get_current_dual_arm_dq()

        try:
            self.low_state.dual_arm_q.extend(current_lr_arm_q)
            self.low_state.dual_arm_dq.extend(current_lr_arm_dq)
            binary_data = self.low_state.SerializeToString()
            msg.data = list(binary_data)
            self.publisher_control[UNITREE_LOW_STATE_TOPIC].publish(msg)
        except Exception as e:
            logging.error(f'SerializeToString Protobuf failed: {e}')
        self.low_state.Clear()

    def _init_unitree(self):
        ChannelFactoryInitialize(1)
        # if self.simulation_mode:
            # ChannelFactoryInitialize(2)
            # logging.info("domaain 2")
        # else:
            # ChannelFactoryInitialize(0)
        # 接口	                             类型	    作用
        # __init__	                        构造函数	初始化 DDS、线程、PID 参数、控制消息
        # ctrl_dual_arm(q, tau)	            外部控制	设置双臂目标关节角度/力矩
        # ctrl_dual_arm_go_home()	        外部控制	将双臂归零位
        # speed_gradual_max(t)	            外部控制	速度渐进到最大值
        # speed_instant_max()	            外部控制	速度立即最大
        # get_current_motor_q()	            状态查询	获取全身电机角度
        # get_current_dual_arm_q()	        状态查询	获取双臂角度
        # get_current_dual_arm_dq()	        状态查询	获取双臂速度
        # get_mode_machine()	            状态查询	获取当前 DDS 模式
        # _subscribe_motor_state()	        内部线程	持续读取 DDS 电机状态，更新缓存
        # _ctrl_motor_state()	            内部线程	循环发送控制命令到 DDS
        # clip_arm_q_target(target, vel)	内部工具	对目标角度做速度限制
        # _Is_weak_motor(id)	            内部工具	判断弱电机
        # _Is_wrist_motor(id)	            内部工具	判断手腕电机
        self.arm_ctrl = G1_29_ArmController(motion_mode=self.motion_mode, simulation_mode=self.simulation_mode)
        self.arm_ctrl.speed_gradual_max()

    def _init_ros(self):
        rclpy.init()
        self.ros_node = Node("unitree_g1_29_controller")
        for topic in self.subscriber_list_:
            self.ros_node.create_subscription(UInt8MultiArray, topic, partial(self._message_callback, topic), 10)
            logging.info(f"create {topic} ros message sub")
        for topic in self.publisher_list_:
            self.publisher_control[topic] = self.ros_node.create_publisher(UInt8MultiArray, topic, 10)
            logging.info(f"create {topic} ros message pub")
        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()

    def _message_callback(self, topic_name, msg):
        self.msg_count += 1
        if self.msg_count % 100 == 0:
            logging.info(f"Received from {topic_name}: {msg.data}")

        if topic_name is UNITREE_IK_SOL_TOPIC:
            state = UnitTreeIkSol()
            try:
                binary_data = bytes(msg.data)
                state.ParseFromString(binary_data)
                with self._state_lock:
                    self.track_state.CopyFrom(state)
                logging.info(f"Get {UNITREE_IK_SOL_TOPIC} msg")
            except Exception as e:
                logging.error(f'ParseFromString Protobuf failed: {e}')

    def _ros_spin(self):
        rclpy.spin(self.ros_node)


def extra_controller_args(parser):
    parser.description = "Unitree G1 specific controller"
    parser.add_argument( "--control-mode", type=str, default="low_cmd", choices=["low_cmd", "high_cmd"], help="Unitree control mode")
    parser.set_defaults(rate_hz=500)
    parser.set_defaults(pub_topic_list=[UNITREE_LOW_STATE_TOPIC])
    parser.set_defaults(sub_topic_list=[UNITREE_IK_SOL_TOPIC, TRACK_STATE_TOPIC])


def main():
    parser = build_arg_parser(extra_controller_args)
    args = parser.parse_args()

    controller = UnitreeG129Controller(vars(args))
    controller.Start()

    while(True):
        msg = "test"
        controller.SendMsg(msg)
        time.sleep(1)
        logging.info(f"Send Msg")

if __name__ == "__main__":
    main()


