from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from ik.src.ik_processor_base import IKProcessor
from foxglove.Pose_pb2 import Pose

UNITREE_IK_SOL_TOPIC = "/unitree/ik_sol"
UNITREE_LOW_STATE_TOPIC = "/unitree/low_state"

class G129IkProcessor(IKProcessor):
    def __init__(self, node: Node):
        super().__init__()
        self._node = node
        self._subscription = node.create_subscription(
            UInt8MultiArray,
            UNITREE_LOW_STATE_TOPIC,
            self._low_state_callback,
            10)

    def _low_state_callback(self, msg: UInt8MultiArray):
        # Process the low state message
        pass

    def process(self, left_ee_pose: Pose, right_ee_pose: Pose):
        pass