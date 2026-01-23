import numpy as np
from scipy.spatial.transform import Rotation as R
from foxglove.Pose_pb2 import Pose
from foxglove.Vector3_pb2 import Vector3
from foxglove.Quaternion_pb2 import Quaternion


def valid_matrix(matrix) -> bool:
    if matrix.shape != (4, 4) or np.all(matrix[:3, :3] == 0):
        return False
    return True

def Matrix2Pose(matrix) -> Pose:
    if matrix.shape != (4, 4) or np.all(matrix[:3, :3] == 0):
        return None
    # 1. 提取平移向量 (前三行，最后一列)
    position = matrix[:3, 3]
    # 2. 提取旋转矩阵 (左上角 3x3)
    rotation_matrix = matrix[:3, :3]

    # 3. 转换为四元数
    # SciPy 默认返回的顺序是 [x, y, z, w]
    quat = R.from_matrix(rotation_matrix).as_quat()

    pose = Pose()
    pose.position.CopyFrom(Vector3(x=position[0], y=position[1], z=position[2]))
    pose.orientation.CopyFrom(Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
    return pose

def Pose2Matrix(pose: Pose) -> np.ndarray:
    matrix = np.eye(4)
    # 提取位置
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z

    # 提取四元数
    quat = [pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w]
    rotation_matrix = R.from_quat(quat).as_matrix()
    matrix[:3, :3] = rotation_matrix
    return matrix
