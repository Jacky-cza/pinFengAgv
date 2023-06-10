import sys
import time
from dashboard_client import DashboardClient
import rtde_receive
import rtde_control
import cv2
import numpy as np
import math

robotReceiveClient = rtde_receive.RTDEReceiveInterface("192.168.1.20")
URpose = robotReceiveClient.getActualTCPPose()
print("URpose:")
print(URpose)

tcp_in_base_rotate_mat = cv2.Rodrigues((URpose[3], URpose[4], URpose[5]))[0]
print("tcp_in_base_rotate_mat:")
print(tcp_in_base_rotate_mat)

tcp_in_base_mat = np.eye(4)
tcp_in_base_mat[:3, :3] = tcp_in_base_rotate_mat
tcp_in_base_mat[0][3] = URpose[0]
tcp_in_base_mat[1][3] = URpose[1]
tcp_in_base_mat[2][3] = URpose[2]

print("tcp_in_base_mat:")
print(tcp_in_base_mat)

# 用10°测试
angle = math.radians(10)

# rotate_and_mat = np.array([[0, 0, -1, 0.33], [0, -1, 0, 0], [-1, 0, 0, 0.12], [0, 0, 0, 1]])


# rotate_and_mat = \
#     np.array(
#         [[math.cos(angle), -math.sin(angle), 0, 0], [math.sin(angle), math.cos(angle), 0, 0], [0, 0, 1, 0],
#          [0, 0, 0, 1]])


rotate_and_mat = \
    np.array(
        [[math.cos(angle), -math.sin(angle), 0, 0], [math.sin(angle), math.cos(angle), 0, 0], [0, 0, 1, 0],
         [0, 0, 0, 1]])

finnal_mat = np.dot(tcp_in_base_mat, rotate_and_mat)
print("finnal_mat:")
print(finnal_mat)

ur_pose_vector = cv2.Rodrigues(finnal_mat[:3, :3])[0]
ur_finnal_pose = [finnal_mat[0][3], finnal_mat[1][3], finnal_mat[2][3], ur_pose_vector[0][0],
                  ur_pose_vector[1][0],
                  ur_pose_vector[2][0]]

robotControlClient = rtde_control.RTDEControlInterface("192.168.1.20")
robotControlClient.moveJ_IK(ur_finnal_pose)
