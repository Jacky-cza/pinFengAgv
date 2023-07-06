# import sys
# import time
# from dashboard_client import DashboardClient
# import rtde_receive
# import rtde_control
# import cv2
# import numpy as np
# import math
#
#
# # 检查一个旋转矩阵是否有效
# def isRotationMatrix(R):
#     # 得到该矩阵的转置
#     Rt = np.transpose(R)
#     # 旋转矩阵的一个性质是，相乘后为单位阵
#     shouldBeIdentity = np.dot(Rt, R)
#     # 构建一个三维单位阵
#     I = np.identity(3, dtype=R.dtype)
#     # 将单位阵和旋转矩阵相乘后的值做差
#     n = np.linalg.norm(I - shouldBeIdentity)
#     # 如果小于一个极小值，则表示该矩阵为旋转矩阵
#     return n < 1e-6
#
#
# # 这部分的代码输出与Matlab里边的rotm2euler一致
# def rotationMatrixToEulerAngles(R):
#     # 断言判断是否为有效的旋转矩阵
#     assert (isRotationMatrix(R))
#
#     sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
#
#     singular = sy < 1e-6
#
#     if not singular:
#         x = math.atan2(R[2, 1], R[2, 2])
#         y = math.atan2(-R[2, 0], sy)
#         z = math.atan2(R[1, 0], R[0, 0])
#     else:
#         x = math.atan2(-R[1, 2], R[1, 1])
#         y = math.atan2(-R[2, 0], sy)
#         z = 0
#
#     # return np.array([z, y, x])
#     return np.array([x, y, z])
#
#
# def eulerAnglesToRotationMatrix(theta):
#     # 分别构建三个轴对应的旋转矩阵
#     R_x = np.array([[1, 0, 0],
#                     [0, math.cos(theta[0]), -math.sin(theta[0])],
#                     [0, math.sin(theta[0]), math.cos(theta[0])]
#                     ])
#
#     R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
#                     [0, 1, 0],
#                     [-math.sin(theta[1]), 0, math.cos(theta[1])]
#                     ])
#
#     R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
#                     [math.sin(theta[2]), math.cos(theta[2]), 0],
#                     [0, 0, 1]
#                     ])
#
#     # 将三个矩阵相乘，得到最终的旋转矩阵
#     R = np.dot(R_z, np.dot(R_y, R_x))
#
#     return R
#
#
# robotControlClient = rtde_control.RTDEControlInterface("192.168.1.20")
# print(robotControlClient.getTCPOffset())
# robotControlClient.movePath()
# #
# # robotReceiveClient = rtde_receive.RTDEReceiveInterface("192.168.1.20")
# # robotPose = robotReceiveClient.getActualTCPPose()
# # print(robotPose)
# #
# # tcp_in_base_rotate_mat = cv2.Rodrigues((robotPose[3], robotPose[4], robotPose[5]))[0]
# # print("tcp_in_base_rotate_mat:", tcp_in_base_rotate_mat)
# #
# # euler = rotationMatrixToEulerAngles(tcp_in_base_rotate_mat)
# # print("euler:", euler)
# # euler[2] = euler[2] - 0.1047
# # print("euler:", euler)
# #
# # """
# # [-3.13099582 -0.00877927  3.11962377]
# # TCP方向目前的坐标系：
# # 绕Z旋转了：+180
# # 绕X转了：-180/+180
# #
# # """
# #
# # rotateMaxi = eulerAnglesToRotationMatrix(euler)
# #
# # print("rotateMaxi:", rotateMaxi)
# #
# # """
# # 绕X转90
# # 绕Y转180
# # """
# #
# # rotate_vector = cv2.Rodrigues(rotateMaxi)[0]
# # print("rotate_vector:", rotate_vector)
# #
# # print(rotate_vector[0][0])
# # print(rotate_vector[1][0])
# # print(rotate_vector[2][0])
# #
# # new_pos_of_ur = [robotPose[0]+0.005, robotPose[1]+0.005, robotPose[2], rotate_vector[0][0],
# #                  rotate_vector[1][0], rotate_vector[2][0]]
# #
# # print(new_pos_of_ur)
# #
# # robotControlClient.moveJ_IK(new_pos_of_ur)

from dashboard_client import DashboardClient

c = DashboardClient("192.168.123.238")
c.connect()
print(c.isConnected())

print(c.safetymode())

"""
Safetymode: ROBOT_EMERGENCY_STOP
Safetymode: NORMAL

"""

print(c.robotmode())
"""
Robotmode: POWER_OFF 没上电之前
Robotmode: IDLE     解抱闸之前
Robotmode: RUNNING  解抱闸之后
"""

