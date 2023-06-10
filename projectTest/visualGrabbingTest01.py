import sys
import time
from dashboard_client import DashboardClient
import rtde_receive
import rtde_control
import cv2
import numpy as np
import math
import time
import socket
import json

c = DashboardClient("192.168.1.20")
c.connect()
if c.isConnected():
    print("OK")
else:
    sys.exit()


def loadURprogrameAndPlay(urPrograme):
    c.loadURP(urPrograme)
    while True:
        if c.getLoadedProgram() == "Loaded program: {}".format(urPrograme):
            print("load {} OK ".format(urPrograme))
            break

    c.play()
    time.sleep(1)
    while True:
        if c.programState().split(" ")[0] == "STOPPED":
            print("play {} OK".format(urPrograme))
            break
    return True


def take_pic():
    s = None
    try:
        ip = "192.168.1.66"
        port = 3000
        s = socket.socket()
        s.connect((ip, port))
        s.send('\002trigger\003'.encode())
        str = ""
        times = 0
        while True:
            data = s.recv(1024)
            str += data.decode()
            times += 1
            if times == 2:
                break

        res = str.split("\x03")[1]
        finnal_res = json.loads(res)
        return {"Pass": finnal_res["Pass"], "MatchScore:": finnal_res["MatchScore"],
                "TranslationX": finnal_res["TranslationX"], "TranslationY": finnal_res["TranslationY"],
                "Angle": finnal_res["Angle"]}
    except Exception as e:
        print(e)
        return False
    finally:
        s.close()


# 检查一个旋转矩阵是否有效
def isRotationMatrix(R):
    # 得到该矩阵的转置
    Rt = np.transpose(R)
    # 旋转矩阵的一个性质是，相乘后为单位阵
    shouldBeIdentity = np.dot(Rt, R)
    # 构建一个三维单位阵
    I = np.identity(3, dtype=R.dtype)
    # 将单位阵和旋转矩阵相乘后的值做差
    n = np.linalg.norm(I - shouldBeIdentity)
    # 如果小于一个极小值，则表示该矩阵为旋转矩阵
    return n < 1e-6


# 这部分的代码输出与Matlab里边的rotm2euler一致
def rotationMatrixToEulerAngles(R):
    # 断言判断是否为有效的旋转矩阵
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def eulerAnglesToRotationMatrix(theta):
    # 分别构建三个轴对应的旋转矩阵
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    # 将三个矩阵相乘，得到最终的旋转矩阵
    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


def adjustURposeForSet(takePicResult):
    rtdeReceiveClient = rtde_receive.RTDEReceiveInterface("192.168.1.20")
    URcurrentPose = rtdeReceiveClient.getActualTCPPose()
    print("URcurrentPose:", URcurrentPose)
    # rtdeReceiveClient.disconnect()
    tcp_in_base_rotate_mat = cv2.Rodrigues((URcurrentPose[3], URcurrentPose[4], URcurrentPose[5]))[0]
    print("tcp_in_base_rotate_mat:", tcp_in_base_rotate_mat)

    angles = rotationMatrixToEulerAngles(tcp_in_base_rotate_mat)
    print("angles[2]:", angles[2])
    # rz = takePicResult["Angle"] * math.pi / 180
    rz = math.radians(takePicResult["Angle"])
    print("rz:", rz)
    angles[2] = angles[2] - rz
    print("finnalRz:", angles[2])
    print("finnalAngle:", angles)

    rotateMatrix = eulerAnglesToRotationMatrix(angles)
    new_rotate_vector = cv2.Rodrigues(rotateMatrix)[0]

    xForChange = (takePicResult["TranslationX"] - 130 * math.sin(rz)) / 1000
    print("-------------------------------------------")
    print(takePicResult["TranslationX"])
    print(130 * math.sin(takePicResult["Angle"]))
    print("-------------------------------------------")
    yForChange = ((130 * math.cos(rz) - 130) + takePicResult["TranslationY"]) / 1000
    print((130 * math.cos(takePicResult["Angle"]) - 130))
    print(takePicResult["TranslationY"])
    print("-------------------------------------------")
    print("xForChange:", xForChange)
    print("yForChange:", yForChange)
    print("-------------------------------------------")

    URcurrentPose[0] = URcurrentPose[0] - xForChange
    URcurrentPose[1] = URcurrentPose[1] + yForChange

    newPoseOfUR = [URcurrentPose[0], URcurrentPose[1], URcurrentPose[2], new_rotate_vector[0][0],
                   new_rotate_vector[1][0],
                   new_rotate_vector[2][0]]

    print("newPoseOfUR:", newPoseOfUR)
    rtdeControlClient = rtde_control.RTDEControlInterface("192.168.1.20")
    rtdeControlClient.moveJ_IK(newPoseOfUR, 0.1)

    # rtdeReceiveClient = rtde_receive.RTDEReceiveInterface("192.168.1.20")
    # URcurrentPose = rtdeReceiveClient.getActualTCPPose()
    # URcurrentPose[2] = URcurrentPose[2] - 0.088
    # rtdeControlClient.moveJ_IK(newPoseOfUR)
    # rtdeReceiveClient.disconnect()


if __name__ == '__main__':
    takePicResult = None
    if loadURprogrameAndPlay("/programs/pf/take_pic_pos.urp"):
        time.sleep(1)
        takePicResult = take_pic()
        print(takePicResult)

    # if loadURprogrameAndPlay("/programs/pf/set_1_to_2.urp"):
    #     print("set_1_to_2 OK")

    # adjustURposeForSet(takePicResult)
