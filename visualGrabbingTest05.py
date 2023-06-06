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


def adjustURposeForSet(takePicResult):
    rtdeReceiveClient = rtde_receive.RTDEReceiveInterface("192.168.1.20")
    tcpPose = rtdeReceiveClient.getActualTCPPose()
    rtdeReceiveClient.disconnect()

    tcpInBaseRotateMat = cv2.Rodrigues((tcpPose[3], tcpPose[4], tcpPose[5]))[0]
    print("tcpInBaseRotateMat:")
    print(tcpInBaseRotateMat)

    tcpInBaseMat = np.eye(4)
    tcpInBaseMat[:3, :3] = tcpInBaseRotateMat
    tcpInBaseMat[0][3] = tcpPose[0]
    tcpInBaseMat[1][3] = tcpPose[1]
    tcpInBaseMat[2][3] = tcpPose[2]
    print("tcpInBaseMat:")
    print(tcpInBaseMat)

    cameraInTcpMat = np.array([[1, 0, 0, 0], [0, 1, 0, -0.1273], [0, 0, 1, 0], [0, 0, 0, 1]])

    print("takePicResult:", takePicResult)
    # 单位是 m
    transX = takePicResult["TranslationX"] / 1000
    transY = takePicResult["TranslationY"] / 1000
    # 单位是弧度
    angle = math.radians(takePicResult["Angle"] - 0.5)

    # 相机坐标系？ mark坐标系？
    markInCameraMat = np.array(
        [[math.cos(angle), -math.sin(angle), 0, transX + 0.008],
         [math.sin(angle), math.cos(angle), 0, transY - 0.004],
         [0, 0, 1, 0.09],
         [0, 0, 0, 1]])

    # mark坐标系？
    rackInMarkMat = np.array([[1, 0, 0, 0.09], [0, 1, 0, -0.13325], [0, 0, 1, 0.03], [0, 0, 0, 1]])
    rackInCamerMat = np.dot(markInCameraMat, rackInMarkMat)
    rackInTcpMat = np.dot(cameraInTcpMat, rackInCamerMat)
    rackInBaseMat = np.dot(tcpInBaseMat, rackInTcpMat)

    rackInBaseMatRotateVector = cv2.Rodrigues(rackInBaseMat[:3, :3])[0]

    ur_finnal_pose = [rackInBaseMat[0][3], rackInBaseMat[1][3], rackInBaseMat[2][3], rackInBaseMatRotateVector[0][0],
                      rackInBaseMatRotateVector[1][0],
                      rackInBaseMatRotateVector[2][0]]
    print("ur_finnal_pose:", ur_finnal_pose)
    global pose
    pose = ur_finnal_pose


if __name__ == '__main__':
    pose = None

    c = DashboardClient("192.168.1.20")
    c.connect()
    if c.isConnected():
        print("connect UR OK")
    else:
        print("connect UR error")
        sys.exit()

    takePicResult = None
    if loadURprogrameAndPlay("/programs/pf/take_pic_pos.urp"):
        time.sleep(0.5)

    takePicResult = take_pic()
    print(takePicResult)

    try:
        if takePicResult["MatchScore"] < 80:
            print("camera does not get mark")
            sys.exit()
    except Exception as e:
        print(e)
        pass

    adjustURposeForSet(takePicResult)
    if loadURprogrameAndPlay("/programs/pf/set_1_to_1.urp"):
        print("set_1_to_1 OK")

    robotControlClient = rtde_control.RTDEControlInterface("192.168.1.20")
    robotControlClient.moveJ_IK(pose)
    print("get pose already")
