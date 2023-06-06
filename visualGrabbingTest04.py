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

    tcpInBaseRotateMat = cv2.Rodrigues((tcpPose[3], tcpPose[4], tcpPose[5]))[0]
    print("tcpInBaseRotateMat:")
    print(tcpInBaseRotateMat)

    tcpInBaseMat = np.eye(4)
    tcpInBaseMat[:3, :3] = tcpInBaseRotateMat
    tcpInBaseMat[0][3] = tcpPose[0]
    tcpInBaseMat[1][3] = tcpPose[1]
    tcpInBaseMat[2][3] = tcpPose[2]

    cameraInTcpMat = np.array([[1, 0, 0, 0], [0, 1, 0, -0.12728], [0, 0, 1, 0], [0, 0, 0, 1]])

    print("takePicResult:", takePicResult)
    transX = takePicResult["TranslationX"] / 1000
    transY = takePicResult["TranslationY"] / 1000
    Angle = math.radians(takePicResult["Angle"])

    markInCameraMat = np.array(
        [[math.cos(Angle), -math.sin(Angle), 0, transX], [math.sin(Angle), math.cos(Angle), 0, transY],
         [0, 0, 1, 0.03],
         [0, 0, 0, 1]])

    rackInMarkMat = np.array([[1, 0, 0, -0.09], [0, 1, 0, -0.13325], [0, 0, 1, 0.03], [0, 0, 0, 1]])

    rackInCamerMat = np.dot(markInCameraMat, rackInMarkMat)
    rackInTcpMat = np.dot(cameraInTcpMat, rackInCamerMat)
    rackInBase = np.dot(tcpInBaseMat, rackInTcpMat)

    rackInBaseRoteVector = cv2.Rodrigues(rackInBase[:3, :3])[0]

    ur_finnal_pose = [rackInBase[0][3], rackInBase[1][3], rackInBase[2][3], rackInBaseRoteVector[0][0],
                      rackInBaseRoteVector[1][0],
                      rackInBaseRoteVector[2][0]]
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
        adjustURposeForSet(takePicResult)

    if loadURprogrameAndPlay("/programs/pf/set_1_to_1.urp"):
        print("set_1_to_1 OK")

    robotControlClient = rtde_control.RTDEControlInterface("192.168.1.20")
    robotControlClient.moveL(pose)
