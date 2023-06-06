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


def adjustURposeForSet(takePicResult):
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

    angle = math.radians(takePicResult["Angle"])
    changeOfX = takePicResult["TranslationX"] / 1000
    print("changeOfX", changeOfX)
    changeOfY = takePicResult["TranslationY"] / 1000
    print("changeOfY", changeOfY)

    rotate_and_mat = \
        np.array(
            [[math.cos(angle), -math.sin(angle), 0, changeOfX], [math.sin(angle), math.cos(angle), 0, changeOfY],
             [0, 0, 1, 0],
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


if __name__ == '__main__':
    c = DashboardClient("192.168.1.20")
    c.connect()
    if c.isConnected():
        print("OK")
    else:
        sys.exit()

    takePicResult = None
    if loadURprogrameAndPlay("/programs/pf/take_pic_pos.urp"):
        time.sleep(1)
        takePicResult = take_pic()
        print(takePicResult)

    if loadURprogrameAndPlay("/programs/pf/set_1_to_2.urp"):
        print("set_1_to_2 OK")

    adjustURposeForSet(takePicResult)
