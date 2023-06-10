import sys
from dashboard_client import DashboardClient
from interfaces.baseCommunicate import Comm
import rtde_receive
import rtde_control
import math
import time
import socket
import json


def drive_agv(station):
    try:
        agv_drive_obj.connectNavPort()
        time.sleep(0.1)
        agv_drive_obj.navigateToStation(station)
        time.sleep(0.1)
        agv_drive_obj.closeNavPort()
        agv_drive_obj.connectStatePort()
        time.sleep(0.1)

        d = {}
        while True:
            agv_drive_obj.getNavigationState(d)
            nav_res = d["status"]
            if nav_res == 4:
                print("robot has arrived at {} station ".format(station))
                break
            else:
                print("navigating to {} station  ".format(station))
                time.sleep(0.5)
        agv_drive_obj.closeStatePort()
        return True
    except Exception as e:
        print(e)
        return False


def loadURprogrameAndPlay(urPrograme):
    c = DashboardClient(robotId)
    try:
        c.connect()
        if c.isConnected():
            print("UR connect OK")
        else:
            count = 0
            while True:
                c.connect()
                time.sleep(0.5)
                if c.isConnected():
                    break
                count += 1
                if count == 3:
                    print("UR connect error")
                    return False

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
    except Exception as e:
        print(e)
        return False
    finally:
        c.disconnect()


def take_pic():
    s = socket.socket()
    try:
        cameraIpAndPort = (cameraId, 3000)
        s.settimeout(3)
        s.connect(cameraIpAndPort)
        time.sleep(0.2)
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
        return {"Pass": finnal_res["Pass"], "MatchScore": finnal_res["MatchScore"],
                "TranslationX": finnal_res["TranslationX"], "TranslationY": finnal_res["TranslationY"],
                "Angle": finnal_res["Angle"]}
    except socket.timeout:
        print("camera socket timeout")
        return False
    except Exception as e:
        print(e)
        return False
    finally:
        s.close()


def adjustURposeForSet(order):
    if order not in ["oneTOone", "oneTOtwo", "twoTOone", "twoTOtwo"]:
        print("order is error")
        sys.exit()
    takePicResult = take_pic()
    print("takePicResult01:", takePicResult)
    if takePicResult["MatchScore"] < 80:
        print("MatchScore is too low")
        sys.exit()
    else:
        print("MatchScore is OK")

    robotReceiveClient = rtde_receive.RTDEReceiveInterface(robotId)
    URpose = robotReceiveClient.getActualTCPPose()
    print("URpose:")
    print(URpose)
    robotReceiveClient.disconnect()

    robotControlClient = rtde_control.RTDEControlInterface(robotId)
    angle = math.radians(takePicResult["Angle"])
    print("angle", angle)
    pose = robotControlClient.poseTrans(URpose, [0, 0, 0, 0, 0, angle])
    print("pose:", pose)
    # moveJ_IK 执行的时候是阻塞的，只有执行完之后才会执行下面的命令，所以使用它移动robot的时候不需要轮询判断robot有没有到位
    robotControlClient.moveJ_IK(pose)
    robotControlClient.disconnect()

    takePicResult = take_pic()
    print("takePicResult02:", takePicResult)
    if takePicResult["MatchScore"] < 80:
        print("MatchScore is too low")
        sys.exit()
    else:
        print("MatchScore is OK")

    x = takePicResult["TranslationX"] / 1000
    y = takePicResult["TranslationY"] / 1000
    robotReceiveClient = rtde_receive.RTDEReceiveInterface(robotId)
    URpose = robotReceiveClient.getActualTCPPose()
    robotReceiveClient.disconnect()

    robotControlClient = rtde_control.RTDEControlInterface(robotId)
    pose = robotControlClient.poseTrans(URpose, [x, y, 0, 0, 0, 0])
    robotControlClient.moveL(pose)
    robotControlClient.disconnect()

    takePicResult = take_pic()
    print("takePicResult03:", takePicResult)
    if takePicResult["MatchScore"] < 80:
        print("MatchScore is too low")
        sys.exit()
    else:
        print("MatchScore is OK")

    robotReceiveClient = rtde_receive.RTDEReceiveInterface(robotId)
    URpose = robotReceiveClient.getActualTCPPose()
    robotReceiveClient.disconnect()

    robotControlClient = rtde_control.RTDEControlInterface(robotId)
    URprograme01 = URprograme02 = None
    if order == "oneTOone":
        pose = robotControlClient.poseTrans(URpose, [0.081, -0.259, 0.11, 0, math.radians(1), -math.radians(1)])
        URprograme01 = "/programs/get_rack_from_1.urp"
        URprograme02 = "/programs/set_rack_to_1_and_back.urp"
    elif order == "oneTOtwo":
        pose = robotControlClient.poseTrans(URpose, [-0.0977, -0.256, 0.11, 0, math.radians(1), -math.radians(1)])
        URprograme01 = "/programs/get_rack_from_1.urp"
        URprograme02 = "/programs/set_rack_to_2_and_back.urp"
    elif order == "twoTOone":
        pose = robotControlClient.poseTrans(URpose, [0.081, -0.259, 0.11, 0, math.radians(1), -math.radians(1)])
        URprograme01 = "/programs/get_rack_from_2.urp"
        URprograme02 = "/programs/set_rack_to_1_and_back.urp"
    elif order == "twoTOtwo":
        pose = robotControlClient.poseTrans(URpose, [-0.0977, -0.256, 0.11, 0, math.radians(1), -math.radians(1)])
        URprograme01 = "/programs/get_rack_from_2.urp"
        URprograme02 = "/programs/set_rack_to_2_and_back.urp"
    robotControlClient.disconnect()

    if loadURprogrameAndPlay(URprograme01):
        print("get rack")

    robotControlClient = rtde_control.RTDEControlInterface(robotId)
    robotControlClient.moveJ_IK(pose)
    robotControlClient.disconnect()

    if loadURprogrameAndPlay(URprograme02):
        print("programe OK")


if __name__ == '__main__':
    agv_drive_obj = Comm()
    robotId = "192.168.123.238"
    cameraId = "192.168.123.237"

    # drive_agv("LM3")
    # drive_agv("AP7")

    if not loadURprogrameAndPlay("/programs/take_pic.urp"):
        print("loadURprogrameAndPlay error")
        sys.exit()

    adjustURposeForSet("twoTOtwo")

    # drive_agv("LM5")
