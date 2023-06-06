import sys
from dashboard_client import DashboardClient
import rtde_receive
import rtde_control
import math
import time
import socket
import json


class GetAndSet:
    def __init__(self):
        self.robotID = "192.168.123.238"
        self.cameraIpAndPort = ("192.168.123.237", 3000)

    def loadURprogrameAndPlay(self, urPrograme):
        c = DashboardClient(self.robotID)
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

    def take_pic(self):
        s = socket.socket()
        try:
            s.settimeout(3)
            s.connect(self.cameraIpAndPort)
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

    def adjustURposeForGet(self, order):
        if order not in ["1To1", "1To2", "2To1", "2To2"]:
            print("order is error")
            sys.exit()

        if not self.loadURprogrameAndPlay("/programs/take_pic.urp"):
            print("loadURprogrameAndPlay error")
            return False

        takePicResult = self.take_pic()
        if not takePicResult:
            return False
        print("takePicResult01:", takePicResult)
        if takePicResult["MatchScore"] < 80:
            print("MatchScore is too low")
            return False
        else:
            print("MatchScore is OK")

        try:
            robotReceiveClient = rtde_receive.RTDEReceiveInterface(self.robotID)
            URpose = robotReceiveClient.getActualTCPPose()
            robotReceiveClient.disconnect()
            print("URpose:")
            print(URpose)

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            angle = math.radians(takePicResult["Angle"])
            print("angle", angle)
            pose = robotControlClient.poseTrans(URpose, [0, 0, 0, 0, 0, angle])
            robotControlClient.moveJ_IK(pose)
            robotControlClient.disconnect()

            takePicResult = self.take_pic()
            if not takePicResult:
                return False
            print("takePicResult02:", takePicResult)
            if takePicResult["MatchScore"] < 80:
                print("MatchScore is too low")
                sys.exit()
            else:
                print("MatchScore is OK")

            x = takePicResult["TranslationX"] / 1000
            y = takePicResult["TranslationY"] / 1000
            robotReceiveClient = rtde_receive.RTDEReceiveInterface(self.robotID)
            URpose = robotReceiveClient.getActualTCPPose()
            robotReceiveClient.disconnect()

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            pose = robotControlClient.poseTrans(URpose, [x, y, 0, 0, 0, 0])
            robotControlClient.moveL(pose)
            robotControlClient.disconnect()

            takePicResult = self.take_pic()
            if not takePicResult:
                return False
            print("takePicResult03:", takePicResult)
            if takePicResult["MatchScore"] < 80:
                print("MatchScore is too low")
                sys.exit()
            else:
                print("MatchScore is OK")

            robotReceiveClient = rtde_receive.RTDEReceiveInterface(self.robotID)
            URpose = robotReceiveClient.getActualTCPPose()
            robotReceiveClient.disconnect()

            URprograme = None
            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            if order == "1To1":
                pose = robotControlClient.poseTrans(URpose, [0.081, -0.261, 0.11, 0, math.radians(1), -math.radians(1)])
                URprograme = "/programs/get_rack_and_back_to_1.urp"
            elif order == "1To2":
                pose = robotControlClient.poseTrans(URpose, [0.081, -0.261, 0.11, 0, math.radians(1), -math.radians(1)])
                URprograme = "/programs/get_rack_and_back_to_2.urp"
            elif order == "2To1":
                pose = robotControlClient.poseTrans(URpose,
                                                    [-0.0977, -0.256, 0.11, 0, math.radians(1), -math.radians(1)])
                URprograme = "/programs/get_rack_and_back_to_1.urp"
            elif order == "2To2":
                pose = robotControlClient.poseTrans(URpose,
                                                    [-0.0977, -0.256, 0.11, 0, math.radians(1), -math.radians(1)])
                URprograme = "/programs/get_rack_and_back_to_2.urp"

            robotControlClient.moveJ_IK(pose)
            robotControlClient.disconnect()

            if not self.loadURprogrameAndPlay(URprograme):
                return False

            return True
        except Exception as e:
            print(e)
        return False

    def adjustURposeForSet(self, order):
        if order not in ["1To1", "1To2", "2To1", "2To2"]:
            print("order is error")
            sys.exit()

        if not self.loadURprogrameAndPlay("/programs/take_pic.urp"):
            print("loadURprogrameAndPlay error")
            return False

        takePicResult = self.take_pic()
        if not takePicResult:
            return False
        print("takePicResult01:", takePicResult)
        if takePicResult["MatchScore"] < 80:
            print("MatchScore is too low")
            return False
        else:
            print("MatchScore is OK")

        try:
            robotReceiveClient = rtde_receive.RTDEReceiveInterface(self.robotID)
            URpose = robotReceiveClient.getActualTCPPose()
            print("URpose:")
            print(URpose)
            robotReceiveClient.disconnect()

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            angle = math.radians(takePicResult["Angle"])
            print("angle", angle)
            pose = robotControlClient.poseTrans(URpose, [0, 0, 0, 0, 0, angle])
            print("pose:", pose)
            # moveJ_IK 执行的时候是阻塞的，只有执行完之后才会执行下面的命令，所以使用它移动robot的时候不需要轮询判断robot有没有到位
            robotControlClient.moveJ_IK(pose)
            robotControlClient.disconnect()

            takePicResult = self.take_pic()
            if not takePicResult:
                return False
            print("takePicResult02:", takePicResult)
            if takePicResult["MatchScore"] < 80:
                print("MatchScore is too low")
                sys.exit()
            else:
                print("MatchScore is OK")

            x = takePicResult["TranslationX"] / 1000
            y = takePicResult["TranslationY"] / 1000
            robotReceiveClient = rtde_receive.RTDEReceiveInterface(self.robotID)
            URpose = robotReceiveClient.getActualTCPPose()
            robotReceiveClient.disconnect()

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            pose = robotControlClient.poseTrans(URpose, [x, y, 0, 0, 0, 0])
            robotControlClient.moveL(pose)
            robotControlClient.disconnect()

            takePicResult = self.take_pic()
            if not takePicResult:
                return False
            print("takePicResult03:", takePicResult)
            if takePicResult["MatchScore"] < 80:
                print("MatchScore is too low")
                sys.exit()
            else:
                print("MatchScore is OK")

            robotReceiveClient = rtde_receive.RTDEReceiveInterface(self.robotID)
            URpose = robotReceiveClient.getActualTCPPose()
            robotReceiveClient.disconnect()

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            URprograme01 = URprograme02 = None
            if order == "1To1":
                pose = robotControlClient.poseTrans(URpose, [0.081, -0.259, 0.11, 0, math.radians(1), -math.radians(1)])
                URprograme01 = "/programs/get_rack_from_1.urp"
                URprograme02 = "/programs/set_rack_to_1_and_back.urp"
            elif order == "1To2":
                pose = robotControlClient.poseTrans(URpose,
                                                    [-0.0977, -0.256, 0.11, 0, math.radians(1), -math.radians(1)])
                URprograme01 = "/programs/get_rack_from_1.urp"
                URprograme02 = "/programs/set_rack_to_2_and_back.urp"
            elif order == "2To1":
                pose = robotControlClient.poseTrans(URpose, [0.081, -0.259, 0.11, 0, math.radians(1), -math.radians(1)])
                URprograme01 = "/programs/get_rack_from_2.urp"
                URprograme02 = "/programs/set_rack_to_1_and_back.urp"
            elif order == "2To2":
                pose = robotControlClient.poseTrans(URpose,
                                                    [-0.0977, -0.256, 0.11, 0, math.radians(1), -math.radians(1)])
                URprograme01 = "/programs/get_rack_from_2.urp"
                URprograme02 = "/programs/set_rack_to_2_and_back.urp"
            robotControlClient.disconnect()

            if not self.loadURprogrameAndPlay(URprograme01):
                return False

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            robotControlClient.moveJ_IK(pose)
            robotControlClient.disconnect()

            if not self.loadURprogrameAndPlay(URprograme02):
                return False

            return True
        except Exception as e:
            print(e)
            return False


if __name__ == '__main__':
    c = GetAndSet()

    c.adjustURposeForGet("1To1")
    # c.adjustURposeForGet("1To2")
    # c.adjustURposeForGet("2To1")
    # c.adjustURposeForGet("2To2")

    # c.adjustURposeForSet("2To1")
    # c.adjustURposeForSet("2To2")
    # c.adjustURposeForSet("1To1")
    # c.adjustURposeForSet("1To2")
