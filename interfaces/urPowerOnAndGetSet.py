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

    def powerOnAndReleaseUR(self):
        startTime = time.time()
        startConnectTime = time.time()
        c = DashboardClient(self.robotID)
        try:
            # 循环连接UR，180s内通讯不上代表UR没开机，此时需要报警人为处理
            while True:
                try:
                    c.connect()
                    if c.isConnected():
                        print("connect UR ok")
                        print("connect time:", time.time() - startConnectTime)
                        break
                    if time.time() - startConnectTime > 180:
                        print("UR power switch is not pushed by human")
                        return False
                except Exception as e:
                    print(e)
                    print("connecting UR ...")
                    time.sleep(1)
                except KeyboardInterrupt:
                    print("UR connection stopped by human,exit")
                    return False

            # 判断UR是否在远程模式,不在远程模式的话报警
            judgeModeStartTime = time.time()
            while True:
                try:
                    if c.isInRemoteControl():
                        print("UR is in remote control mode")
                        print("judge UR if is in remote control mode time:", time.time() - judgeModeStartTime)
                        break
                    if time.time() - judgeModeStartTime > 10:
                        print("UR not in remote mode,exit ")
                        return False
                except Exception as e:
                    print(e)
                    return False

            # 判断UR是否在正常模式,不在的话报警
            while True:
                try:
                    if c.safetymode() == "Safetymode: NORMAL" and c.robotmode() == "Robotmode: POWER_OFF":
                        print("Safetymode: NORMAL OK and UR is in POWER_OFF mode")
                        break
                    else:
                        print("judging UR Mode")
                        time.sleep(1)
                except Exception as e:
                    print(e)
                    print("judging UR Mode error")
                    return False

            # 10s延迟为的是UR刚连上，需要延迟一些时间才能上电，否则上电会失败 ☆☆☆☆☆
            try:
                time.sleep(10)
                c.powerOn()
            except Exception as e:
                print(e)
                print("UR power on error")
                return False

            powerOnStartTime = time.time()
            while True:
                try:
                    if time.time() - powerOnStartTime > 30:
                        print("UR power on step error, exit")
                        return False
                    if c.robotmode() == "Robotmode: IDLE":
                        print("UR is already powered on ")
                        print("UR power on time:", time.time() - powerOnStartTime)
                        break
                    else:
                        print("UR in power on process ")
                        time.sleep(1)
                except Exception as e:
                    print(e)
                    print("UR power on error")
                    return False

            releaseStartTime = time.time()
            c.brakeRelease()
            while True:
                try:
                    if c.robotmode() == "Robotmode: RUNNING":
                        print("breakRelease ok")
                        print("breakRelease ok time:", time.time() - releaseStartTime)
                        c.disconnect()
                        print("UR get ready...")
                        break
                    else:
                        print("UR is breakreleasing")
                        time.sleep(1)
                except Exception as e:
                    print(e)
                    print("UR breakRelease error")
                    return False

            # sleep(10)作用是机械臂刚上完电不加延迟就加载某个程序然后运行的时候会出现加载程序缓慢，运行程序状态位改变很慢的问题
            time.sleep(10)

            print("total time:", time.time() - startTime)
            return True
        except Exception as e:
            print(e)
            return False
        except KeyboardInterrupt as e:
            print(e)
            print("stopped bu human")
            return False
        finally:
            c.disconnect()

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

            # sleep(2)是因为上电后第一次加载程序会比较慢,需要验证
            time.sleep(0.5)
            c.play()
            # sleep(3)是因为上电后第一次加载程序后程序执行状态更新会比较慢，需要验证
            time.sleep(0.5)
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
                data = s.recv(256)
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
        if takePicResult["MatchScore"] < 90:
            print("MatchScore is too low")
            # 拍照出现错误时候重复拍3次，3次连续错误的话再报警
            takePicTimes = 0
            while True:
                if takePicTimes >= 2:
                    takePicResultSign = False
                    print("take pic 3 times over")
                    break
                takePicResult = self.take_pic()
                takePicTimes += 1
                if takePicResult["MatchScore"] >= 90:
                    takePicResultSign = True
                    print("MatchScore is OK")
                    break
                if takePicResult["MatchScore"] < 90:
                    print("MatchScore is too low")
                    time.sleep(1)

            if not takePicResultSign:
                return False
        else:
            print("MatchScore is OK")
        print("takePicResult01:", takePicResult)

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
            if takePicResult["MatchScore"] < 90:
                print("MatchScore is too low")
                # 拍照出现错误时候重复拍3次，3次连续错误的话再报警
                takePicTimes = 0
                while True:
                    if takePicTimes >= 2:
                        takePicResultSign = False
                        print("take pic 3 times over")
                        break
                    takePicResult = self.take_pic()
                    takePicTimes += 1
                    if takePicResult["MatchScore"] >= 90:
                        takePicResultSign = True
                        print("MatchScore is OK")
                        break
                    if takePicResult["MatchScore"] < 90:
                        print("MatchScore is too low")
                        time.sleep(1)

                if not takePicResultSign:
                    return False
            else:
                print("MatchScore is OK")
            print("takePicResult02:", takePicResult)

            x = takePicResult["TranslationX"] / 1000
            y = takePicResult["TranslationY"] / 1000
            print("X:", x)
            print("Y:", y)

            robotReceiveClient = rtde_receive.RTDEReceiveInterface(self.robotID)
            URpose = robotReceiveClient.getActualTCPPose()
            robotReceiveClient.disconnect()

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            URpose = robotControlClient.poseTrans(URpose, [x, y, 0, 0, 0, 0])
            # 只拍照计算位姿，不移动UR，节省时间
            robotControlClient.disconnect()

            URprograme = None
            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            if order == "1To1":
                pose = robotControlClient.poseTrans(URpose,
                                                    [0.083, -0.26, 0.025, math.radians(0.5), math.radians(0.5),
                                                     -math.radians(1)])
                URprograme = "/programs/get_rack_and_back_to_1.urp"
            elif order == "1To2":
                pose = robotControlClient.poseTrans(URpose,
                                                    [0.083, -0.26, 0.025, math.radians(0.5), math.radians(0.5),
                                                     -math.radians(1)])
                URprograme = "/programs/get_rack_and_back_to_2.urp"
            elif order == "2To1":
                pose = robotControlClient.poseTrans(URpose,
                                                    [-0.096, -0.2585, 0.025, math.radians(0.5), math.radians(0.5),
                                                     -math.radians(0.75)])
                URprograme = "/programs/get_rack_and_back_to_1.urp"
            elif order == "2To2":
                pose = robotControlClient.poseTrans(URpose,
                                                    [-0.096, -0.2585, 0.025, math.radians(0.5), math.radians(0.5),
                                                     -math.radians(0.75)])
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
        if takePicResult["MatchScore"] < 90:
            print("MatchScore is too low")
            # 拍照出现错误时候重复拍3次，3次连续错误的话再报警
            takePicTimes = 0
            while True:
                if takePicTimes >= 2:
                    takePicResultSign = False
                    print("take pic 3 times over")
                    break
                takePicResult = self.take_pic()
                takePicTimes += 1
                if takePicResult["MatchScore"] >= 90:
                    takePicResultSign = True
                    print("MatchScore is OK")
                    break
                if takePicResult["MatchScore"] < 90:
                    print("MatchScore is too low")
                    time.sleep(1)

            if not takePicResultSign:
                return False
        else:
            print("MatchScore is OK")
        print("takePicResult01:", takePicResult)

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
            if takePicResult["MatchScore"] < 90:
                print("MatchScore is too low")
                # 拍照出现错误时候重复拍3次，3次连续错误的话再报警
                takePicTimes = 0
                while True:
                    if takePicTimes >= 2:
                        takePicResultSign = False
                        print("take pic 3 times over")
                        break
                    takePicResult = self.take_pic()
                    takePicTimes += 1
                    if takePicResult["MatchScore"] >= 90:
                        takePicResultSign = True
                        print("MatchScore is OK")
                        break
                    if takePicResult["MatchScore"] < 90:
                        print("MatchScore is too low")
                        time.sleep(1)

                if not takePicResultSign:
                    return False
            else:
                print("MatchScore is OK")
            print("takePicResult02:", takePicResult)

            x = takePicResult["TranslationX"] / 1000
            y = takePicResult["TranslationY"] / 1000
            robotReceiveClient = rtde_receive.RTDEReceiveInterface(self.robotID)
            URpose = robotReceiveClient.getActualTCPPose()
            robotReceiveClient.disconnect()

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            URpose = robotControlClient.poseTrans(URpose, [x, y, 0, 0, 0, 0])
            # 只计算不移动
            robotControlClient.disconnect()

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            URprograme01 = URprograme02 = pose01 = pose02 = None
            if order == "1To1":
                pose01 = robotControlClient.poseTrans(URpose,
                                                      [0.080, 0, 0.05, math.radians(0.5), math.radians(0.5),
                                                       -math.radians(1)])
                pose02 = robotControlClient.poseTrans(URpose,
                                                      [0.083, -0.26, 0.05, math.radians(0.5), math.radians(0.5),
                                                       -math.radians(1)])
                URprograme01 = "/programs/get_rack_from_1.urp"
                URprograme02 = "/programs/set_rack_to_1_and_back.urp"
            elif order == "1To2":
                pose01 = robotControlClient.poseTrans(URpose,
                                                      [-0.096, 0, 0.05, math.radians(0.5), math.radians(0.5),
                                                       -math.radians(0.75)])
                pose02 = robotControlClient.poseTrans(URpose,
                                                      [-0.096, -0.2585, 0.05, math.radians(0.5), math.radians(0.5),
                                                       -math.radians(0.75)])
                URprograme01 = "/programs/get_rack_from_1.urp"
                URprograme02 = "/programs/set_rack_to_2_and_back.urp"
            elif order == "2To1":
                pose01 = robotControlClient.poseTrans(URpose,
                                                      [0.080, 0, 0.05, math.radians(0.5), math.radians(0.5),
                                                       -math.radians(1)])
                pose02 = robotControlClient.poseTrans(URpose,
                                                      [0.083, -0.26, 0.05, math.radians(0.5), math.radians(0.5),
                                                       -math.radians(1)])
                URprograme01 = "/programs/get_rack_from_2.urp"
                URprograme02 = "/programs/set_rack_to_1_and_back.urp"
            elif order == "2To2":
                pose01 = robotControlClient.poseTrans(URpose,
                                                      [-0.096, 0, 0.05, math.radians(0.5), math.radians(0.5),
                                                       -math.radians(0.75)])
                pose02 = robotControlClient.poseTrans(URpose,
                                                      [-0.096, -0.2585, 0.05, math.radians(0.5), math.radians(0.5),
                                                       -math.radians(0.75)])
                URprograme01 = "/programs/get_rack_from_2.urp"
                URprograme02 = "/programs/set_rack_to_2_and_back.urp"
            robotControlClient.disconnect()

            if not self.loadURprogrameAndPlay(URprograme01):
                return False

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            robotControlClient.moveJ_IK(pose01)
            robotControlClient.disconnect()

            robotControlClient = rtde_control.RTDEControlInterface(self.robotID)
            robotControlClient.moveL(pose02)
            robotControlClient.disconnect()

            if not self.loadURprogrameAndPlay(URprograme02):
                return False

            return True
        except Exception as e:
            print(e)
            return False


if __name__ == '__main__':
    beginTime = time.time()
    c = GetAndSet()

    # if not c.powerOnAndReleaseUR():
    #     print("UR start error")
    #     sys.exit()

    # if not c.adjustURposeForGet("1To2"):
    #    print("get error")
    # print("OK")

    # c.adjustURposeForSet("1To1")
    # c.adjustURposeForSet("2To2")


    # c.adjustURposeForSet("2To1")
    # c.adjustURposeForSet("1To2")

    # c.adjustURposeForGet("1To2")
    # c.adjustURposeForGet("2To1")
    # c.adjustURposeForGet("2To2")
    c.adjustURposeForGet("1To1")

    # while True:
    #     if not c.adjustURposeForSet("1To1"):
    #         break
    #     if not c.adjustURposeForGet("1To1"):
    #         break
    #     if not c.adjustURposeForSet("1To2"):
    #         break
    #     if not c.adjustURposeForGet("2To1"):
    #         break
    #
    #     if not c.adjustURposeForSet("1To1"):
    #         break
    #     if not c.adjustURposeForGet("1To2"):
    #         break
    #
    #     if not c.adjustURposeForSet("2To1"):
    #         break
    #     if not c.adjustURposeForGet("1To2"):
    #         break
    #     if not c.adjustURposeForSet("2To2"):
    #         break
    #     if not c.adjustURposeForGet("2To2"):
    #         break
    #
    #     if not c.adjustURposeForSet("2To1"):
    #         break
    #     if not c.adjustURposeForGet("1To1"):
    #         break

    # count = 0
    # while True:
    #     if not c.adjustURposeForSet("1To1"):
    #         break
    #
    #     if not c.adjustURposeForSet("2To2"):
    #         break
    #
    #     if not c.adjustURposeForGet("2To2"):
    #         break
    #
    #     if not c.adjustURposeForGet("1To1"):
    #         break
    #     count += 1
    #     print("has already run {} times".format(count))

    # print("total time:", time.time() - beginTime)
