from dashboard_client import DashboardClient
import time


def powerOnAndReleaseUR():
    startTime = time.time()

    startConnectTime = time.time()
    c = DashboardClient("192.168.123.238")
    try:
        # 循环连接UR，180s内通讯不上代表UR没开机，此时需要报警人为处理
        while True:
            try:
                c.connect()
                if c.isConnected():
                    print("connect UR  ok")
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
        while True:
            try:
                c.brakeRelease()
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


powerOnAndReleaseUR()
