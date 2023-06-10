import numpy as np
import math

# cameraInTcpMat = np.array([[1, 0, 0, 0], [0, 1, 0, -0.12728], [0, 0, 1, 0], [0, 0, 0, 1]])
# print(cameraInTcpMat)
# print(type(cameraInTcpMat))

# angle = math.radians(30)
# angle01 = math.sin(angle)
# print(angle01)
#
# angle02 = np.sin(angle)
# print(angle02)

from interfaces.urPowerOnAndGetSet import GetAndSet
from dashboard_client import DashboardClient
import rtde_receive
import rtde_control

# r = GetAndSet()

# 拿取
# if r.adjustURposeForGet():
#     print("Get OK")
# else:
#     print("Get Error")


# 放置
# if r.adjustURposeForSet():
#     print("Set OK")
# else:
#     print("Set Error")


# c = DashboardClient("192.168.123.238")
# c.connect()
# if c.isConnected():
#     print("UR connect OK")

# c.brakeRelease()

# print(c.safetymode())
# print(c.safetystatus())
# c.powerOff()
# print(c.getRobotModel())
# c.powerOn()
# c.brakeRelease()
# print(c.robotmode())

"""
Robotmode: POWER_OFF
Robotmode: IDLE
Robotmode: RUNNING
"""

# rtdeReceive = rtde_receive.RTDEReceiveInterface("192.168.123.238")
# print(rtdeReceive.getActualTCPPose())
"""
拍照位
pose01:
[0.005018593113787098, -0.2193136383002385, 0.7168432472933255, -0.0031934014287014747, 3.1264406974879426, -0.0015255801544713859]


放置点
pose02:
[-0.07836862220347364, -0.4784920171658646, 0.5787243609784314, -0.04972315995740759, 3.1330555770081077, -0.002105592116972321]


x:0.0833
y:-0.259
z:0.108

"""
# l = [1, 2, 3, 4, 5]
# l.clear()
# print(l)

# from showDemos.baseCommunicate import Comm
#
# c=Comm()
# c.connectStatePort()
# print(c.getLocalizationStatus())

# print(bool(None))
