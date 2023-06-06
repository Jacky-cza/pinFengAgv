from dashboard_client import DashboardClient

c = DashboardClient("192.168.1.238")
c.connect(timeout_ms=5000)
if c.isConnected():
    print("UR connect OK")

# c.powerOn()
# c.brakeRelease()

# c.powerOn()
# c.brakeRelease()
# if c.isInRemoteControl():
#     c.powerOn()
# print(c.robotmode())
# c.powerOn()
# c.brakeRelease()

# Robotmode: POWER_OFF
# Robotmode: IDLE
# Robotmode: RUNNING
