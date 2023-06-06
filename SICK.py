import socket
import json

s = socket.socket()
ip = "192.168.123.237"
port = 3000

s.connect((ip, port))
s.send('\002trigger\003'.encode())
"""
\x02trigger\x03
"""
str = ""
times = 0
while True:
    data = s.recv(1024)
    str += data.decode()
    times += 1
    if times == 2:
        break

print(str)
print(type(str))
res = str.split("\x03")[1]
print(res)
print(type(res))
finnal_res = json.loads(res)
print("Pass:", finnal_res["Pass"])
print("MatchScore:", finnal_res["MatchScore"])
print("TranslationX:", finnal_res["TranslationX"])
print("TranslationY:", finnal_res["TranslationY"])
print("Angle", finnal_res["Angle"])
