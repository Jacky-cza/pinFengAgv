#!python3
# -*- coding:utf-8 -*-
"""
底盘通讯
"""
import socket
import json
import re


class Comm(object):
    def __init__(self):
        super(Comm, self).__init__()
        self.hostname = "192.168.123.239"
        self.statePort = 19204
        self.navPort = 19206
        self.otherPort = 19210
        self.controlPort = 19205
        self.stateSocket = socket.socket()
        self.navSocket = socket.socket()
        self.otherSocket = socket.socket()
        self.controlSocket = socket.socket()

        self.logger = None

    def setLogger(self, logger):
        self.logger = logger

    def connectStatePort(self):
        """
        连接底盘状态端口
        """
        self.stateSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.stateSocket.settimeout(5.0)
        try:
            if self.stateSocket.connect_ex((self.hostname, self.statePort)):
                self.logger.info('socket connect state port fail', extra={'role': 'project'})
                return False
            self.stateSocket.settimeout(5.0)
        except (TimeoutError, Exception) as e:
            if self.logger:
                self.logger.info('socket connect state port error {}'.format(e), extra={'role': 'project'})
            return False
        return True

    def closeStatePort(self):
        """
        关闭状态端口
        """
        self.stateSocket.close()

    def connectNavPort(self):
        """
        连接导航端口
        """
        self.navSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.navSocket.settimeout(5.0)
        try:
            if self.navSocket.connect_ex((self.hostname, self.navPort)):
                self.logger.info('socket connect nav port fail', extra={'role': 'project'})
                return False
            self.navSocket.settimeout(5.0)
        except (TimeoutError, Exception) as e:
            if self.logger:
                self.logger.info('socket connect Nav Port error {}'.format(e), extra={'role': 'project'})
            return False
        return True

    def closeNavPort(self):
        """
        关闭导航端口
        """
        self.navSocket.close()

    def connectOtherPort(self):
        """
        连接其他端口
        """
        self.otherSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.otherSocket.settimeout(5.0)
        try:
            if self.otherSocket.connect_ex((self.hostname, self.otherPort)):
                self.logger.info('socket connect other port fail', extra={'role': 'project'})
                return False
            self.otherSocket.settimeout(5.0)
        except (TimeoutError, Exception) as e:
            if self.logger:
                self.logger.info('connect other port error {}'.format(e), extra={'role': 'project'})
            return False
        return True

    def closeOtherPort(self):
        """
        关闭其他端口
        """
        self.otherSocket.close()

    def connectControlPort(self):
        """
        连接控制端口
        """
        self.controlSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.controlSocket.settimeout(5.0)
        try:
            if self.controlSocket.connect_ex((self.hostname, self.controlPort)):
                self.logger.info('socket connect state port fail', extra={'role': 'project'})
                return False
            self.controlSocket.settimeout(5.0)
        except (TimeoutError, Exception) as e:
            if self.logger:
                self.logger.info('connect control port error {}'.format(e), extra={'role': 'project'})
            return False
        return True

    def closeControlPort(self):
        """
        关闭控制端口
        """
        self.controlSocket.close()

    def queryRobotPose(self, pose):
        """
        获取机器人位置
        """
        # one byte      one byte            two bytes                       four bytes          two bytes
        # header        protocol version    order(to check the response)    size of data area   serial number
        packet = [0x5A, 0x01, 0x11, 0x22, 0x00, 0x00, 0x00, 0x00, 0x03, 0xEC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        # send
        try:
            self.stateSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket connect query robot Pose error {}'.format(e), extra={'role': 'project'})
            return False
        # recv and regular check
        recvData = bytes()
        regu = b'\\x5A\\x01\\x11\\x22.{4}\\x2A\\xFC\\x03\\xec\\x00{4}'
        try:
            while True:
                recvData += self.stateSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket receive query robot pose error {}'.format(e), extra={'role': 'project'})
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
            pose['x'] = jsonObj['x']
            pose['y'] = jsonObj['y']
            pose['theta'] = jsonObj['angle']
            pose['station'] = jsonObj['current_station']
            pose['confidence'] = jsonObj['confidence']

        except json.decoder.JSONDecodeError as e:
            return False
        except (KeyError, Exception) as e:
            return False
        return True

    def queryBattery(self, power):
        """
        获取机器人电量
        """
        packet = [0x5A, 0x01, 0x22, 0x33]
        j = dict()
        j['simple'] = False
        strTmp = json.dumps(j)
        # construct
        packet = packet + list(len(strTmp).to_bytes(length=4, byteorder='big', signed=False)) \
                 + [0x03, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] \
                 + list(bytes(strTmp, encoding='utf-8'))
        # send
        try:
            self.stateSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        # recv and regular check
        recvData = bytes()
        regu = b'\\x5A\\x01\\x22\\x33.{4}\\x2A\\xFF\\x03\\xEF\\x00{4}'
        try:
            while True:
                recvData += self.stateSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
            power['power'] = jsonObj['battery_level']
            power['charging'] = jsonObj['charging']
            # print(power)
        except json.decoder.JSONDecodeError as e:
            return False
        except (KeyError, Exception) as e:
            return False
        return True

    # 导航到固定站点
    """
    :param target includes target station name
    :param method means forward or backward (default valus is in configuration)
    """

    def navigateToStation(self, target, method=None):
        packet = [0x5A, 0x01, 0x33, 0x44]
        j = dict()
        j['id'] = target
        j['method'] = method
        strTmp = json.dumps(j)
        packet = packet + list(len(strTmp).to_bytes(length=4, byteorder='big', signed=False)) \
                 + [0x0B, 0xEB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] \
                 + list(bytes(strTmp, encoding='utf-8'))
        try:
            self.navSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        recvData = bytes()
        regu = b'\\x5A\\x01\\x33\\x44.{4}\\x32\\xFB\\x0B\\xEB\\x00{4}'
        try:
            while True:
                recvData += self.navSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            return False
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
            # print(jsonObj)
        except (json.decoder.JSONDecodeError, Exception) as e:
            return False
        return True

    # 暂停导航
    def pause_navigation(self):
        packet = [0x5A, 0x01, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00]
        packet += [0x0B, 0xB9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.navSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        recvData = bytes()
        regu = b'\\x5A\\x01\\x01\\x02.{4}\\x32\\xC9\\x0B\\xBB\\x00{4}'
        try:
            while True:
                recvData += self.navSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            return False
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
        except (json.decoder.JSONDecodeError, Exception) as e:
            pass
        return True

    # 继续导航(相对于暂停导航)
    def continue_navigation(self):
        packet = [0x5A, 0x01, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00]
        packet += [0x0B, 0xBA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.navSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        recvData = bytes()
        regu = b'\\x5A\\x01\\x01\\x02.{4}\\x32\\xCA\\x0B\\xBB\\x00{4}'
        try:
            while True:
                recvData += self.navSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            return False
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
        except (json.decoder.JSONDecodeError, Exception) as e:
            pass
        return True

    # 取消导航(取消当前的导航命令)
    def cancle_navigation(self):
        packet = [0x5A, 0x01, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00]
        packet += [0x0B, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.navSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        recvData = bytes()
        regu = b'\\x5A\\x01\\x01\\x02.{4}\\x32\\xCB\\x0B\\xBB\\x00{4}'
        try:
            while True:
                recvData += self.navSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            return False
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
        except (json.decoder.JSONDecodeError, Exception) as e:
            pass
        return True

    # 获取导航状态 2 or 4
    def getNavigationState(self, tasks):
        packet = [0x5A, 0x01, 0x55, 0x66, 0x00, 0x00, 0x00, 0x00]
        packet = packet + [0x03, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.stateSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        recvData = bytes()
        regu = b'\\x5A\\x01\\x55\\x66.{4}\\x2B\\x0C\\x03\\xFC\\x00{4}'
        try:
            while True:
                recvData += self.stateSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            pass
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
        except (json.decoder.JSONDecodeError, Exception) as e:
            return False
        try:
            """
            0   None
            1   waiting
            2   running
            3   suspended
            4   completed
            5   failed
            6   canceled
            """
            tasks['status'] = jsonObj['task_status']
            """
            0   没有导航
            1   自由导航到任意点
            2   自由导航到站点
            3   路径导航到站点
            7   平动转动
            100 其他
            """
            tasks['type'] = jsonObj['task_type']
            tasks['station'] = jsonObj['target_id']
            tasks['finished_path'] = jsonObj['finished_path']
            tasks['unfinished_path'] = jsonObj['unfinished_path']
        except KeyError:
            tasks['status'] = 0
            tasks['type'] = None
            tasks['station'] = None
            tasks['finished_path'] = None
            tasks['unfinished_path'] = None
        # print(tasks)
        return True

    """
    :brief  play sound alarm
    :param sound    sound music name
    :param play     play or not play
    """

    # 播放声音
    def playErrorSound(self, sound, play):
        if play:
            packet = [0x5A, 0x01, 0x66, 0x77]
            j = dict()
            j['name'] = sound
            j['loop'] = True
            strTmp = json.dumps(j)
            packet = packet + list(len(strTmp).to_bytes(length=4, byteorder='big', signed=False)) \
                     + [0x17, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] \
                     + list(bytes(strTmp, encoding='utf-8'))
            try:
                self.otherSocket.send(bytes(packet))
            except (ConnectionResetError, Exception) as e:
                if self.logger:
                    self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
                return False
            recvData = bytes()
            regu = b'\\x5A\\x01\\x66\\x77.{4}\\x3E\\x80\\x17\\x70\\x00{4}'
            try:
                while True:
                    recvData += self.otherSocket.recv(256)
                    m = re.search(regu, recvData)
                    if m:
                        size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                        if len(recvData) >= (size + 16):
                            recvData = recvData[m.end():m.end() + size]
                            break
            except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
                if self.logger:
                    self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
                return False
            try:
                jsonObj = json.loads(recvData.decode(encoding='utf-8'))
            except (json.decoder.JSONDecodeError, Exception) as e:
                pass
            return True
        else:
            packet = [0x5A, 0x01, 0x66, 0x77, 0x00, 0x00, 0x00, 0x00]
            packet = packet + [0x17, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            try:
                self.otherSocket.send(bytes(packet))
            except (ConnectionResetError, Exception) as e:
                if self.logger:
                    self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
                return False
            recvData = bytes()
            regu = b'\\x5A\\x01\\x66\\x77.{4}\\x3E\\x8C\\x17\\x7C\\x00{4}'
            try:
                while True:
                    recvData += self.otherSocket.recv(256)
                    m = re.search(regu, recvData)
                    if m:
                        size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                        if len(recvData) >= (size + 16):
                            recvData = recvData[m.end():m.end() + size]
                            break
            except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
                if self.logger:
                    self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
                return False
            try:
                jsonObj = json.loads(recvData.decode(encoding='utf-8'))
            except (json.decoder.JSONDecodeError, Exception) as e:
                pass
            return True

    """
    :brief  get DI data
    :param di_index   the number of DI 
    """
    # 复位按钮
    def getDIStatus(self, di_index):
        packet = [0x5A, 0x01, 0x77, 0x88, 0x00, 0x00, 0x00, 0x00]
        packet = packet + [0x03, 0xF5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.stateSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        recvData = bytes()
        regu = b'\\x5A\\x01\\x77\\x88.{4}\\x2B\\x05\\x03\\xF5\\x00{4}'
        try:
            while True:
                recvData += self.stateSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            pass
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
            return jsonObj['DI'][di_index]['status']
        except json.decoder.JSONDecodeError as e:
            return False
        except (KeyError, Exception) as e:
            return False
        # return True

    """
    :brief  set emergency 
    :param value    True, set;False, reset
    """
    # 远程软急停
    def setSoftEmergency(self, value):
        packet = [0x5A, 0x01, 0x88, 0x99]
        j = dict()
        j['status'] = value
        strTmp = json.dumps(j)
        packet = packet + list(len(strTmp).to_bytes(length=4, byteorder='big', signed=False)) \
                 + [0x17, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] \
                 + list(bytes(strTmp, encoding='utf-8'))
        try:
            self.otherSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        recvData = bytes()
        regu = b'\\x5A\\x01\\x88\\x99.{4}\\x3E\\x84\\x17\\x74\\x00{4}'
        try:
            while True:
                recvData += self.otherSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            return False
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
        except (json.decoder.JSONDecodeError, Exception) as e:
            return False
        return True

    # 获取急停按钮状态
    """
    @:brief     get emergency button status
    """
    def getEmergencyStatus(self):
        packet = [0x5A, 0x01, 0x11, 0x44, 0x00, 0x00, 0x00, 0x00]
        packet = packet + [0x03, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.stateSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return {}
        recvData = bytes()
        regu = b'\\x5A\\x01\\x11\\x44.{4}\\x2B\\x04\\x03\\xF4\\x00{4}'
        try:
            while True:
                recvData += self.stateSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            pass
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
        except (json.decoder.JSONDecodeError, Exception) as e:
            return {}
        return jsonObj

    # 获取重定位状态，重定位是否成功
    """
    @brief: get relocation status   success is ok, status 'completed' needs to be confirmed
    """
    def getLocalizationStatus(self):
        packet = [0x5A, 0x01, 0x99, 0x10, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFD]
        packet = packet + [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.stateSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return 'FAILED'
        recvData = bytes()
        regu = b'\\x5A\\x01\\x99\\x10.{4}\\x2B\\x0D\\x03\\xFD\\x00{4}'
        try:
            while True:
                recvData += self.stateSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            return 'FAILED'
        jsonObj = {}
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
        except json.decoder.JSONDecodeError as e:
            pass
        try:
            if jsonObj['reloc_status'] == 0:
                return 'FAILED'
            elif jsonObj['reloc_status'] == 1:
                return 'SUCCESS'
            elif jsonObj['reloc_status'] == 2:
                return 'RELOCING'
            elif jsonObj['reloc_status'] == 3:
                return 'COMPLETED'
        except (KeyError, Exception) as e:
            return 'FAILED'

    """
    relocation robot with robotHome params
    """
    # 小车重定位，用于小车被人为移动了位置
    def reLocation(self):
        packet = [0x5A, 0x01, 0x11, 0x33]
        j = dict()
        j['x'] = 0
        j['y'] = 0
        j['angle'] = 0
        j['home'] = True
        strTmp = json.dumps(j)
        packet = packet + list(len(strTmp).to_bytes(length=4, byteorder='big', signed=False)) \
                 + [0x07, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] \
                 + list(bytes(strTmp, encoding='utf-8'))
        try:
            self.controlSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        recvData = bytes()
        regu = b'\\x5A\\x01\\x11\\x33.{4}\\x2E\\xE2\\x07\\xD2\\x00{4}'
        try:
            while True:
                recvData += self.controlSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            pass
        try:
            jsonObj = json.loads(recvData.decode(encoding='utf-8'))
            # print(jsonObj)
            return True
        except (json.decoder.JSONDecodeError, Exception) as e:
            return False

    # 确认小车目前所在的位置，类似robotshop里面开机后需要点击确定小车所在位置一样
    """
    confirm location is ok
    """
    def confirm_location(self):
        packet = [0x5A, 0x01, 0x22, 0x44, 0x00, 0x00, 0x00, 0x00]
        packet = packet + [0x07, 0xD3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            self.controlSocket.send(bytes(packet))
        except (ConnectionResetError, Exception) as e:
            if self.logger:
                self.logger.info('socket send error {}'.format(e), extra={'role': 'project'})
            return False
        recvData = bytes()
        regu = b'\\x5A\\x01\\x22\\x44.{4}\\x2E\\xE3\\x07\\xD3\\x00{4}'
        try:
            while True:
                recvData += self.controlSocket.recv(256)
                m = re.search(regu, recvData)
                if m:
                    size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    if len(recvData) >= (size + 16):
                        recvData = recvData[m.end():m.end() + size]
                        break
                return True
        except (ConnectionResetError, ConnectionAbortedError, Exception) as e:
            if self.logger:
                self.logger.info('socket recv error {}'.format(e), extra={'role': 'project'})
            return False


