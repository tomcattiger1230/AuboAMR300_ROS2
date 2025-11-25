from MessageManager import MessageManager
from RobotConfig import AGV_IP
from RobotConfig import AGV_STATUS_PORT, AGV_NAVIGATION_PORT, AGV_CONTROL_PORT
import socket


class AgvControl():
    messagemanager = MessageManager()

    def __init__(self) -> None:
        pass

    # AGV运动状态
    def AGV_Status(self):
        Socket_Status = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket_Status.connect((AGV_IP, AGV_STATUS_PORT))
        Socket_Status.settimeout(5)
        # 查询机器人导航状态
        message = self.messagemanager.PackMessage(1, 1020, {"simple": True})
        Socket_Status.send(message)
        info_dict = self.messagemanager.UnpackMessage(Socket_Status)
        # print('info_dict: ', info_dict)
        info = info_dict["task_status"]
        # print('info: ', info)
        return info

    # AGV路径导航
    def AGV_Navigation(self, source_id, target_id):
        Socket_Navigation = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket_Navigation.connect((AGV_IP, AGV_NAVIGATION_PORT))
        Socket_Navigation.settimeout(5)
        # 执行AGV导航
        message = self.messagemanager.PackMessage(1, 3051, {"source_id": source_id, "id": target_id, "task_id": ''})
        # print("\n\nreq:")
        # print(' '.join('{:02X}'.format(x) for x in message))
        Socket_Navigation.send(message)

    def AGV_Battery_Charge(self):
        Socket_Status = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket_Status.connect((AGV_IP, AGV_STATUS_PORT))
        Socket_Status.settimeout(5)
        # 查询机器人导航状态
        message = self.messagemanager.PackMessage(1, 1007, {"charging": True})
        # print("\n\nreq:")
        # print(' '.join('{:02X}'.format(x) for x in message))
        Socket_Status.send(message)
        info_dict = self.messagemanager.UnpackMessage(Socket_Status)
        # print('info_dict: ', info_dict)
        info = info_dict["charging"]
        # print('info: ', info)
        return info

    def AGV_Battery_Level(self):
        Socket_Status = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket_Status.connect((AGV_IP, AGV_STATUS_PORT))
        Socket_Status.settimeout(5)
        # 查询机器人导航状态
        message = self.messagemanager.PackMessage(1, 1007, {"battery_level": True})
        # print("\n\nreq:")
        # print(' '.join('{:02X}'.format(x) for x in message))
        Socket_Status.send(message)
        info_dict = self.messagemanager.UnpackMessage(Socket_Status)
        # print('info_dict: ', info_dict)
        info = info_dict["battery_level"]
        # print('info: ', info)
        return info

    # AGV重定位
    def AGV_Relocal(self, source_id, target_id):
        Socket_Relocal = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket_Relocal.connect((AGV_IP, AGV_CONTROL_PORT))
        Socket_Relocal.settimeout(5)
        # 执行AGV导航
        message = self.messagemanager.PackMessage(1, 2002, {"source_id": source_id, "id": target_id, "task_id": ''})
        # print(' '.join('{:02X}'.format(x) for x in message))
        Socket_Relocal.send(message)

    # 查询电池温度
    def AGV_Battery_Temp(self):
        Socket_Temp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket_Temp.connect((AGV_IP, AGV_STATUS_PORT))
        Socket_Temp.settimeout(5)
        # 查询机器人导航状态
        message = self.messagemanager.PackMessage(1, 1007, {"battery_temp": True})
        # print(' '.join('{:02X}'.format(x) for x in message))
        Socket_Temp.send(message)
        info_dict = self.messagemanager.UnpackMessage(Socket_Temp)
        # print('info_dict: ', info_dict)
        info = info_dict["battery_temp"]
        # print('info: ', info)
        return info

    # 查询电机速度
    def AGV_motor_speed(self):
        Socket_speed = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket_speed.connect((AGV_IP, AGV_STATUS_PORT))
        Socket_speed.settimeout(5)
        # 查询机器人导航状态
        message = self.messagemanager.PackMessage(1, 1040, {"speed": True})
        # print(' '.join('{:02X}'.format(x) for x in message))
        Socket_speed.send(message)
        info_dict = self.messagemanager.UnpackMessage(Socket_speed)
        # print('info_dict: ', info_dict)
        info = info_dict["speed"]
        # print('info: ', info)
        return info

    # 查询电机电压
    def AGV_motor_current(self):
        Socket_current = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket_current.connect((AGV_IP, AGV_STATUS_PORT))
        Socket_current.settimeout(5)
        # 查询机器人导航状态
        message = self.messagemanager.PackMessage(1, 1040, {"current": True})
        # print(' '.join('{:02X}'.format(x) for x in message))
        Socket_current.send(message)
        info_dict = self.messagemanager.UnpackMessage(Socket_current)
        # print('info_dict: ', info_dict)
        info = info_dict["current"]
        # print('info: ', info)
        return info

    # 查询电机电压
    def AGV_motor_voltage(self):
        Socket_voltage = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket_voltage.connect((AGV_IP, AGV_STATUS_PORT))
        Socket_voltage.settimeout(5)
        # 查询机器人导航状态
        message = self.messagemanager.PackMessage(1, 1040, {"voltage": True})
        # print(' '.join('{:02X}'.format(x) for x in message))
        Socket_voltage.send(message)
        info_dict = self.messagemanager.UnpackMessage(Socket_voltage)
        # print('info_dict: ', info_dict)
        info = info_dict["voltage"]
        # print('info: ', info)
        return info