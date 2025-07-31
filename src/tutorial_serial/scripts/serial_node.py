#!/usr/bin/env python
import collections
import rospy
import serial
import struct
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, TransformStamped, PoseStamped
from tf2_ros import StaticTransformBroadcaster
import tf2_ros
import rosbag
import math
from tf.transformations import euler_from_quaternion
from tutorial_vision.msg import StringStamped  # 导入二维码消息类型
from std_msgs.msg import UInt8, Int32
from std_msgs.msg import Bool
from tutorial_serial.msg import SerialData
from tutorial_serial.msg import WayPoint, WayPointArry, AninmalData
from tutorial_vision.msg import Aninmal
class IMUSerialNode:
    def __init__(self):
        rospy.init_node('serial_node', log_level=rospy.INFO)
        #self.serial_port = serial.Serial('/dev/ttyserial', 9600, timeout=1)
        try:
            self.serial_port = serial.Serial('/dev/ttyserial', 9600, timeout=1)
        except Exception as e:
            rospy.logerr("Failed to open serial port")

        self.coordinateConverter = CoordinateConverter()
        self.waypointController = WaypointController()
        self.buffer = bytearray()
        self.receiving = False
        self.length = 0
        self.func = 0
        self.current = 0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.fly_task = 1

        self.yaw_symbol = 0

        self.target = 0

        self.count = 0

        self.deny_fly_1 = None
        self.deny_fly_2 = None
        self.deny_fly_3 = None

        self.laser_status = 0
        self.camera_pos = 0
        
        self._shoot_idx = 0

        self.camera_en_pub = rospy.Publisher("camera_en", Bool, queue_size=10)
        self.waypoint_reuqest_pub = rospy.Publisher("waypoint_request", WayPointArry, queue_size=10)
        self.clear_waypoint_pub = rospy.Publisher("clear_waypoint", UInt8, queue_size=10)
        self.camera_pos_pub = rospy.Publisher("camera_pos", UInt8, queue_size=10)
        self.laser_status_pub = rospy.Publisher("laser_status", UInt8, queue_size=10)
        # 订阅二维码检测结果
        #self.qr_sub = rospy.Subscriber("qr_detect_result", StringStamped, self.qr_callback)
        self.fly_task_sub = rospy.Subscriber("fly_task", Int32, self.fly_task_callback)
        self.yaw_symbol_sub = rospy.Subscriber("yaw_symbol", Int32, self.yaw_symbol_callback)
        self.true_pos_sub = rospy.Subscriber("true_position", PoseStamped, self.true_pos_callback)
        self.serial_ctrl_sub = rospy.Subscriber("serial_ctrl", SerialData, self.serial_ctrl_callback)
        self.yolo_result_sub = rospy.Subscriber("yolo_result", Aninmal, self.yolo_result_callback)


        # TF2 相关
        self.tf_broadcaster = StaticTransformBroadcaster()
        rospy.Timer(rospy.Duration(0.001), self.read_serial)

        self.camera_pos_down() #摄像头下看
        self.laser_on()#打激光
        #self.send_packet(0x0A, 0x01) #蜂鸣

    def read_serial(self, event):
        try:
            while self.serial_port.in_waiting > 0:
                byte = self.serial_port.read(1)
                if not self.receiving and byte == b'\xAA':
                    self.receiving = True
                    self.buffer = bytearray()
                    self.func = 0
                    self.current = 0
                    self.length = 0
                elif self.receiving:
                    if self.current == 0:
                        self.func = ord(byte)
                        self.current += 1
                    elif self.current == 1:
                        self.length = ord(byte)
                        self.current += 1
                    elif self.current - 2 < self.length:
                        self.buffer.extend(byte)
                        self.current += 1
                    elif self.current - 2 == self.length:
                        if byte == b'\xAF':
                            self.deal_with_data(self.func, self.length, self.buffer)
                        self.receiving = False
        except Exception as e:
            pass
            #rospy.logerr("Failed to read serial port: %s", str(e))

    def yolo_result_callback(self, data):
        self.send_animnal(data.type, self.waypointController._full_path[self._shoot_idx - 1][0], self.waypointController._full_path[self._shoot_idx - 1][1], data.number)
        #self.send_animnal(data.type, data.x, data.y, data.number)

    def serial_ctrl_callback(self, data):
        self.send_packet(data.func,data.data)
    def send_packet(self, func , data):
        packet = bytearray()
        packet.append(0xAA)
        packet.append(func)
        packet.append(0x01)
        packet.append(data)
        packet.append(0xAF)
        rospy.loginfo("Sending packet: %s", ' '.join(format(byte, '02x') for byte in packet))
        try:    
            self.serial_port.write(packet)
        except Exception as e:
            rospy.logerr("Failed to write to serial port: %s", str(e))

    def camera_pos_forward(self):
        if self.camera_pos == 0:
            rospy.loginfo("Camera is already in forward position")
            return
        self.camera_pos = 0
        m = UInt8()
        m.data = self.camera_pos
        self.camera_pos_pub.publish(m)

    def camera_pos_down(self):
        if self.camera_pos == 1:
            rospy.loginfo("Camera is already in down position")
            return
        self.send_packet(0x04, 0x01)
        self.camera_pos = 1
        m = UInt8()
        m.data = self.camera_pos
        self.camera_pos_pub.publish(m)

    def laser_on(self):
        if self.laser_status == 1:
            rospy.loginfo("Laser is already on")
            return
        self.send_packet(0x08, 0x01)
        self.laser_status = 1
        m = UInt8()
        m.data = self.laser_status
        self.laser_status_pub.publish(m)
    
    def laser_off(self):
        if self.laser_status == 0:
            rospy.loginfo("Laser is already off")
            return
        self.send_packet(0x08, 0x00)
        self.laser_status = 0
        m = UInt8()
        m.data = self.laser_status
        self.laser_status_pub.publish(m)



    def deal_with_data(self, func, length, data):
        if(func == 0x01):
            self.deny_fly_1 = (data[0], data[1])
            rospy.loginfo("deny_fly_1: %s", self.deny_fly_1)
        elif(func == 0x02):
            self.deny_fly_2 = (data[0], data[1])
            rospy.loginfo("deny_fly_2: %s", self.deny_fly_2)
        elif(func == 0x03):
            self.deny_fly_3 = (data[0], data[1])
            rospy.loginfo("deny_fly_3: %s", self.deny_fly_3)
            if self.deny_fly_1 and self.deny_fly_2 and self.deny_fly_3:
                no_fly_map_coords = [self.deny_fly_1, self.deny_fly_2, self.deny_fly_3]
                rospy.loginfo("No-fly zone coordinates: %s", no_fly_map_coords)
                self.waypointController.plan_mission(no_fly_map_coords)
                self.clear_waypoint_pub.publish(UInt8(1))  # 清除之前的航点
                rospy.loginfo("Mission planning complete.")
                WayPointArry_msg = WayPointArry()
                packet = bytearray()
                packet.append(0xAA)  # Frame header
                packet.append(0x11)
                packet.append(len(self.waypointController._full_path) * 2)  # Data length (1 byte for waypoint)
                for i in range(len(self.waypointController._full_path)):
                    (mx,my) , is_new = self.waypointController.get_next_waypoint()
                    ax, ay = self.coordinateConverter.map_to_aircraft((mx, my))
                    packet.append(mx)
                    packet.append(my)

                for point in self.waypointController.easy_path:
                    p = Point()
                    p.x = point[0]
                    p.y = point[1]
                    p.z = 0
                    WayPointArry_msg.points.append(p)
                packet.append(0xAF)
                self.serial_port.write(packet)
                self.waypoint_reuqest_pub.publish(WayPointArry_msg)


    def send_waypoint(self, waypoint):
        packet = bytearray()
        packet.append(0xAA)  # Frame header
        packet.append(0x11)  # Function code (custom)
        packet.append(0x02)  # Data length (2 byte for waypoint)
        packet.append(waypoint[0])  # x coordinate
        packet.append(waypoint[1])  # y coordinate
        packet.append(0xAF)  # Frame footer
        rospy.loginfo("Sending waypoint: %s", ' '.join(format(byte, '02x') for byte in packet))
        try:
            self.serial_port.write(packet)
        except Exception as e:
            rospy.logerr("Failed to write to serial port: %s", str(e))     

    # 0x02:老虎  0x03:大象 0x04：猴子 0x05: 狼 0x06: 孔雀 
    def send_animnal(self, animal, x , y , number ):
        packet = bytearray()
        packet.append(0xAA)  # Frame header
        packet.append(animal + 0x10)  # Function code (custom)
        packet.append(0x03)  # Data length (4 bytes for animal data)
        packet.append(x)  # x coordinate
        packet.append(y)  # y coordinate
        packet.append(number)  # number of animals
        packet.append(0xAF)  # Frame footer
        rospy.loginfo("Sending animal data: %s", ' '.join(format(byte, '02x') for byte in packet))
        try:
            self.serial_port.write(packet)
        except Exception as e:
            rospy.logerr("Failed to write to serial port: %s", str(e))       

    def send_animal_laser_error(self,x_error , y_error):
        packet = bytearray()
        packet.append(0xAA)  # Frame header
        packet.append(0x17)  # Function code (custom)
        packet.append(0x02)  # Data length (2 bytes for error data)
        packet.append(x_error)  # x coordinate error
        packet.append(y_error)  # y coordinate error
        packet.append(0xAF)  # Frame footer
        rospy.loginfo("Sending animal laser error: %s", ' '.join(format(byte, '02x') for byte in packet))
        try:
            self.serial_port.write(packet)
        except Exception as e:
            rospy.logerr("Failed to write to serial port: %s", str(e))                             

            
    def camera_en(self):
        self.camera_en_pub.publish(Bool(True))
        rospy.loginfo("camera_en")

    def fly_task_callback(self, data):
        self.fly_task = data.data

    def yaw_symbol_callback(self, data):
        self.yaw_symbol = data.data
        rospy.logerr("yaw_symbol: %d", self.yaw_symbol )


    def true_pos_callback(self, msg):
        # 提取位置
        position = msg.pose.position
        self.x = position.x
        self.y = position.y
        self.z = position.z

        # 提取偏航角
        orientation = msg.pose.orientation
        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.yaw = yaw

        if not self.waypointController._full_path:
            return
        
        if self._shoot_idx < len(self.waypointController._full_path):
            mx, my = self.waypointController._full_path[self._shoot_idx]
            ax, ay = self.coordinateConverter.map_to_aircraft((mx, my))
            ax = ax * 0.5 - 0.5
            ay = ay * 0.5 - 0.5
            d = math.sqrt((ax - self.x) ** 2 + (ay - self.y) ** 2)
            if d < 0.15:
                self.camera_en()
                self._shoot_idx += 1

            


class CoordinateConverter:
    def map_to_aircraft(self, map_coords: tuple) -> tuple:
        mx, my = map_coords
        if not (1 <= mx <= 9 and 1 <= my <= 7):
            raise ValueError(f"输入的地图坐标 {map_coords} 超出范围 (x: 1-9, y: 1-7)")
        ax = my
        ay = 10 - mx
        return (ax, ay)

    def aircraft_to_map(self, aircraft_coords: tuple) -> tuple:
        ax, ay = aircraft_coords
        if not (1 <= ax <= 7 and 1 <= ay <= 9):
            raise ValueError(f"输入的飞机坐标 {aircraft_coords} 超出范围 (x: 1-7, y: 1-9)")
        my = ax
        mx = 10 - ay

        return (mx, my)


class WaypointController:
    def __init__(self):
        self.width = 9
        self.height = 7
        self.start_pos = (9, 1)
        self.no_fly_zone = set()

        # 内部状态
        self._full_path = []
        self._current_step_index = 0
        self._visited_for_recognition = set()
        self._easy_path = []

        self.dd = True

    # ---------- 原有工具 ----------
    def _is_no_fly_zone_horizontal(self, no_fly_coords: list) -> bool:
        first_y = no_fly_coords[0][1]
        if all(y == first_y for _, y in no_fly_coords):
            return True
        first_x = no_fly_coords[0][0]
        if all(x == first_x for x, _ in no_fly_coords):
            return False
        raise ValueError("禁飞区坐标不构成连续的直线")

    def _find_path_between(self, start_pos: tuple, end_pos: tuple) -> list:
        queue = collections.deque([[start_pos]])
        seen = {start_pos}
        while queue:
            path = queue.popleft()
            x, y = path[-1]
            if (x, y) == end_pos:
                return path
            for dx, dy in [(0, 1), (0, -1), (-1, 0), (1, 0)]:
                next_x, next_y = x + dx, y + dy
                if 1 <= next_x <= self.width and 1 <= next_y <= self.height and \
                        (next_x, next_y) not in self.no_fly_zone and \
                        (next_x, next_y) not in seen:
                    seen.add((next_x, next_y))
                    new_path = list(path)
                    new_path.append((next_x, next_y))
                    queue.append(new_path)
        return None

    # ---------- 任务规划 ----------
    def plan_mission(self, no_fly_map_coords: list):
        self.no_fly_zone = set(no_fly_map_coords)
        is_horizontal = self._is_no_fly_zone_horizontal(no_fly_map_coords)

        # 生成覆盖航点
        waypoints = []
        if is_horizontal:
            for y in range(1, self.height + 1):
                x_range = range(self.width, 0, -1) if y % 2 else range(1, self.width + 1)
                for x in x_range:
                    if (x, y) not in self.no_fly_zone:
                        waypoints.append((x, y))
        else:
            for x in range(self.width, 0, -1):
                y_range = range(1, self.height + 1) if x % 2 else range(self.height, 0, -1)
                for y in y_range:
                    if (x, y) not in self.no_fly_zone:
                        waypoints.append((x, y))

        # 生成覆盖路径
        path = [self.start_pos]
        current_pos = self.start_pos
        if self.start_pos in waypoints:
            waypoints.remove(self.start_pos)

        for target in waypoints:
            segment = self._find_path_between(current_pos, target)
            if segment:
                path.extend(segment[1:])
                current_pos = target
            else:
                print(f"警告: 无法从 {current_pos} 到达目标点 {target}。")

        # 新增：从最后一个航点直线回到起飞点
        if path:
            last_wp = path[-1]
            home_path = self._find_path_between(last_wp, self.start_pos)
            if home_path:
                path.extend(home_path[1:-1])

        self._full_path = path
        if self.dd:
            self._full_path = [
    (9, 1), (8, 1), (7, 1), (6, 1), (5, 1), (4, 1), (3, 1), (2, 1), (1, 1),
    (1, 2), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2), (7, 2), (8, 2), (9, 2),
    (9, 3), (8, 3), (7, 3), (6, 3), (5, 3), (4, 3), (3, 3), (2, 3), (1, 3),
    (1, 4), (2, 4), (3, 4), (4, 4), (5, 4), (6, 4), (7, 4), (8, 4), (9, 4),
    (9, 5), (8, 5), (7, 5), (6, 5), (5, 5), (4, 5), (3, 5), (2, 5), (1, 5),
    (1, 6), (2, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (8, 6), (9, 6),
    (9, 7), (8, 7), (7, 7), (6, 7), (5, 7), (4, 7), (3, 7), (2, 7), (1, 7), (2, 7), (3, 7), (4, 7), (5, 7),
    (6, 7), (7, 7), (8, 7), (9, 7),
    (9, 6), (9, 5), (9, 4), (9, 3), (9, 2), (9, 1)
]
        self._current_step_index = 0
        self._visited_for_recognition = set()
        self.easy_path = self.compress_path(self._full_path)
        print(f"Complete : {len(self._full_path)}。")

    # ---------- 获取下一个航点 ----------
    def get_next_waypoint(self) -> tuple:
        if self._current_step_index >= len(self._full_path):
            print("所有航点已飞完，任务结束。")
            return None, None
        coord = self._full_path[self._current_step_index]
        is_new = coord not in self._visited_for_recognition
        if is_new:
            self._visited_for_recognition.add(coord)
        self._current_step_index += 1
        return coord, is_new
    
    @staticmethod
    def compress_path(path: list) -> list:
        """
        把一条由相邻格点组成的路径压缩成只包含起点、拐弯点和终点的列表。
        """
        if len(path) < 3:
            return path

        compressed = [path[0]]
        prev_dx = path[1][0] - path[0][0]
        prev_dy = path[1][1] - path[0][1]

        for i in range(2, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            # 方向改变 -> 前一个点就是拐弯点
            if (dx, dy) != (prev_dx, prev_dy):
                compressed.append(path[i - 1])
                prev_dx, prev_dy = dx, dy

        compressed.append(path[-1])
        return compressed


        
    



if __name__ == '__main__':
    try:
        node = IMUSerialNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
