#!/usr/bin/env python
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
class IMUSerialNode:
    def __init__(self):
        rospy.init_node('serial_node', log_level=rospy.INFO)
        #self.serial_port = serial.Serial('/dev/ttyserial', 9600, timeout=1)
        try:
            self.serial_port = serial.Serial('/dev/ttyserial', 9600, timeout=1)
        except Exception as e:
            rospy.logerr("Failed to open serial port")
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
        self.fly_target_pub = rospy.Publisher('fly_target', UInt8, queue_size=10)

        self.count = 0

        # 订阅二维码检测结果
        self.qr_sub = rospy.Subscriber("qr_detect_result", StringStamped, self.qr_callback)
        self.fly_task_sub = rospy.Subscriber("fly_task", Int32, self.fly_task_callback)
        self.yaw_symbol_sub = rospy.Subscriber("yaw_symbol", Int32, self.yaw_symbol_callback)
        self.true_pos_sub = rospy.Subscriber("true_position", PoseStamped, self.true_pos_callback)

        # TF2 相关
        self.tf_broadcaster = StaticTransformBroadcaster()
        rospy.Timer(rospy.Duration(0.001), self.read_serial)

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

    def deal_with_data(self, func, length, data):
        if(func == 0x01):
            self.target = data[0]
            target_msg = UInt8()
            target_msg.data = self.target
            self.fly_target_pub.publish(target_msg)

    def fly_task_callback(self, data):
        self.fly_task = data.data

    def yaw_symbol_callback(self, data):
        self.yaw_symbol = data.data

    def qr_callback(self, msg):
        try:
            if(msg.data):
                if self.fly_task == 1:
                    packet = bytearray()
                    packet.append(0xAA)  # 帧头
                    packet.append(0x01)  # 功能码（自定义）
                    
                    # 将 msg.data 中的字符串转换为对应的十进制数，再转换为十六进制数
                    qr_data = bytearray()
                    for char in msg.data:
                        try:
                            decimal_value = int(char)  # 将字符转换为对应的十进制数
                            qr_data.append(decimal_value)  # 将十进制数添加到 qr_data
                        except ValueError:
                            rospy.logwarn(f"Invalid decimal value: {char}, skipping this value.")
                    
                    data_length = len(qr_data) + 0x01
                    packet.append(data_length) 
                    packet.append(self.judge_location())
                    
                    packet.extend(qr_data) 
                    packet.append(0xAF)  # 帧尾
                    
                    rospy.loginfo("Sending packet: %s x:%.2f, y:%.2f, z:%.2f, yaw:%.2f",
                                ' '.join(format(byte, '02x') for byte in packet),
                                self.x, self.y, self.z, self.yaw)
                    self.serial_port.write(packet) 
                elif self.fly_task == 2:
                    packet = bytearray()
                    packet.append(0xAA)  # 帧头
                    packet.append(0x02)  # 功能码（自定义）
                    qr_data = bytearray()
                    for char in msg.data:
                        try:
                            decimal_value = int(char)  # 将字符转换为对应的十进制数
                            qr_data.append(decimal_value)  # 将十进制数添加到 qr_data
                        except ValueError:
                            rospy.logwarn(f"Invalid decimal value: {char}, skipping this value.")
                    
                    data_length = len(qr_data)
                    packet.append(data_length)
                    packet.extend(qr_data)
                    packet.append(0xAF)  # 帧尾
                    rospy.loginfo("Sending packet: %s", ' '.join(format(byte, '02x') for byte in packet))
                    self.serial_port.write(packet) 

        except Exception as e:
            rospy.logerr("send qr failed: %s", str(e))

    def true_pos_callback(self, msg):
        self.count = self.count + 1
        if(self.count != 10):
            return
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

        
        packet = bytearray()
        packet.append(0xAA)  # 帧头
        packet.append(0x03)  # 功能码（自定义）
        packet.append(0x08)

# 将 self.x 和 self.y 各自乘以 100 并转为整数
        x_int = int(self.x * 100)
        y_int = int(self.y * 100)
       # rospy.loginfo("x_int: %d, y_int: %d", x_int, y_int)
# 使用 struct 将整数打包为字节数据
        packet.extend(struct.pack('>ii', x_int, y_int))  # 修改这里
        packet.append(0xAF)

        

        #rospy.loginfo("Sending Location packet: %s", ' '.join(format(byte, '02x') for byte in packet))
        self.serial_port.write(packet) 
        self.count = 0
            #rospy.logerr("send location failed: %s", str(e))

    def judge_location(self):
        if(self.x > -0.5 and self.x < 1.0): #在1～6
            if(self.z > 1.2 and self.z <1.6):
                if(self.y >0.5 and self.y < 1.0):
                    return 0x03
                elif(self.y >1.0 and self.y < 1.5):
                    return 0x02
                elif(self.y >1.5 and self.y < 2.0):
                    return 0x01
            elif(self.z > 0.8 and self.z <1.2):
                if(self.y >0.5 and self.y < 1.0):
                    return 0x06
                elif(self.y >1.0 and self.y < 1.5):
                    return 0x05
                elif(self.y >1.5 and self.y < 2.0):
                    return 0x04   
        elif(self.x >1.0 and self.x < 2.5): #在7～18
            if(self.yaw_symbol == 0 ):#在13～18
                if(self.z > 1.2 and self.z <1.6):
                    if(self.y >0.5 and self.y < 1.0):
                        return 0x0D
                    elif(self.y >1.0 and self.y < 1.5):
                        return 0x0C
                    elif(self.y >1.5 and self.y < 2.0):
                        return 0x0B
                elif(self.z > 0.8 and self.z <1.2):
                    if(self.y >0.5 and self.y < 1.0):
                        return 0x12
                    elif(self.y >1.0 and self.y < 1.5):
                        return 0x11
                    elif(self.y >1.5 and self.y < 2.0):
                        return 0x10  
            elif(self.yaw_symbol == 1):#在7～12
                if(self.z > 1.2 and self.z <1.6):
                    if(self.y >0.5 and self.y < 1.0):
                        return 0x07
                    elif(self.y >1.0 and self.y < 1.5):
                        return 0x08
                    elif(self.y >1.5 and self.y < 2.0):
                        return 0x09
                elif(self.z > 0.8 and self.z <1.2):
                    if(self.y >0.5 and self.y < 1.0):
                        return 0x0A
                    elif(self.y >1.0 and self.y < 1.5):
                        return 0x0B
                    elif(self.y >1.5 and self.y < 2.0):
                        return 0x0C 
        elif(self.x >2.5 and self.x < 4.0): #在19～24
            if(self.z > 1.2 and self.z <1.6):
                if(self.y >0.5 and self.y < 1.0):
                    return 0x13
                elif(self.y >1.0 and self.y < 1.5):
                    return 0x14
                elif(self.y >1.5 and self.y < 2.0):
                    return 0x15
            elif(self.z > 0.8 and self.z <1.2):
                if(self.y >0.5 and self.y < 1.0):
                    return 0x16
                elif(self.y >1.0 and self.y < 1.5):
                    return 0x17
                elif(self.y >1.5 and self.y < 2.0):
                    return 0x18 
        return 0x17


if __name__ == '__main__':
    try:
        node = IMUSerialNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
