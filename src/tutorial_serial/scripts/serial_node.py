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
from std_msgs.msg import Bool
from tutorial_serial.msg import SerialData
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
        self.serial_ctrl_sub = rospy.Subscriber("serial_ctrl", SerialData, self.serial_ctrl_callback)
        self.camera_en_sub   = rospy.Subscriber("camera_en", Bool, self.camera_en_callback)
        self.camera_en  = False

        # TF2 相关
        self.tf_broadcaster = StaticTransformBroadcaster()
        rospy.Timer(rospy.Duration(0.001), self.read_serial)

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
        self.serial_port.write(packet)

    def camera_en_callback(self,data):
        self.camera_en = data.data
        #rospy.loginfo("camera_en: %s", self.camera_en)

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
        rospy.logerr("yaw_symbol: %d", self.yaw_symbol )

    """def qr_callback(self, msg):
        try:
            if(msg.data):
                if self.fly_task == 1:
                    if self.camera_en == True:
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
            rospy.logerr("send qr failed: %s", str(e))"""

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

        
    



if __name__ == '__main__':
    try:
        node = IMUSerialNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
