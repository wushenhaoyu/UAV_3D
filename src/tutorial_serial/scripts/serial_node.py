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
from std_msgs.msg import UInt8
class IMUSerialNode:
    def __init__(self):
        rospy.init_node('serial_node', log_level=rospy.INFO)
        self.serial_port = serial.Serial('/dev/imu', 115200, timeout=1)
        self.buffer = bytearray()
        self.receiving = False
        self.length = 0
        self.func = 0
        self.current = 0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.target = 0
        self.fly_target_pub = rospy.Publisher('fly_target', UInt8, queue_size=10)

        # 订阅二维码检测结果
        self.qr_sub = rospy.Subscriber("qr_detect_result", StringStamped, self.qr_callback)

        # TF2 相关
        self.tf_broadcaster = StaticTransformBroadcaster()
        rospy.Timer(rospy.Duration(0.001), self.read_serial)

    def read_serial(self, event):
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

    def deal_with_data(self, func, length, data):
        if(func == 0x01):
            self.target = data[0]
            target_msg = UInt8()
            target_msg.data = self.target
            self.fly_target_pub.publish(target_msg)


    def qr_callback(self, msg):
        try:
            packet = bytearray()
            packet.append(0xAA)  # 帧头
            packet.append(0x01)  # 功能码（自定义）
            
            qr_data = ''.join(msg.data).encode('utf-8')
            data_length = len(qr_data) + 0x01
            packet.append(data_length) 
            packet.append(self.judge_location())
            
            packet.extend(qr_data) 
            packet.append(0xAF)  
            
            self.serial_port.write(packet)
            rospy.loginfo("发送二维码数据: %s", qr_data)
            
        except Exception as e:
            rospy.logerr("串口发送失败: %s", str(e))

    def vision_pose_callback(self, msg):
        position = msg.pose.position
        self.x = position.x
        self.y = position.y
        self.z = position.z
        orientation = msg.pose.orientation
        quaternion = [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.yaw = yaw
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
            if(self.yaw > math.PI / 30 and self.yaw < math.PI / 30):#在13～18
                if(self.z > 1.2 and self.z <1.6):
                    if(self.y >0.5 and self.y < 1.0):
                        return 0x15
                    elif(self.y >1.0 and self.y < 1.5):
                        return 0x14
                    elif(self.y >1.5 and self.y < 2.0):
                        return 0x13
                elif(self.z > 0.8 and self.z <1.2):
                    if(self.y >0.5 and self.y < 1.0):
                        return 0x18
                    elif(self.y >1.0 and self.y < 1.5):
                        return 0x17
                    elif(self.y >1.5 and self.y < 2.0):
                        return 0x16  
            elif(abs(self.yaw - math.pi) < math.pi / 30):#在7～12
                if(self.z > 1.2 and self.z <1.6):
                    if(self.y >0.5 and self.y < 1.0):
                        return 0x09
                    elif(self.y >1.0 and self.y < 1.5):
                        return 0x08
                    elif(self.y >1.5 and self.y < 2.0):
                        return 0x07
                elif(self.z > 0.8 and self.z <1.2):
                    if(self.y >0.5 and self.y < 1.0):
                        return 0x12
                    elif(self.y >1.0 and self.y < 1.5):
                        return 0x11
                    elif(self.y >1.5 and self.y < 2.0):
                        return 0x10 
        elif(self.x >2.5 and self.x < 4.0): #在19～24
            if(self.z > 1.2 and self.z <1.6):
                if(self.y >0.5 and self.y < 1.0):
                    return 0x21
                elif(self.y >1.0 and self.y < 1.5):
                    return 0x20
                elif(self.y >1.5 and self.y < 2.0):
                    return 0x19
            elif(self.z > 0.8 and self.z <1.2):
                if(self.y >0.5 and self.y < 1.0):
                    return 0x24
                elif(self.y >1.0 and self.y < 1.5):
                    return 0x23
                elif(self.y >1.5 and self.y < 2.0):
                    return 0x21 
        else:
            return 0x00



if __name__ == '__main__':
    try:
        node = IMUSerialNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass