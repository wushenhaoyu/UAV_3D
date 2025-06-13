#!/usr/bin/env python
import rospy
import serial
import struct
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf2_ros
import rosbag

class IMUSerialNode:
    def __init__(self):
        rospy.init_node('serial_node', log_level=rospy.INFO)
        self.buffer = bytearray()
        self.receiving = False
        self.length = 0
        self.func = 0
        self.current = 0

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
        rospy.loginfo(data)
        pass



if __name__ == '__main__':
    try:
        node = IMUSerialNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass