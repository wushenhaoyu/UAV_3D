#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

# ---------- 全局变量 ----------
capture     = None
bridge      = None
image_pub   = None
frame_id    = None
save_idx    = 0        # 可选：避免文件名覆盖
save_dir    = "/tmp"   # 可选：保存目录

# ---------- 回调 ----------
def camera_en_cb(msg):
    global capture, bridge, image_pub, frame_id, save_idx

    if not msg.data:          # 只处理 True
        return

    ret, frame = capture.read()
    if not ret:
        rospy.logwarn("Failed to grab frame from camera")
        return

    # 保存图片（带序号，防止覆盖）
    #save_path = "{}/img_{:04d}.jpg".format(save_dir, save_idx)
    #cv2.imwrite(save_path, frame)
    #save_idx += 1
    #rospy.loginfo("Saved %s", save_path)

    # 转成 ROS 消息并发布
    try:
        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        img_msg.header.stamp    = rospy.Time.now()
        img_msg.header.frame_id = frame_id
        image_pub.publish(img_msg)
    except CvBridgeError as e:
        rospy.logerr(e)

# ---------- 主函数 ----------
def main(argv):
    global capture, bridge, image_pub, frame_id

    rospy.init_node("simple_camera_driver", argv=argv)

    camera_id   = rospy.get_param("~camera_id", 0)
    frame_id    = rospy.get_param("~frame_id", "mono_camera")
    save_dir    = rospy.get_param("~save_dir", "/tmp")

    capture   = cv2.VideoCapture(camera_id)
    bridge    = CvBridge()
    image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)
    rospy.Subscriber("camera_en", Bool, camera_en_cb)

    if not capture.isOpened():
        rospy.logfatal("Cannot open camera %d", camera_id)
        return

    rospy.loginfo("Ready. Waiting for /camera_en = True ...")
    rospy.spin()      # 保持节点运行，等待回调

if __name__ == '__main__':
    import sys
    main(sys.argv)