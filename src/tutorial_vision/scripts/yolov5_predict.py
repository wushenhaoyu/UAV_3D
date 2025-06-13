#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
from tutorial_vision.msg import StringStamped
import torch
import torch.nn.functional as F


ROOT = ''
if __name__ == '__main__':
    rospy.init_node('yolov5_predict_node', sys.argv)
    ROOT = rospy.get_param('~yolov5_root')
else:
    FILE = Path(__file__).resolve()
    ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))


from models.common import DetectMultiBackend
from utils.augmentations import classify_transforms
from utils.general import LOGGER, check_requirements, colorstr, increment_path, print_args, set_logging
from utils.torch_utils import select_device, smart_inference_mode, time_sync


@smart_inference_mode()
def detect(
        weights, #weightpath
        source,  # image
        imgsz=224,  # inference size
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        confidence=0.85  # Confidence Threshold
):
    file = str(source)
    seen, dt = 1, [0.0, 0.0, 0.0]
    device = select_device(device)
    
    # Transforms
    transforms = classify_transforms(imgsz)

    # Load model
    model = DetectMultiBackend(weights, device=device, dnn=dnn, fp16=half)
    model.warmup(imgsz=(1, 3, imgsz, imgsz))  # warmup

    # Image
    t1 = time_sync()
    im = cv2.cvtColor(source, cv2.COLOR_BGR2RGB)
    im = transforms(im).unsqueeze(0).to(device)
    im = im.half() if model.fp16 else im.float()
    t2 = time_sync()
    dt[0] += t2 - t1

    # Inference
    results = model(im)
    t3 = time_sync()
    dt[1] += t3 - t2

    p = F.softmax(results, dim=1)  # probabilities
    i = p.argsort(1, descending=True)[:, :5].squeeze()  # top 5 indices
    
    return model.names[i[0]] if torch.max(p) > confidence else ''


def image_cb(msg, bridge, weights, device, pub, confidence):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    name = detect(weights=weights, source=cv_image, device=device, confidence=confidence)
    if name != '':
        result = StringStamped()
        result.header = msg.header
        result.data.append(name)
        pub.publish(result)


def main():
    set_logging(verbose=False)
    weights = rospy.get_param('~weights_path')
    device = rospy.get_param('~device', '')
    confidence = rospy.get_param('~confidence', 0.85)
    bridge = CvBridge()
    str_pub = rospy.Publisher('yolo_detect', StringStamped, queue_size=1)
    rospy.Subscriber('camera/image_raw', Image,
        lambda msg: image_cb(msg, bridge, weights, device, str_pub, confidence)
    )
    rospy.spin()


if __name__ == '__main__':
    main()

