#!/usr/bin/env python3
#
# Copyright 1993-2019 NVIDIA Corporation.  All rights reserved.
#
# NOTICE TO LICENSEE:
#
# This source code and/or documentation ("Licensed Deliverables") are
# subject to NVIDIA intellectual property rights under U.S. and
# international Copyright laws.
#
# These Licensed Deliverables contained herein is PROPRIETARY and
# CONFIDENTIAL to NVIDIA and is being provided under the terms and
# conditions of a form of NVIDIA software license agreement by and
# between NVIDIA and Licensee ("License Agreement") or electronically
# accepted by Licensee.  Notwithstanding any terms or conditions to
# the contrary in the License Agreement, reproduction or disclosure
# of the Licensed Deliverables to any third party without the express
# written consent of NVIDIA is prohibited.
#
# NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
# LICENSE AGREEMENT, NVIDIA MAKES NO REPRESENTATION ABOUT THE
# SUITABILITY OF THESE LICENSED DELIVERABLES FOR ANY PURPOSE.  IT IS
# PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY OF ANY KIND.
# NVIDIA DISCLAIMS ALL WARRANTIES WITH REGARD TO THESE LICENSED
# DELIVERABLES, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
# NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
# NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
# LICENSE AGREEMENT, IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY
# SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
# DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
# WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
# ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
# OF THESE LICENSED DELIVERABLES.
#
# U.S. Government End Users.  These Licensed Deliverables are a
# "commercial item" as that term is defined at 48 C.F.R. 2.101 (OCT
# 1995), consisting of "commercial computer software" and "commercial
# computer software documentation" as such terms are used in 48
# C.F.R. 12.212 (SEPT 1995) and is provided to the U.S. Government
# only as a commercial end item.  Consistent with 48 C.F.R.12.212 and
# 48 C.F.R. 227.7202-1 through 227.7202-4 (JUNE 1995), all
# U.S. Government End Users acquire the Licensed Deliverables with
# only those rights set forth herein.
#
# Any use of the Licensed Deliverables in individual and commercial
# software must include, in the user documentation and internal
# comments to the code, the above Disclaimer and U.S. Government End
# Users Notice.
#

import sys, os
import time
import numpy as np
import math
import cv2
# from cv_bridge import CvBridge  # If you use Python2

import rospy
from sensor_msgs.msg import Image as Imageros
from yolov3_trt_ros.msg import BoundingBox, BoundingBoxes

from data_processing import PreprocessYOLO, PostprocessYOLO, ALL_CATEGORIES

import tensorrt as trt
import common
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
MODEL = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..",
    "model/model_epoch4400_pretrained.trt"
    )

CFG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "config/yolov3-tiny_tstl_416.cfg")
NUM_CLASS = 8

xycar_image = np.empty(shape=[0])

# global CAMERA_MATRIX, DISTORT_COEFF, HOMOGRAPHY
CAMERA_MATRIX = np.array([[352.494189, 0.000000, 295.823760],
                          [0.000000, 353.504572, 239.649689],
                          [0.000000, 0.000000, 1.000000]])
DISTORT_COEFF = np.array([-0.318744, 0.088199, 0.000167, 0.000699, 0.000000])
HOMOGRAPHY = [[-1.91653414e-01, -1.35359667e+00, 3.09165939e+02],
              [8.42075718e-04, -2.87835672e+00, 6.16140688e+02],
              [9.75538785e-06, -5.04169177e-03, 1.00000000e+00]]

class yolov3_trt(object):
    def __init__(self):
        self.cfg_file_path = CFG
        self.num_class = NUM_CLASS
        width, height, masks, anchors = parse_cfg_wh(self.cfg_file_path)
        self.model_path = MODEL
        # Two-dimensional tuple with the target network's (spatial) input resolution in HW ordered
        input_resolution_yolov3_WH = (width, height)
        # Create a pre-processor object by specifying the required input resolution for YOLOv3
        self.preprocessor = PreprocessYOLO(input_resolution_yolov3_WH)

        # Output shapes expected by the post-processor
        output_channels = (self.num_class + 5) * 3
        if len(masks) == 2:
            self.output_shapes = [(1, output_channels, height//32, width//32), (1, output_channels, height//16, width//16)]
        else:
            self.output_shapes = [(1, output_channels, height//32, width//32), (1, output_channels, height//16, width//16), (1, output_channels, height//8, width//8)]

        postprocessor_args = {"yolo_masks": masks,                    # A list of 3 three-dimensional tuples for the YOLO masks
                              "yolo_anchors": anchors,
                              "obj_threshold": 0.5,                                               # Threshold for object coverage, float value between 0 and 1
                              "nms_threshold": 0.3,                                               # Threshold for non-max suppression algorithm, float value between 0 and 1
                              "yolo_input_resolution": input_resolution_yolov3_WH,
                              "num_class": self.num_class}

        self.postprocessor = PostprocessYOLO(**postprocessor_args)

        self.engine = get_engine(self.model_path)
        self.context = self.engine.create_execution_context()

        self.detection_pub = rospy.Publisher('/yolov3_trt_ros/detections', BoundingBoxes, queue_size=1)


    def detect(self):
        rate = rospy.Rate(30)
        image_sub = rospy.Subscriber("/usb_cam/image_raw", Imageros, img_callback)

        while not rospy.is_shutdown():
            # If xycar_image is empty, skip inference
            if xycar_image.shape[0] == 0:
                continue

            image = self.preprocessor.process(xycar_image)
            # Store the shape of the original input image in WH format, we will need it for later
            shape_orig_WH = (image.shape[3], image.shape[2])

            start_time = time.time()
            # Do inference with TensorRT
            inputs, outputs, bindings, stream = common.allocate_buffers(self.engine)

            # Set host input to the image. The common.do_inference function will copy the input to the GPU before executing.
            inputs[0].host = image
            trt_outputs = common.do_inference(self.context, bindings=bindings, inputs=inputs, outputs=outputs, stream=stream)

            # Before doing post-processing, we need to reshape the outputs as the common.do_inference will give us flat arrays.
            trt_outputs = [output.reshape(shape) for output, shape in zip(trt_outputs, self.output_shapes)]

            # Run the post-processing algorithms on the TensorRT outputs and get the bounding box details of detected objects
            boxes, classes, scores = self.postprocessor.process(trt_outputs, shape_orig_WH)
            latency = time.time() - start_time
            fps = 1 / latency
            # print(fps)

            xdepth_list = []
            ydepth_list = []
            if boxes is not None and not classes is None:
                for box, class_id in zip(boxes, classes):
                    xmin, ymin, width, height = box
                    obj_bottom_center = np.array([xmin + width/2, ymin + height, 1])
                    grid_point = np.dot(HOMOGRAPHY, obj_bottom_center)
                    grid_point /= grid_point[2] + 0.000001
                    grid_point = np.round(grid_point).astype(int)

                    x, y = grid_point[0], grid_point[1]
                    xdepth = abs(270 - x) / 2
                    xdepth_list.append(xdepth)
                    ydepth = (540 - y) / 2
                    ydepth_list.append(ydepth)

            # Publish detected objects boxes and classes
            self.publisher(boxes, scores, classes, xdepth_list, ydepth_list)

            rate.sleep()


    def _write_message(self, detection_results, boxes, scores, classes, xdepth_list, ydepth_list):
        """ populate output message with input header and bounding boxes information """
        if boxes is None:
            return None
        for box, score, category, xdepth, ydepth in zip(boxes, scores, classes, xdepth_list, ydepth_list):
            # Populate darknet message
            xmin, ymin, width, height = box
            detection_msg = BoundingBox()
            detection_msg.xmin = int(xmin)
            detection_msg.xmax = int(xmin + width)
            detection_msg.ymin = int(ymin)
            detection_msg.ymax = int(ymin + height)
            detection_msg.prob = float(score)
            detection_msg.id = int(category)
            detection_msg.xdepth = int(xdepth)
            detection_msg.ydepth = int(ydepth)
            detection_results.bbox.append(detection_msg)
        return detection_results

    def publisher(self, boxes, confs, classes, xdepth_list, ydepth_list):
        """ Publishes to detector_msgs
        Parameters:
        boxes (List(List(int))) : Bounding boxes of all objects
        confs (List(double))	: Probability scores of all objects
        classes  (List(int))	: Class ID of all classes
        """
        detection_results = BoundingBoxes()
        self._write_message(detection_results, boxes, confs, classes, xdepth_list, ydepth_list)
        self.detection_pub.publish(detection_results)


# Parse width, height, masks and anchors from cfg file
def parse_cfg_wh(cfg):
    masks = []
    with open(cfg, 'r') as f:
        lines = f.readlines()
        for line in lines:
            if 'width' in line:
                w = int(line[line.find('=')+1:].replace('\n',''))
            elif 'height' in line:
                h = int(line[line.find('=')+1:].replace('\n',''))
            elif 'anchors' in line:
                anchor = line.split('=')[1].replace('\n','')
                anc = [int(a) for a in anchor.split(',')]
                anchors = [(anc[i*2], anc[i*2+1]) for i in range(len(anc) // 2)]
            elif 'mask' in line:
                mask = line.split('=')[1].replace('\n','')
                m = tuple(int(a) for a in mask.split(','))
                masks.append(m)
    return w, h, masks, anchors

def img_callback(data):
    global xycar_image

    # Python 2
    # xycar_image = CvBridge().imgmsg_to_cv2(data, "bgr8")

    # Python 3
    # xycar_image == RGB
    test_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    mapx, mapy = cv2.initUndistortRectifyMap(CAMERA_MATRIX, DISTORT_COEFF, None, None, (test_image.shape[1], test_image.shape[0]), 5)
    xycar_image = cv2.remap(test_image, mapx, mapy, cv2.INTER_LINEAR)

def get_engine(model_path=""):
    """Attempts to load a serialized engine if available, otherwise builds a new TensorRT engine and saves it."""
    if os.path.exists(model_path):
        # If a serialized engine exists, use it instead of building an engine.
        print("Reading engine from file {}".format(model_path))
        with open(model_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            return runtime.deserialize_cuda_engine(f.read())
    else:
        print("no trt model")
        sys.exit(1)

if __name__ == '__main__':
    yolo = yolov3_trt()
    rospy.init_node('yolov3_trt_ros', anonymous=True)
    yolo.detect()