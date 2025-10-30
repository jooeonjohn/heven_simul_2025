#! /usr/bin/env python3
import rospy
import sys
import numpy as np
import argparse
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
sys.path.append('/home/heven/erp_ws/src/heven_m_m/yolopv2')
# Conclude setting / general reprocessing / plots / metrices / datasets
from utils.utils import \
    time_synchronized,select_device, increment_path,\
    scale_coords,xyxy2xywh,non_max_suppression,split_for_trace_model,\
    driving_area_mask,lane_line_mask,plot_one_box,show_seg_result, letterbox,\
    AverageMeter,\
    LoadImages
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

import torch_tensorrt

# def make_parser():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--weights', nargs='+', type=str, default='data/weights/yolopv2.pt', help='model.pt path(s)')
#     parser.add_argument('--source', type=str, default='data/3_test_video.mp4', help='source')  # file/folder, 0 for webcam
#     parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
#     parser.add_argument('--conf-thres', type=float, default=0.3, help='object confidence threshold')
#     parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
#     parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
#     parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
#     parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
#     parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
#     parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
#     parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
#     parser.add_argument('--project', default='runs/detect', help='save results to project/name')
#     parser.add_argument('--name', default='exp', help='save results to project/name')
#     parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
#     return parser

# @torch.no_grad()
class LaneDetector():
    def __init__(self):
        rospy.init_node('LaneDetector_node', anonymous=False)
        self.bridge = CvBridge()
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.device = rospy.get_param("~device")
        self.image_size = rospy.get_param("~img-size")
        self.input_topic = rospy.get_param("~input_image_topic")
        self.init_flag = 0
        self.count = 1
        self.inf_time = AverageMeter()
        self.waste_time = AverageMeter()
        self.nms_time = AverageMeter()
        self.weights = rospy.get_param("~weights")
        with torch.no_grad():
            self.cuda_device = select_device(str(self.device))
            self.half = self.cuda_device.type != 'cpu'
            self.model = torch.jit.load(self.weights)
            self.model = self.model.to(self.cuda_device)
            if self.half:
                self.model.half()
            self.model.eval()

            self.model_trt = torch_tensorrt.compile(
                self.model,
                inputs=[torch_tensorrt.Input(
                    min_shape=[1, 3, self.image_size, self.image_size],
                    opt_shape=[1, 3, self.image_size, self.image_size],
                    max_shape=[1, 3, self.image_size, self.image_size],
                    dtype=torch.half if self.half else torch.float
                )],
                enabled_precisions={torch.half} if self.half else {torch.float},
                workspace_size=1 << 20
            )

            cudnn.benchmark = True
            image = torch.empty((1, 3, 640, 480), dtype=torch.half, device=self.device)
            [pred, anchor_grid], seg, ll = self.model(image)
        
        self.image_sub = rospy.Subscriber(self.input_topic, CompressedImage, self.camera_callback)
        # self.image_sub = rospy.Subscriber(self.input_topic, Image, self.camera_callback, queue_size=1)
        self.pub = rospy.Publisher('/usb_cam/lane_output', Image, queue_size=10)

    def camera_callback(self, data):
        # print(self.cuda_device)
        self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        # self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # self.background_image = cv2.resize(self.image, (1280)
        # cv2.imshow("Original", self.image)
        # cv2.waitKey(1)
        with torch.no_grad():
            self.model(torch.zeros(1, 3, self.image_size, self.image_size).to(self.cuda_device).type_as(next(self.model.parameters())))
            self.background_image = cv2.resize(self.image, (640,480), interpolation=cv2.INTER_LINEAR)
            self.im = letterbox(self.image, 640, stride=32)[0]
            self.im = self.im[:, :, ::-1].transpose(2, 0, 1)
            self.im = np.ascontiguousarray(self.im)
            # with torch.no_grad():
            self.im = torch.from_numpy(self.im).to(self.cuda_device)
            self.im = self.im.half()
            self.im /= 255.0
            if self.im.ndimension() == 3:
                self.im = self.im.unsqueeze(0)
            self.detect()
            torch.cuda.empty_cache()

    def detect(self):
        with torch.no_grad():
            t1 = time_synchronized()
            [pred, anchor_grid], seg, ll = self.model_trt(self.im)
            t2 = time_synchronized()
            tw1 = time_synchronized()
            pred = split_for_trace_model(pred, anchor_grid)
            tw2 = time_synchronized()
            t3 = time_synchronized()
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=None, agnostic=False)
            t4 = time_synchronized()
            da_seg_mask = driving_area_mask(seg).astype(np.uint8)
            ll_seg_mask = lane_line_mask(ll).astype(np.uint8) 
        
        result_seg_img = np.dstack((ll_seg_mask,ll_seg_mask,ll_seg_mask)) * 255
        cv2.imshow('result', result_seg_img)
        # cv2.imwrite('/home/heven/Downloads/testimage'+str(self.count)+'.png', ll_seg_mask)
        # self.count += 1
        print("ll_seg_mask shape : ", ll_seg_mask.shape)
        print("background shape : ", self.background_image.shape)

        result_img = show_seg_result(self.background_image, (da_seg_mask, ll_seg_mask), is_demo=True)
        # for i, det in enumerate(pred):
        #     if len(det):
        #         det[:, :4] = scale_coords(self.im.shape[2:], det[:, :4], self.background_image.shape).round()
            
            # cv2.imshow("seg", ll_seg_mask)
            # result_img = show_seg_result(self.background_image, (da_seg_mask, ll_seg_mask), is_demo=True)
            # print(result_img)
            # result_img = show_seg_result(self.background_image, ll_seg_mask, is_demo=True)
        # self.inf_time.update(t2-t1,self.im.size(0))
        # self.nms_time.update(t4-t3,self.im.size(0))
        # self.waste_time.update(tw2-tw1,self.im.size(0))
        # print('inf : (%.4fs/frame)   nms : (%.4fs/frame)' % (self.inf_time.avg,self.nms_time.avg))
        cv2.imshow("Result", result_img)
        cv2.waitKey(1)
        result_msg = self.bridge.cv2_to_imgmsg(result_seg_img, encoding="bgr8")
        self.pub.publish(result_msg)

if __name__ == '__main__':
    if not rospy.is_shutdown():
        LaneDetector()
        rospy.spin()