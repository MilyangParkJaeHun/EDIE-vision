#!/usr/bin/env python3
"""
    edie_detector_node.py
    Author: Park Jaehun
    Purpose
        Object Detection Vision System for EDIE        
"""
import os
import cv2
from argparse import ArgumentParser

import rospy
import rospkg
import sys
from edie_msgs.msg import EdieView

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('edie_detector')
sys.path.append(os.path.join(pkg_path, 'scripts'))

from openvino_detector.DetModel import OpenvinoDet
from openvino_detector.Model.Yolo import Yolo
from openvino_detector.Model.Ssd import Ssd
from utils import CameraState, ZoomCamera, AsyncZoomCamera, ObjectState
 
class EdieDetector():
    def __init__(self):
        args = self.build_argparser().parse_args()

        if args.sync:
            cap = ZoomCamera(args.input_stream)
        else:
            cap = AsyncZoomCamera(args.input_stream)
        self.cap = cap

        self.init_variable(args)

        if args.yolo:
            model_parser = Yolo()
        if args.ssd:
            model_parser = Ssd()

        self.detector = OpenvinoDet(model_parser=model_parser, \
                        model_path=args.model_path, \
                        device=args.device, \
                        label_map=self.label_map_, \
                        prob_threshold=args.prob_threshold)

        self.display_flag = args.display
    
    def build_argparser(self):
        parser = ArgumentParser(add_help=True)
        args = parser.add_argument_group('Common options')
        args.add_argument('-display', '--display', action='store_true',
                        help="Optional. Display detection output.")
        args.add_argument('-i', '--input_stream', required=False, type=str, default='0', 
                        help='Optional. Camera or video input stream. Default : /dev/video0')
        args.add_argument('-bs', '--bbox_buf_size', required=False, type=int, default=10, 
                        help='Optional. Size of bbox buffer')
        args.add_argument('-maw', '--max_average_bbox_width', required=False, type=int, default=20, 
                        help='Optional. Maximum value of the average width of bbox in buffer')
        args.add_argument('-mcr', '--max_center_ratio', required=False, type=int, default=0.7, 
                        help='Optional. Maximum value of ratio of bbox x coordinate center')
        args.add_argument('-mwr', '--min_bbox_width_ratio', required=False, type=float, default=0.05, 
                        help='Optional. Minimum width ratio of bbox')
        args.add_argument('-mma', '--max_miss_age', required=False, type=int, default=40, 
                        help='Optional. Maximum period to tolerate detection failure')

        model_args = parser.add_argument_group('Detection model options')
        model_args.add_argument('-mp', '--model_path', required=True, type=str,
                        help='Required.')
        model_args.add_argument('-cf', '--class_config_file', required=True, type=str, 
                        help='Required.')
        model_args.add_argument('-d', '--device', required=False, type=str, default='MYRIAD', 
                        help='Optional.')
        model_args.add_argument('-pt', '--prob_threshold', required=False, type=float, default=0.5, 
                        help='Optional.')
        model_args.add_argument('--sync', action='store_true', required=False,
                        help='Optional')


        model_type_args = parser.add_mutually_exclusive_group(required=True)
        model_type_args.add_argument('--ssd', action='store_true',
                        help='[ssd / yolo]')
        model_type_args.add_argument('--yolo', action='store_true',
                        help='[ssd / yolo]')

        return parser

    class ConstVariable():
        """
        BBUF_SIZE : Size of bbox buffer
        MAX_AVG_BWIDTH : Maximum value of the average width of bbox in buffer
        MAX_CENTER_RATIO : Maximum value of ratio of bbox x coordinate center
        MIN_BWIDTH_RATIO : Minimum width ratio of bbox
        MAX_MISS_AGE : Maximum period to tolerate detection failure
        FRAME_WIDTH : Input frame's width
        FRAME_HEIGHT : Input frame's height
        INF : Value that means infinity
        """
        def __init__(self):
            self.BBUF_SIZE = int()
            self.MAX_AVG_BWIDTH = int()
            self.MAX_CENTER_RATIO = float()
            self.MIN_BWIDTH_RATIO = float()
            self.MAX_MISS_AGE = int()

            self.FRAME_WIDTH = int()
            self.FRAME_HEIGHT = int()
            self.INF = 987654321

    def init_variable(self, args):
        self.CONST = self.ConstVariable()

        self.CONST.BBUF_SIZE = args.bbox_buf_size
        self.CONST.MAX_AVG_BWIDTH = args.max_average_bbox_width
        self.CONST.MAX_CENTER_RATIO = args.max_center_ratio
        self.CONST.MIN_BWIDTH_RATIO = args.min_bbox_width_ratio
        self.CONST.MAX_MISS_AGE = args.max_miss_age

        self.CONST.FRAME_WIDTH = int(self.cap.get_width())
        self.CONST.FRAME_HEIGHT = int(self.cap.get_height())
        
        self.label_map_ = ['white','red','orange','yellow','green',
                    'sky','blue','mint','pink','purple',
                    'darkgreen','beige','brown','gray','black']
        self.target_class_ = self.get_target_class(args.class_config_file)
        self.time_since_detect_ = 0
        self.bbox_buf_ = [self.CONST.MAX_AVG_BWIDTH for _ in range(self.CONST.BBUF_SIZE)]
    
    def get_target_class(self, class_config_file):
        with open(class_config_file, 'r') as f:
            line = f.readline().split(':')
        class_id = int(line[1]) - 1

        return class_id

    def main(self, pub):
        target_state = ObjectState.STABLE

        while not rospy.is_shutdown():
            
            ret, frame = self.cap.read()
            if not ret:
                break
        
            res = self.detector.inference(frame)
            res = self.filter_class(res, self.target_class_)

            if self.display_flag:
                out_frame = self.detector.get_results_img(res)
                if self.cap.is_zoom():
                    out_frame = cv2.resize(out_frame, (self.CONST.FRAME_WIDTH, self.CONST.FRAME_HEIGHT), interpolation=cv2.INTER_CUBIC)

                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(out_frame, target_state.name, (20, 30), font, 0.7, (255, 0, 0), 2)
                cv2.putText(out_frame, "Zoom : %d%%"%(int(self.cap.get_zoom_rate()*100)) if self.cap.is_zoom() else "Normal", \
                            (20, 70), font, 0.7, (0, 255, 0), 2)
                cv2.imshow("EDIE View", out_frame)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    break

            if self.cap.is_zoom():
                res = self.cap.restore_bbox(res)

            area_center_ratio, area_width_ratio, max_bbox_width_ratio, avg_bbox_width = self.parse_res(res)

            self.cap.update(area_center_ratio, max_bbox_width_ratio)

            target_state = self.get_target_state(res, self.cap.get_mode(), avg_bbox_width)
            if target_state == ObjectState.STABLE:
                pass
            elif target_state == ObjectState.APPROACH or target_state == ObjectState.MISS:
                self.cap.set_mode(CameraState.NORMAL)
            elif target_state == ObjectState.FARAWAY:
                self.cap.set_mode(CameraState.ZOOM)

            if len(res) > 0:
                if max_bbox_width_ratio < self.CONST.MIN_BWIDTH_RATIO:
                    self.cap.set_mode(CameraState.ZOOM)
                if abs(area_center_ratio) > self.CONST.MAX_CENTER_RATIO:
                    self.cap.set_mode(CameraState.NORMAL)

            self.publish_process(pub, area_center_ratio, area_width_ratio, max_bbox_width_ratio)

        cv2.destroyAllWindows()

    def filter_class(self, bboxs, target):
        def is_target(bbox):
            class_id = bbox['class_id']
            return class_id == target
        return list(filter(is_target, bboxs))
    
    def parse_res(self, res):
        object_count = len(res)
        if object_count == 0:
            err = 0
            return err, err, err, err

        area_xmin = self.CONST.INF
        area_xmax = 0

        max_bbox_width = 0
        avg_bbox_width = 0

        x_center_sum = 0
        width_sum = 0

        for bbox in res:
            xmin, xmax = bbox['xmin'], bbox['xmax']
            x_center_sum += (xmin + xmax) / 2
            
            if area_xmin > xmin:
                area_xmin = xmin
            if area_xmax < xmax:
                area_xmax = xmax
            if max_bbox_width < (xmax - xmin):
                max_bbox_width = xmax - xmin
            width_sum += xmax - xmin
        avg_x_center = x_center_sum / object_count

        frame_width = self.CONST.FRAME_WIDTH
        area_center_ratio = (avg_x_center - frame_width / 2) / (frame_width / 2)
        area_width_ratio = (area_xmax - area_xmin) / frame_width
        max_bbox_width_ratio = max_bbox_width / frame_width
        avg_bbox_width = width_sum / object_count

        return area_center_ratio, area_width_ratio, max_bbox_width_ratio, avg_bbox_width
    
    def get_target_state(self, res, cap_mode, avg_bbox_width):

        def is_outlier(value):
            return abs(self.bbox_buf_[-1] - value) > 20

        def is_closer(ratio):
            return ratio >= 0.25
        
        def is_far(ratio):
            return ratio <= -0.1

        if len(res) == 0:
            self.time_since_detect_ += 1
            if self.time_since_detect_ > self.CONST.MAX_MISS_AGE:
                return ObjectState.MISS
            else:
                return ObjectState.STABLE
        else:
            self.time_since_detect_ = 0

            if is_outlier(avg_bbox_width):
                return ObjectState.STABLE
            else:
                object_state = ObjectState.STABLE

                avg_buf_width = sum(self.bbox_buf_) / len(self.bbox_buf_)
                ratio = (avg_bbox_width - avg_buf_width) / avg_buf_width

                if cap_mode == CameraState.ZOOM:
                    if is_closer(ratio):
                        object_state = ObjectState.APPROACH
                else:
                    if is_far(ratio):
                        object_state = ObjectState.FARAWAY

                self.bbox_buf_.pop(0)
                self.bbox_buf_.append(avg_bbox_width)
                
                return object_state
    
    def publish_process(self, pub, area_center_ratio, area_width_ratio, max_bbox_width_ratio):
        edie_view_msgs = EdieView()
        edie_view_msgs.area_center_ratio = area_center_ratio
        edie_view_msgs.area_ratio = area_width_ratio
        edie_view_msgs.max_box_ratio = max_bbox_width_ratio

        pub.publish(edie_view_msgs)

if __name__ == "__main__":
    rospy.init_node('edie_detector_node', anonymous=False)

    pub = rospy.Publisher('/edie/view', EdieView, queue_size=1)

    edie_detector = EdieDetector()
    edie_detector.main(pub)