#!/usr/bin/env python3
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
from utils import CameraState, ZoomCamera, ObjectState
from const_variable import *

label_map_ = list()
target_class_ = int()
time_since_detect_ = int()
bbox_buf_ = list()

def build_argparser():
    parser = ArgumentParser(add_help=True)
    args = parser.add_argument_group('Common options')
    args.add_argument('-show_on', '--show_on', action='store_true',
                    help="Optional. Show output.")
    args.add_argument('-i', '--input_stream', required=False, type=str, default='0', 
                    help='Optional.')
    args.add_argument('-bs', '--bbox_buf_size', required=False, type=int, default=10, 
                    help='Optional.')
    args.add_argument('-maw', '--max_average_bbox_width', required=False, type=int, default=20, 
                    help='Optional.')
    args.add_argument('-mcr', '--max_center_ratio', required=False, type=int, default=0.7, 
                    help='Optional.')
    args.add_argument('-mwr', '--min_bbox_width_ratio', required=False, type=float, default=0.05, 
                    help='Optional.')
    args.add_argument('-mma', '--max_miss_age', required=False, type=int, default=40, 
                    help='Optional.')

    model_args = parser.add_argument_group('Detection model options')
    model_args.add_argument('-mp', '--model_path', required=True, type=str,
                    help='Required.')
    model_args.add_argument('-cf', '--class_config_file', required=True, type=str, 
                    help='Required.')
    model_args.add_argument('-d', '--device', required=False, type=str, default='MYRIAD', 
                    help='Optional.')
    model_args.add_argument('-pt', '--prob_threshold', required=False, type=int, default=0.5, 
                    help='Optional.')

    return parser

def parse_res(res):
    global FRAME_WIDTH, FRAME_HEIGHT
    object_count = len(res)
    if object_count == 0:
        err = 0
        return err, err, err, err

    area_xmin = INF
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

    area_center_ratio = (avg_x_center - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2)
    area_width_ratio = (area_xmax - area_xmin) / FRAME_WIDTH
    max_bbox_width_ratio = max_bbox_width / FRAME_WIDTH
    avg_bbox_width = width_sum / object_count

    return area_center_ratio, area_width_ratio, max_bbox_width_ratio, avg_bbox_width

def get_target_state(res, cap_mode, avg_bbox_width):
    global time_since_detect_, MAX_MISS_AGE

    def is_outlier(value):
        return abs(bbox_buf_[-1] - value) > 20

    def is_closer(ratio):
        return ratio >= 0.25
    
    def is_far(ratio):
        return ratio <= -0.1

    if len(res) == 0:
        time_since_detect_ += 1
        if time_since_detect_ > MAX_MISS_AGE:
            return ObjectState.MISS
        else:
            return ObjectState.STABLE
    else:
        time_since_detect_ = 0

        if is_outlier(avg_bbox_width):
            return ObjectState.STABLE
        else:
            object_state = ObjectState.STABLE

            avg_buf_width = sum(bbox_buf_) / len(bbox_buf_)
            ratio = (avg_bbox_width - avg_buf_width) / avg_buf_width

            if cap_mode == CameraState.ZOOM:
                if is_closer(ratio):
                    object_state = ObjectState.APPROACH
            else:
                if is_far(ratio):
                    object_state = ObjectState.FARAWAY

            bbox_buf_.pop(0)
            bbox_buf_.append(avg_bbox_width)
            
            return object_state

def get_target_class(class_config_file):
    global target_class_

    with open(class_config_file, 'r') as f:
        line = f.readline().split(':')
    class_id = int(line[1]) - 1

    return class_id

def filter_class(bboxs, target):
    def is_target(bbox):
        class_id = bbox['class_id']
        return class_id == target
    return list(filter(is_target, bboxs))

def init_process(args, cap):
    global label_map_, target_class_, bbox_buf_, time_since_detect_, \
            BBUF_SIZE, MAX_AVG_BWIDTH, MAX_CENTER_RATIO, \
            MIN_BWIDTH_RATIO, MAX_MISS_AGE , FRAME_WIDTH, FRAME_HEIGHT

    BBUF_SIZE = args.bbox_buf_size
    MAX_AVG_BWIDTH = args.max_average_bbox_width
    MAX_CENTER_RATIO = args.max_center_ratio
    MIN_BWIDTH_RATIO = args.min_bbox_width_ratio
    MAX_MISS_AGE = args.max_miss_age

    FRAME_WIDTH = cap.get_width()
    FRAME_HEIGHT = cap.get_height()
    
    label_map_ = ['white','red','orange','yellow','green',
                'sky','blue','mint','pink','purple',
                'darkgreen','beige','brown','gray','black']
    target_class_ = get_target_class(args.class_config_file)
    time_since_detect_ = 0
    bbox_buf_ = [MAX_AVG_BWIDTH for _ in range(BBUF_SIZE)]

def publish_process(pub, area_center_ratio, area_width_ratio, max_bbox_width_ratio):
    edie_view_msgs = EdieView()
    edie_view_msgs.area_center_ratio = area_center_ratio
    edie_view_msgs.area_ratio = area_width_ratio
    edie_view_msgs.max_box_ratio = max_bbox_width_ratio

    pub.publish(edie_view_msgs)

def main(pub):
    global label_map_, target_class_, bbox_buf_, time_since_detect_, \
            MIN_BWIDTH_RATIO, MAX_CENTER_RATIO

    args = build_argparser().parse_args()
    cap = ZoomCamera(args.input_stream)

    init_process(args, cap)

    model = Yolo()
    detector = OpenvinoDet(model_parser=model, \
                        model_path=args.model_path, \
                        device=args.device, \
                        label_map=label_map_, \
                        prob_threshold=args.prob_threshold)

    target_state = ObjectState.STABLE
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
    
        res = detector.inference(frame)
        res = filter_class(res, target_class_)

        if args.show_on:
            out_frame = detector.get_results_img(res)
            if cap.is_zoom():
                width = int(cap.get_width())
                height = int(cap.get_height())
                out_frame = cv2.resize(out_frame, (width, height), interpolation=cv2.INTER_CUBIC)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(out_frame, target_state.name, (20, 30), font, 0.7, (255, 0, 0), 2)
            cv2.putText(out_frame, "Zoom : %d%%"%(int(cap.get_zoom_rate()*100)) if cap.is_zoom() else "Normal", \
                        (20, 70), font, 0.7, (0, 255, 0), 2)
            cv2.imshow("test", out_frame)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
    
        if cap.is_zoom():
            res = cap.restore_coord(res)

        area_center_ratio, area_width_ratio, max_bbox_width_ratio, avg_bbox_width = parse_res(res)
        target_state = get_target_state(res, cap.get_mode(), avg_bbox_width)

        cap.update(area_center_ratio, max_bbox_width_ratio)
        if target_state == ObjectState.STABLE:
            pass
        elif target_state == ObjectState.APPROACH or target_state == ObjectState.MISS:
            cap.zoom_out()
        elif target_state == ObjectState.FARAWAY:
            cap.zoom_in()

        if len(res) > 0:
            if max_bbox_width_ratio < MIN_BWIDTH_RATIO:
                cap.zoom_in()
            if abs(area_center_ratio) > MAX_CENTER_RATIO:
                cap.zoom_out()

        publish_process(pub, area_center_ratio, area_width_ratio, max_bbox_width_ratio)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node('edie_detector_node', anonymous=False)

    pub = rospy.Publisher('/edie/view', EdieView, queue_size=1)

    main(pub)