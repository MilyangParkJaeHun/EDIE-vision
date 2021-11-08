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
# from .openvino_detector.Model.Ssd import Ssd
from utils import ZoomCamera, ObjectState
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
    args.add_argument('-mxr', '--max_x_ratio', required=False, type=int, default=20, 
                    help='Optional.')
    args.add_argument('-mwr', '--min_bbox_width_ratio', required=False, type=float, default=0.7, 
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
    pass

def get_target_state(res):
    # (x_center, area_ratio, max_box_ratio, avg_box) = parse_res(res)

    return ObjectState.FARAWAY

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
    global label_map_, target_class_, bbox_buf_

    BBUF_SIZE = args.bbox_buf_size
    MAX_AVG_BWIDTH = args.max_average_bbox_width
    MAX_X_RATIO = args.max_x_ratio
    MIN_BWIDTH_RATIO = args.min_bbox_width_ratio
    FRAME_WIDTH = cap.get_width()
    FRAME_HEIGHT = cap.get_height()
    
    label_map_ = ['white','red','orange','yellow','green',
                'sky','blue','mint','pink','purple',
                'darkgreen','beige','brown','gray','black']

    target_class_ = get_target_class(args.class_config_file)

    bbox_buf_ = [MAX_AVG_BWIDTH for _ in range(BBUF_SIZE)]


def main():
    global label_map_, target_class_, bbox_buf_, time_since_detect_

    args = build_argparser().parse_args()
    cap = ZoomCamera(args.input_stream)

    init_process(args, cap)

    model = Yolo()
    detector = OpenvinoDet(model_parser=model, \
                        model_path=args.model_path, \
                        device=args.device, \
                        label_map=label_map_, \
                        prob_threshold=args.prob_threshold)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        
        if not ret:
            break
    
        res = detector.inference(frame)

        if len(res) == 0:
            time_since_detect_ += 1
        else:
            time_since_detect_ = 0
            res = filter_class(res, target_class_)

        if args.show_on:
            out_frame = detector.get_results_img(res)
            if cap.is_zoom:
                width = int(cap.get_width())
                height = int(cap.get_height())
                out_frame = cv2.resize(out_frame, (width, height), interpolation=cv2.INTER_CUBIC)
            cv2.imshow("test", out_frame)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

            if key == ord('z'):
                cap.zoom_out()
            elif key == ord('o'):
                cap.zoom_in()

        if cap.is_zoom():
            cap.restore_coord(res)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node('edie_detector_node', anonymous=False)

    pub = rospy.Publisher('/edie/view', EdieView, queue_size=1)

    main()