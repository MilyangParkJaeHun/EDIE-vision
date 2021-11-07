#!/usr/bin/env python3
import os
import cv2
from enum import Enum, auto
from argparse import ArgumentParser

import rospy
import sys
from edie_msgs.msg import EdieView

from openvino_detector.DetModel import OpenvinoDet
from openvino_detector.Model.Yolo import Yolo
from openvino_detector.Model.Ssd import Ssd

def build_argparser():
    parser = ArgumentParser(add_help=True)
    args = parser.add_argument_group('Common options')
    args.add_argument('-show_on', '--show_on', action='store_true',
                    help="Optional. Show output.")

    model_args = parser.add_argument_group('Detection model options')
    model_args.add_argument('-mp', '--model_path', required=True, type=int,
                    help='Required.')
    model_args.add_argument('-d', '--device', required=False, type=str, default='MYRIAD', 
                    help='Optional.')
    model_args.add_argument('-pt', '--prob_threshold', required=False, type=int, default=0.5, 
                    help='Optinal.')

    return parser

class ObjectState(Enum):
    STABLE = auto()
    APPROACH = auto()
    FARAWAY = auto()

def main():
    args = build_argparser().parse_args()
    label_map = ['white','red','orange','yellow','green',
                'sky','blue','mint','pink','purple',
                'darkgreen','beige','brown','gray','black']

    model = Yolo()
    detector = OpenvinoDet(model_parser=model, \
                        model_path=args.model_path, \
                        device=args.device, \
                        label_map=label_map, \
                        prob_threshold=args.prob_threshold)

    cap = zoom_camera()

    while not rospy.is_shutdown():
        cap_err, frame = cap.read()
        
        if cap_err:
            break
    
        res = detector.inference(frame)

        if cap.is_zoom():
            res = cap.restore_coord(res)
        
        target_state = get_target_state(res)

        if target_state == ObjectState.STABLE:
            pass
        elif target_state == ObjectState.APPROACH:
            pass
        elif target_state == ObjectState.FARAWAY:
            pass

        out_frame = detector.get_results_img(res)
        cv2.imshow("test", out_frame)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node('edie_detector_node', anonymous=False)

    pub = rospy.Publisher('/edie/view', EdieView, queue_size=1)

    try:
        main()
    except:
        sys.exit(1)