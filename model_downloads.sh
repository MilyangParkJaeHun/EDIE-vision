#!/bin/bash

wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=13lK8n9QNnp3S1tCo22rAOWv6pXgx_Kp6' -O edie-yolov4-tiny.tar.xz
tar -xvf edie-yolov4-tiny.tar.xz
rm edie-yolov4-tiny.tar.xz
mv edie-yolov4-tiny edie_detector/scripts/openvino_detector/IR/Yolo/

wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=10T4Clz-PMfxydcualaFvD0GIdWS0ZpDU' -O edie-ssd-mobilenet-v2.tar.xz
tar -xvf edie-ssd-mobilenet-v2.tar.xz
rm edie-ssd-mobilenet-v2.tar.xz
mv edie-ssd-mobilenet-v2 edie_detector/scripts/openvino_detector/IR/Ssd/