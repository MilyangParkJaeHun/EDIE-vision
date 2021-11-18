# EDIE_Vision
Object detection system for EDIE

Supports YOLO and SSD type detection model optimized by OpenVINO

# Environments
- Ubuntu 20.04
- ROS noetic
- OpenVINO
    - [openvino git](https://github.com/openvinotoolkit/openvino) checkpoint : 3775efd54a48421c0e1c50834a721e8e87bc95e2
- OpenCV 4.5.2-openvino

# Build
```
$ git clone --recursive https://github.com/MilyangParkJaeHun/EDIE-vision.git
$ catkin_make -C ~/catkin_ws
```

# Version Notice
- if you use openvino 2021.3.394 version, you need to change submodule branch
    ```
    $ vi /path/to/EDIE-vision/.gitmodules
    ```
    ```
    [submodule "edie_detector/scripts/openvino_detector"]
        path = edie_detector/scripts/openvino_detector
        url = https://github.com/MilyangParkJaeHun/openvino_detector.git
    +   branch = openvino-2021.3.394
    ```
    ```
    $ git submodule update --remote
    ```

# Demo Run
1. Downloads YOLO/SSD IR file
    ```
    $ cd /path/to/EDIE-vision
    $ source model_downloads.sh
    ```
2. Run using CPU
- SSD version
    ```
    $ rosrun edie_detector edie_detector_node.py \
        -i 0 \
        -mp /path/to/EDIE-vision/edie_detector/scripts/openvino_detector/IR/Ssd/edie-ssd-mobilenet-v2 \
        -cf /path/to/EDIE-vision/edie_detector/scripts/EDIE_ID.txt \
        --ssd \
        --display \
        --device CPU
    ```
- YOLO version
    ```
    $ rosrun edie_detector edie_detector_node.py \
        -i 0 \
        -mp /path/to/EDIE-vision/edie_detector/scripts/openvino_detector/IR/Yolo/edie-yolov4-tiny \
        -cf /path/to/EDIE-vision/edie_detector/scripts/EDIE_ID.txt \
        --yolo \
        --display \
        --device CPU
    ```
3. Run using intel NCS2
- SSD version
    ```
    $ rosrun edie_detector edie_detector_node.py \
        -i 0 \
        -mp /path/to/EDIE-vision/edie_detector/scripts/openvino_detector/IR/Ssd/edie-ssd-mobilenet-v2 \
        -cf /path/to/EDIE-vision/edie_detector/scripts/EDIE_ID.txt \
        --ssd \
        --display \
        --device MYRIAD
    ```
- YOLO version
    ```
    $ rosrun edie_detector edie_detector_node.py \
        -i 0 \
        -mp /path/to/EDIE-vision/edie_detector/scripts/openvino_detector/IR/Yolo/edie-yolov4-tiny \
        -cf /path/to/EDIE-vision/edie_detector/scripts/EDIE_ID.txt \
        --yolo \
        --display \
        --device MYRIAD
    ```
