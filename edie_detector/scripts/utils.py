import cv2
from enum import Enum, auto

class InvalidInput(Exception):

    def __init__(self, message):
        self.message = message

class ObjectState(Enum):
    STABLE = auto()
    APPROACH = auto()
    FARAWAY = auto()

class CameraState(Enum):
    NORMAL = 0
    ZOOM = 1

class ZoomCamera():
    def __init__(self, input_stream):
        self.cap = cv2.VideoCapture()
        try:
            status = self.cap.open(int(input_stream))
        except:
            status = self.cap.open(input_stream)
        
        if not status:
            raise InvalidInput("Can't find input stream : {}".format(input_stream))
        
        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.mode = CameraState.NORMAL
        self.zoom_rate = 1.5
        self.x_center = self.width / 2
        self.y_center = self.height / 2

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return ret, None
    
        if self.is_zoom():
            return ret, self.zoom(frame)
        return ret, frame

    def get_width(self):
        return self.width
    
    def get_height(self):
        return self.height

    def set_x_center(self, x):
        self.x_center = x
    
    def is_zoom(self):
        return self.mode == CameraState.ZOOM

    def zoom(self, frame):
        x_gap = (self.width / self.zoom_rate) / 2
        y_gap = (self.height / self.zoom_rate) / 2

        start_x = self.x_center - x_gap
        end_x = self.x_center + x_gap

        if start_x < 0:
            center = x_gap
        elif end_x >= self.width:
            center = self.width - x_gap
        else:
            center = self.x_center

        xmin = int(center-x_gap)
        xmax = int(center+x_gap)
        ymin = int(self.height-2*y_gap)
        ymax = int(self.height)

        self.set_x_center(center)
        return frame[ymin:ymax, xmin:xmax]

    def zoom_in(self):
        self.mode = CameraState.ZOOM

    def zoom_out(self):
        self.mode = CameraState.NORMAL

    def restore_coord(self, res):
        def restore_x(x):
            w = self.width
            zoom_rate = self.zoom_rate
            x_center = self.x_center
            restored_x = int(x_center + (x - w / 2) / zoom_rate)

            return restored_x
        
        def restore_y(y):
            h = self.height
            zoom_rate = self.zoom_rate
            y_center = h - (h / 2) / zoom_rate
            restored_y = int(y_center + (y - h / 2) / zoom_rate)

            return restored_y

        for bbox in res:
            bbox['xmin'] = restore_x(bbox['xmin'])
            bbox['xmax'] = restore_x(bbox['xmax'])
            bbox['ymin'] = restore_y(bbox['ymin'])
            bbox['ymax'] = restore_y(bbox['ymax'])