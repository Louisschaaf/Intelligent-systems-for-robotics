# Volledige, minimale stub: geeft ground-truth pose van DEF-nodes terug.
from controller import Supervisor
from controller import Camera
from ultralytics import YOLO
import numpy as np
import cv2

class CameraDetection:
    def __init__(self, robot: Supervisor, camera_def: str):
        self.robot = robot
        self.camera_def = camera_def
        self.camera = robot.getDevice("Astra rgb")
        self.camera_width = self.camera.getWidth()
        self.camera_height = self.camera.getHeight()
        if self.camera is None:
            raise RuntimeError(f"Camera met DEF {camera_def} niet gevonden.")
        self.model = YOLO("yolo11n.pt")

    def enable(self, timestep: int):
        self.camera.enable(timestep)

    def get_image(self):
        img = np.frombuffer(self.camera.getImage(), np.uint8).reshape((self.camera_height, self.camera_width, 4))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img_bgr

    def get_pose_from_def(self, def_name: str):
        node = self.robot.getFromDef(def_name)
        if node is None:
            raise RuntimeError(f"DEF {def_name} niet gevonden.")
        return node.getPosition(), node.getOrientation()
    
    def detect_objects(self):
        image = self.get_image()
        results = self.model(image)
        detections = []
        for result in results:
            for box in result.boxes:
                detections.append({
                    "class": box.cls,
                    "confidence": box.conf,
                    "bbox": box.xyxy
                })
        return detections
    
    def detect_object(self, name: str):
        detections = self.detect_objects()
        for detection in detections:
            if detection["class"] == name:
                return detection
        return None