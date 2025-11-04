from controller import Supervisor
from ultralytics import YOLO
import numpy as np
import cv2

class CameraDetection:
    def __init__(self, robot: Supervisor, camera_name: str = "Astra rgb", model_path: str = "yolo11n.pt", range_finder_name: str = "Astra depth"):
        self.robot = robot
        self.camera = robot.getDevice(camera_name)
        if self.camera is None:
            raise RuntimeError(f"Camera device '{camera_name}' niet gevonden.")
        self.model = YOLO(model_path)
        self.range_finder = robot.getDevice(range_finder_name)
        if self.range_finder is None:
            raise RuntimeError(f"RangeFinder device '{range_finder_name}' niet gevonden.")
        self.width = None
        self.height = None
        self._frame = None
        self._detections = []
        self._tick = 0

        # Pak klassennamen dynamisch uit het model (beter dan vaste COCO-map):
        # self.model.names is bij Ultralytics een dict: {id: "classname"}
        self.id2name = {int(k): v for k, v in getattr(self.model, "names", {}).items()}

    def enable(self, timestep: int):
        self.camera.enable(timestep)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        self.range_finder.enable(timestep)


    def get_image(self):
        buf = self.camera.getImage()
        if buf is None:
            return None
        img = np.frombuffer(buf, np.uint8).reshape((self.height, self.width, 4))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        self._frame = img_bgr
        return img_bgr

    def _run_yolo(self):
        if self._frame is None:
            return []
        results = self.model(self._frame, verbose=False)
        out = []
        for r in results:
            if not hasattr(r, "boxes") or r.boxes is None:
                continue
            for b in r.boxes:
                cls_id = int(b.cls[0])
                name = self.id2name.get(cls_id, str(cls_id))
                conf = float(b.conf[0]) if b.conf is not None else 0.0
                xyxy = b.xyxy[0].tolist()
                out.append({"class_id": cls_id, "class_name": name, "confidence": conf, "bbox_xyxy": xyxy})
        return out

    def detect_objects(self, every_n_frames: int = 3):
        self._tick += 1
        if self._tick % every_n_frames == 0:
            self._detections = self._run_yolo()
        return self._detections

    def detect_object(self, class_name: str):
        class_name = class_name.lower().strip()
        dets = self.detect_objects()
        matches = [d for d in dets if d["class_name"].lower() == class_name]
        if not matches:
            return None
        return max(matches, key=lambda d: d["confidence"])
    
    def get_distance_to_object(self, detection: dict):
        if detection is None or self.range_finder is None:
            return None
        x1, y1, x2, y2 = map(int, detection["bbox_xyxy"])
        cx = round((x1 + x2) / 2)
        cy = round((y1 + y2) / 2)
        depth_buf = self.range_finder.getRangeImage()
        if depth_buf is None:
            return None
        depth_image = np.array(depth_buf, dtype=np.float32).reshape((self.range_finder.getHeight(), self.range_finder.getWidth()))
        if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
            distance = depth_image[cy, cx]
            return float(distance)
        return None
    
    # function to check the depth of the whole camera view and returns the minimum distance
    def get_min_distance(self):
        if self.range_finder is None:
            return None
        depth_buf = self.range_finder.getRangeImage()
        if depth_buf is None:
            return None
        depth_image = np.array(depth_buf, dtype=np.float32).reshape((self.range_finder.getHeight(), self.range_finder.getWidth()))
        min_distance = np.min(depth_image)
        return float(min_distance)