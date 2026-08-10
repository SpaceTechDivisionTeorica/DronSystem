import cv2

from .camera_interface import CameraInterface


class OpenCVCamera(CameraInterface):

    def __init__(self, source=0):

        self.source = source
        self.camera = None


    def open(self):

        self.camera = cv2.VideoCapture(
            self.source
        )


    def read(self):

        if not self.is_open():
            raise RuntimeError(
                "La cámara no está abierta."
            )

        success, frame = self.camera.read()

        if not success:
            return None

        return frame


    def is_open(self):

        return (
            self.camera is not None
            and self.camera.isOpened()
        )


    def close(self):

        if self.camera is not None:
            self.camera.release()

        self.camera = None