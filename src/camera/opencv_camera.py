import time
from pathlib import Path

import cv2

from .camera_interface import CameraInterface
from .frame import Frame


class OpenCVCamera(CameraInterface):

    def __init__(
        self,
        source=0,
        width=1280,
        height=720,
        fps=30,
        source_name="opencv",
    ):
        self.source = source

        self.width = width
        self.height = height
        self.fps = fps

        self.source_name = source_name

        self.camera = None
        self.frame_id = 0

    def open(self) -> None:

        if isinstance(self.source, str):
            path = Path(self.source)

            if not path.exists():
                raise FileNotFoundError(
                    f"No existe el archivo: {self.source}"
                )

        self.camera = cv2.VideoCapture(self.source)

        if not self.camera.isOpened():
            self.camera.release()
            self.camera = None

            raise RuntimeError(
                f"No se pudo abrir la fuente: {self.source}"
            )

        # Estas propiedades tienen sentido principalmente
        # para webcams/cámaras USB.
        if isinstance(self.source, int):
            self.camera.set(
                cv2.CAP_PROP_FRAME_WIDTH,
                self.width
            )

            self.camera.set(
                cv2.CAP_PROP_FRAME_HEIGHT,
                self.height
            )

            self.camera.set(
                cv2.CAP_PROP_FPS,
                self.fps
            )

        self.frame_id = 0

    def read(self) -> Frame | None:

        if not self.is_open():
            raise RuntimeError(
                "La cámara no está abierta"
            )

        success, image = self.camera.read()

        if not success:
            return None

        frame = Frame(
            image=image,
            timestamp=time.monotonic(),
            frame_id=self.frame_id,
            source=self.source_name,
            color_space="BGR",
        )

        self.frame_id += 1

        return frame

    def is_open(self) -> bool:

        return (
            self.camera is not None
            and self.camera.isOpened()
        )

    def close(self) -> None:

        if self.camera is not None:
            self.camera.release()

        self.camera = None