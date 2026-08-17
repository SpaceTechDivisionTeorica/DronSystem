import time

from .camera_interface import CameraInterface
from .frame import Frame


class PiCamera3(CameraInterface):

    def __init__(
        self,
        width=1280,
        height=720,
        fps=30,
        pixel_format="RGB888",
    ):
        self.width = width
        self.height = height
        self.fps = fps

        self.pixel_format = pixel_format

        self.camera = None
        self.opened = False
        self.frame_id = 0

    def open(self) -> None:

        if self.opened:
            return

        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise RuntimeError(
                "Picamera2 no está disponible. "
                "Este módulo debe utilizarse en "
                "la Raspberry Pi."
            ) from error

        self.camera = Picamera2()

        config = (
            self.camera.create_video_configuration(
                main={
                    "size": (
                        self.width,
                        self.height
                    ),
                    "format": self.pixel_format,
                },
                controls={
                    "FrameRate": self.fps
                },
            )
        )

        self.camera.configure(config)

        self.camera.start()

        self.frame_id = 0
        self.opened = True

    def read(self) -> Frame:

        if not self.opened:
            raise RuntimeError(
                "PiCamera3 no está abierta"
            )

        image = self.camera.capture_array(
            "main"
        )

        frame = Frame(
            image=image,
            timestamp=time.monotonic(),
            frame_id=self.frame_id,
            source="picamera3",
            color_space="RGB",
        )

        self.frame_id += 1

        return frame

    def is_open(self) -> bool:
        return self.opened

    def close(self) -> None:

        if self.camera is not None:
            self.camera.stop()

        self.camera = None
        self.opened = False