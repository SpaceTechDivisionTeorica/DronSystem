import time

import cv2
import numpy as np

from .camera_interface import CameraInterface
from .frame import Frame


class MockCamera(CameraInterface):

    def __init__(
        self,
        width=1280,
        height=720,
        fps=30,
    ):
        self.width = width
        self.height = height
        self.fps = fps

        self.opened = False
        self.frame_id = 0

        self._last_frame_time = None

    def open(self) -> None:

        self.opened = True
        self.frame_id = 0
        self._last_frame_time = None

    def read(self) -> Frame:

        if not self.opened:
            raise RuntimeError(
                "MockCamera no está abierta"
            )

        self._wait_for_next_frame()

        image = np.zeros(
            (
                self.height,
                self.width,
                3
            ),
            dtype=np.uint8,
        )

        # Objeto simulado moviéndose horizontalmente.
        box_width = 120
        box_height = 120

        available_width = max(
            1,
            self.width - box_width
        )

        x = (
            self.frame_id * 10
        ) % available_width

        y = (
            self.height - box_height
        ) // 2

        cv2.rectangle(
            image,
            (x, y),
            (
                x + box_width,
                y + box_height
            ),
            (255, 255, 255),
            -1,
        )

        frame = Frame(
            image=image,
            timestamp=time.monotonic(),
            frame_id=self.frame_id,
            source="mock_camera",
            color_space="BGR",
        )

        self.frame_id += 1

        return frame

    def _wait_for_next_frame(self) -> None:

        if self.fps <= 0:
            return

        period = 1.0 / self.fps

        now = time.monotonic()

        if self._last_frame_time is not None:

            elapsed = (
                now - self._last_frame_time
            )

            remaining = period - elapsed

            if remaining > 0:
                time.sleep(remaining)

        self._last_frame_time = time.monotonic()

    def is_open(self) -> bool:
        return self.opened

    def close(self) -> None:
        self.opened = False