import math
import time

import numpy as np

from .tof_frame import ToFFrame
from .tof_interface import ToFInterface


class MockToF(ToFInterface):

    def __init__(
        self,
        rows=8,
        cols=8,
        fps=15,
        background_distance=3.0,
    ):

        self.rows = rows
        self.cols = cols

        self.fps = fps

        self.background_distance = (
            background_distance
        )

        self.opened = False
        self.frame_id = 0

        self._last_frame_time = None

    def open(self) -> None:

        self.opened = True

        self.frame_id = 0

        self._last_frame_time = None

    def read(self) -> ToFFrame:

        if not self.opened:
            raise RuntimeError(
                "MockToF no está abierto"
            )

        self._wait_for_next_frame()

        distances = np.full(
            (
                self.rows,
                self.cols
            ),
            self.background_distance,
            dtype=np.float32,
        )

        # Simulamos un obstáculo en el centro
        # cuya distancia cambia suavemente.

        obstacle_distance = (
            1.3
            + 0.4
            * math.sin(
                self.frame_id * 0.1
            )
        )

        center_row = (
            self.rows // 2
        )

        center_col = (
            self.cols // 2
        )

        row_start = max(
            0,
            center_row - 1
        )

        row_end = min(
            self.rows,
            center_row + 1
        )

        col_start = max(
            0,
            center_col - 1
        )

        col_end = min(
            self.cols,
            center_col + 1
        )

        distances[
            row_start:row_end,
            col_start:col_end
        ] = obstacle_distance

        valid_mask = np.ones(
            (
                self.rows,
                self.cols
            ),
            dtype=bool,
        )

        frame = ToFFrame(
            distances=distances,
            timestamp=time.monotonic(),
            frame_id=self.frame_id,
            source="mock_tof",
            valid_mask=valid_mask,
        )

        self.frame_id += 1

        return frame

    def _wait_for_next_frame(self):

        if self.fps <= 0:
            return

        period = 1.0 / self.fps

        now = time.monotonic()

        if self._last_frame_time is not None:

            elapsed = (
                now
                - self._last_frame_time
            )

            remaining = (
                period - elapsed
            )

            if remaining > 0:
                time.sleep(remaining)

        self._last_frame_time = (
            time.monotonic()
        )

    def is_open(self) -> bool:
        return self.opened

    def close(self) -> None:
        self.opened = False