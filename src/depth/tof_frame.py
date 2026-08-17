from dataclasses import dataclass

import numpy as np


@dataclass
class ToFFrame:

    distances: np.ndarray

    timestamp: float

    frame_id: int

    source: str

    valid_mask: np.ndarray | None = None

    def __post_init__(self):

        if not isinstance(
            self.distances,
            np.ndarray
        ):
            raise TypeError(
                "distances debe ser numpy.ndarray"
            )

        if self.distances.ndim != 2:
            raise ValueError(
                "distances debe ser "
                "una matriz 2D"
            )

        if self.valid_mask is None:

            self.valid_mask = (
                np.isfinite(self.distances)
                & (self.distances > 0)
            )

        if (
            self.valid_mask.shape
            != self.distances.shape
        ):
            raise ValueError(
                "valid_mask y distances "
                "deben tener el mismo tamaño"
            )

    @property
    def shape(self):
        return self.distances.shape

    def valid_distances(self):

        return self.distances[
            self.valid_mask
        ]

    def min_distance(
        self
    ) -> float | None:

        values = self.valid_distances()

        if values.size == 0:
            return None

        return float(
            np.min(values)
        )

    def median_distance(
        self
    ) -> float | None:

        values = self.valid_distances()

        if values.size == 0:
            return None

        return float(
            np.median(values)
        )