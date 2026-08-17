from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    image: np.ndarray
    timestamp: float
    frame_id: int
    source: str
    color_space: str = "BGR"

    def __post_init__(self):
        if not isinstance(self.image, np.ndarray):
            raise TypeError("image debe ser un numpy.ndarray")

        if self.image.size == 0:
            raise ValueError("La imagen no puede estar vacía")

        if self.image.ndim not in (2, 3):
            raise ValueError(
                "La imagen debe tener 2 o 3 dimensiones"
            )

        if self.frame_id < 0:
            raise ValueError(
                "frame_id no puede ser negativo"
            )

    @property
    def height(self) -> int:
        return self.image.shape[0]

    @property
    def width(self) -> int:
        return self.image.shape[1]

    @property
    def dtype(self):
        return self.image.dtype

    @property
    def shape(self):
        return self.image.shape