from dataclasses import dataclass


@dataclass
class BoundingBox:

    x1: float
    y1: float
    x2: float
    y2: float

    def __post_init__(self):

        if self.x2 < self.x1:
            raise ValueError(
                "x2 debe ser mayor o igual a x1"
            )

        if self.y2 < self.y1:
            raise ValueError(
                "y2 debe ser mayor o igual a y1"
            )

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def area(self) -> float:
        return (
            self.width
            * self.height
        )

    @property
    def center(self) -> tuple[float, float]:

        cx = (
            self.x1 + self.x2
        ) / 2.0

        cy = (
            self.y1 + self.y2
        ) / 2.0

        return cx, cy

    def as_xyxy(self) -> tuple:

        return (
            self.x1,
            self.y1,
            self.x2,
            self.y2,
        )

    def clamp(
        self,
        width: int,
        height: int
    ):

        self.x1 = max(
            0,
            min(self.x1, width - 1)
        )

        self.y1 = max(
            0,
            min(self.y1, height - 1)
        )

        self.x2 = max(
            0,
            min(self.x2, width - 1)
        )

        self.y2 = max(
            0,
            min(self.y2, height - 1)
        )

        return self