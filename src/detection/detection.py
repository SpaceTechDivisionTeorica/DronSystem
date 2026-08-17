from dataclasses import dataclass

from .bounding_box import BoundingBox


@dataclass
class Detection:

    class_id: int
    class_name: str

    confidence: float

    bbox: BoundingBox

    frame_id: int

    timestamp: float | None = None

    def __post_init__(self):

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError(
                "confidence debe estar "
                "entre 0 y 1"
            )

    def to_dict(self) -> dict:

        return {
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox": {
                "x1": self.bbox.x1,
                "y1": self.bbox.y1,
                "x2": self.bbox.x2,
                "y2": self.bbox.y2,
            },
            "frame_id": self.frame_id,
            "timestamp": self.timestamp,
        }