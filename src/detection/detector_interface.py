from abc import ABC, abstractmethod

from src.camera.frame import Frame

from .detection import Detection


class DetectorInterface(ABC):

    @abstractmethod
    def load(self) -> None:
        pass

    @abstractmethod
    def predict(
        self,
        frame: Frame
    ) -> list[Detection]:
        pass

    @abstractmethod
    def close(self) -> None:
        pass

    def __enter__(self):
        self.load()
        return self

    def __exit__(
        self,
        exc_type,
        exc_value,
        traceback
    ):
        self.close()