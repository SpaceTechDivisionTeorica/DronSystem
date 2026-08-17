from abc import ABC, abstractmethod

from .frame import Frame



class CameraInterface(ABC):

    @abstractmethod
    def open(self) -> None:
        pass

    @abstractmethod
    def read(self) -> Frame | None:
        pass

    @abstractmethod
    def is_open(self) -> bool:
        pass

    @abstractmethod
    def close(self) -> None:
        pass

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()