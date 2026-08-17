from abc import ABC, abstractmethod

from .tof_frame import ToFFrame


class ToFInterface(ABC):

    @abstractmethod
    def open(self) -> None:
        pass

    @abstractmethod
    def read(self) -> ToFFrame | None:
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

    def __exit__(
        self,
        exc_type,
        exc_value,
        traceback
    ):
        self.close()