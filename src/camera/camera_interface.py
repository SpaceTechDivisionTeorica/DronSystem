from abc import ABC, abstractmethod

class CameraInterface(ABC):
    
    @abstractmethod
    def open(self):
        """Inicialización de la cámara"""
        pass

    @abstractmethod
    def read(self):
        """Devuelve el siguiente Frame"""
        pass

    @abstractmethod
    def is_open(self):
        """Indica si la cámara está disponible"""
        pass

    @abstractmethod
    def close(self):
        """Libera los recursos de la cámara"""
        pass
    