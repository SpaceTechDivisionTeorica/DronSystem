from .camera_interface import CameraInterface


class PiCamera3(CameraInterface):

    def __init__(
        self,
        width=1280,
        height=720,
    ):

        self.width = width
        self.height = height

        self.camera = None
        self.opened = False


    def open(self):

        from picamera2 import Picamera2

        self.camera = Picamera2()

        config = self.camera.create_video_configuration(
            main={
                "size": (
                    self.width,
                    self.height
                )
            }
        )

        self.camera.configure(config)

        self.camera.start()

        self.opened = True


    def read(self):

        if not self.opened:
            raise RuntimeError(
                "La cámara no está abierta."
            )

        return self.camera.capture_array()


    def is_open(self):

        return self.opened


    def close(self):

        if self.camera is not None:
            self.camera.stop()

        self.camera = None
        self.opened = False