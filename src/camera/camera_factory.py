from pathlib import Path

import yaml

from .camera_interface import CameraInterface
from .mock_camera import MockCamera
from .opencv_camera import OpenCVCamera
from .picamera3 import PiCamera3


class CameraFactory:

    @staticmethod
    def create(
        config: dict
    ) -> CameraInterface:

        camera_type = (
            config
            .get("type", "")
            .lower()
        )

        width = config.get(
            "width",
            1280
        )

        height = config.get(
            "height",
            720
        )

        fps = config.get(
            "fps",
            30
        )

        if camera_type == "mock":

            return MockCamera(
                width=width,
                height=height,
                fps=fps,
            )

        if camera_type == "opencv":

            return OpenCVCamera(
                source=config.get(
                    "source",
                    0
                ),
                width=width,
                height=height,
                fps=fps,
                source_name="opencv_camera",
            )

        if camera_type == "video":

            source = config.get(
                "source"
            )

            if source is None:
                raise ValueError(
                    "Una cámara tipo video "
                    "necesita 'source'"
                )

            return OpenCVCamera(
                source=source,
                width=width,
                height=height,
                fps=fps,
                source_name="video",
            )

        if camera_type == "picamera3":

            return PiCamera3(
                width=width,
                height=height,
                fps=fps,
                pixel_format=config.get(
                    "pixel_format",
                    "RGB888"
                ),
            )

        raise ValueError(
            f"Tipo de cámara desconocido: "
            f"{camera_type}"
        )

    @classmethod
    def from_yaml(
        cls,
        path
    ) -> CameraInterface:

        path = Path(path)

        if not path.exists():
            raise FileNotFoundError(
                f"No existe: {path}"
            )

        with open(
            path,
            "r",
            encoding="utf-8"
        ) as file:

            config = yaml.safe_load(file)

        if "camera" not in config:
            raise KeyError(
                "El archivo YAML debe contener "
                "la sección 'camera'"
            )

        return cls.create(
            config["camera"]
        )