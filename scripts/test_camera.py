from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.camera import CameraFactory


def main():

    camera = CameraFactory.from_yaml(
        "configs/camera.yaml"
    )

    try:

        camera.open()

        print(
            "Cámara abierta:",
            camera.is_open()
        )

        while True:

            frame = camera.read()

            if frame is None:
                print(
                    "No hay más frames"
                )
                break

            print(
                f"\rFrame: {frame.frame_id} | "
                f"{frame.width}x{frame.height}",
                end=""
            )

            image = frame.image

            if frame.color_space == "RGB":
                image = cv2.cvtColor(
                    image,
                    cv2.COLOR_RGB2BGR
                )

            cv2.imshow(
                "Camera Test",
                image
            )

            key = cv2.waitKey(1)

            if key == 27:
                break

    finally:

        camera.close()

        cv2.destroyAllWindows()

        print(
            "\nCámara cerrada"
        )


if __name__ == "__main__":
    main()