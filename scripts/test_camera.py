from pathlib import Path
import sys
sys.path.append(str(Path(__file__).resolve().parents[1]))
from src.camera.opencv_camera import OpenCVCamera


def main():

    camera = OpenCVCamera(
        source=0
    )

    camera.open()

    if not camera.is_open():
        print("No se pudo abrir la cámara.")
        return

    frame = camera.read()

    if frame is None:
        print("No se pudo obtener un frame.")
        return

    print("Frame capturado")
    print("Shape:", frame.shape)
    print("Type:", frame.dtype)

    camera.close()


if __name__ == "__main__":
    main()