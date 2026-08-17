
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from src.depth import MockToF


def main():

    sensor = MockToF(
        rows=8,
        cols=8,
        fps=15,
    )

    try:

        sensor.open()

        for _ in range(10):

            frame = sensor.read()

            print(
                "\nFrame:",
                frame.frame_id
            )

            print(
                np.round(
                    frame.distances,
                    2
                )
            )

            print(
                "Distancia mínima:",
                frame.min_distance()
            )

    finally:

        sensor.close()


if __name__ == "__main__":
    main()