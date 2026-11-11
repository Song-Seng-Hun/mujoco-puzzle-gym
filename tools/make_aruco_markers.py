"""Write the 50 DICT_4X4_50 marker PNGs into the package (R26, Open question 6 (a)).

    uv run python tools/make_aruco_markers.py

The bit table lives in `mujoco_puzzle_gym.markers.BITS` (copied from OpenCV
4.12); no OpenCV is needed. Each PNG is 8 x 8 cells of 16 px: a white margin,
the black border and the 4 x 4 bits, as `cv2.aruco.generateImageMarker` draws
them plus the margin.
"""

from __future__ import annotations

import imageio.v3 as iio

from mujoco_puzzle_gym import markers


def main() -> None:
    markers.PNG_DIR.mkdir(parents=True, exist_ok=True)
    for i in range(markers.N_IDS):
        iio.imwrite(markers.png_path(i), markers.marker_image(i))
    print(f"wrote {markers.N_IDS} PNGs to {markers.PNG_DIR}")


if __name__ == "__main__":
    main()
