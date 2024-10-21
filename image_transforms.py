import cv2
import numpy as np


CAMERAMTX = np.array([[955.00503025, 0, 945.90679043], [0, 928.24453561, 602.90539444], [0, 0, 1]])
ROI = (250, 130, 1220, 950)
MATRIX = np.array([[1149.98429, 0, 939.584851], [0, 1147.62943, 555.691984], [0, 0, 1]])
DISTORTION = np.array([[-0.35772729,  0.15165342, -0.00318789, -0.00075705, -0.03185217]])
template_h = cv2.imread('res\\images\\patterns\\pattern_hor.png', cv2.IMREAD_GRAYSCALE)
template_v = cv2.imread('res\\images\\patterns\\pattern_vert.png', cv2.IMREAD_GRAYSCALE)
size_grid = (120, 95)
size_img = (1220, 950)


def max_pooling(image: cv2.typing.MatLike, kernel_size: int) -> cv2.typing.MatLike:
    height, width = image.shape
    out_height = height // kernel_size
    out_width = width // kernel_size
    reshaped_image = image.reshape(out_height, kernel_size, out_width, kernel_size)
    pooled_image = reshaped_image.max(axis=(1, 3))
    return pooled_image


def make_grid(image: cv2.typing.MatLike) -> cv2.typing.MatLike:
    image = np.max(image, axis=-1)
    _, image = cv2.threshold(image, 255, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((2, 2))
    image = cv2.erode(image, kernel, iterations=3)
    image = cv2.dilate(image, kernel, iterations=3)
    grid = max_pooling(image, 10)
    grid = cv2.dilate(grid, np.ones((2, 2)))
    return grid


def unfish(image: cv2.typing.MatLike):
    image = cv2.undistort(image, MATRIX, DISTORTION, None, CAMERAMTX)
    x, y, w, h = ROI
    image = image[y:y+h, x:x+w]
    return image


def match_template(test_image):
    result1 = cv2.matchTemplate(test_image, template_h, cv2.TM_CCOEFF_NORMED)
    min_val1, max_val1, min_loc1, max_loc1 = cv2.minMaxLoc(result1)
    result2 = cv2.matchTemplate(test_image, template_v, cv2.TM_CCOEFF_NORMED)
    min_val2, max_val2, min_loc2, max_loc2 = cv2.minMaxLoc(result2)
    if max_val1 > max_val2:
        best_match = template_h
    else:
        best_match = template_v
    return best_match


def transform_coordinates(x: float, y: float):
    return int(x * size_grid[0] / size_img[0]), int(y * size_grid[1] / size_img[1])