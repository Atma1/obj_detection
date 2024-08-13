import cv2
import math

WHITE = (255, 255, 255)
RED = (0, 0, 255)


def center_crop(img, dim):
    """Returns center cropped image
    Args:
    img: image to be center cropped
    dim: dimensions (width, height) to be cropped
    """
    width, height = img.shape[1], img.shape[0]

# process crop width and height for max available dimension
    crop_width = dim[0] if dim[0] < img.shape[1] else img.shape[1]
    crop_height = dim[1] if dim[1] < img.shape[0] else img.shape[0]
    mid_x, mid_y = int(width / 2), int(height / 2)
    cw2, ch2 = int(crop_width / 2), int(crop_height / 2)
    crop_img = img[mid_y - ch2:mid_y + ch2, mid_x - cw2:mid_x + cw2]
    return crop_img


def get_degree(img_wh: tuple[int, int], bbox_xy: list[int]) -> float:
    """
      bbox_xy: [x1,y1,x2,y2]
    """
    img_w_c, img_h_c = img_wh[0] // 2, img_wh[1] // 2
    bbox_x_center = (bbox_xy[0] + bbox_xy[2]) // 2
    bbox_y = bbox_xy[3]

    delta_x = img_w_c - bbox_x_center
    delta_y = img_h_c - bbox_y

    radian = math.atan2(delta_y, delta_x)
    degree = round(math.degrees(radian), ndigits=2)
    print(degree)
    return degree


def convert_to_cm(pixel: float, factor: int = 5) -> float:
    """
    Convert pixel to cm
    """

    cm_pixel_ratio = 0.02645833 / 1
    factored_px = pixel * factor

    converted_to_cm = factored_px * cm_pixel_ratio

    return round(converted_to_cm, ndigits=2)


def get_distance(img_wh: tuple[int, int], bbox_xy: list[int]) -> float:
    """
    Get distance between two points
    Args:

    img_wh: [w, h]
    bbox_xy: [x1,y1,x2,y2]
    """
    img_w_c, img_h_c = img_wh[0] // 2, img_wh[1] // 2
    bbox_x_center = (bbox_xy[0] + bbox_xy[2]) // 2
    bbox_y = bbox_xy[3]

    delta_x = img_w_c - bbox_x_center
    delta_y = img_h_c - bbox_y
    distance = math.sqrt(delta_x**2 + delta_y**2)
    distance = round(distance, ndigits=2)

    return distance


def draw_helper_line(img, img_w: int, img_h: int) -> None:
    """
    Draw visualization helper line
    """
    img_w_c, img_h_c = img_w // 2, img_h // 2
    cv2.line(img, pt1=(img_w_c, 0), pt2=(img_w_c, img_h), color=WHITE)
    cv2.line(img, pt1=(0, img_h_c), pt2=(img_w, img_h_c), color=WHITE)


def draw_line_bounding_box(img, img_hw: list[int], bbox_xy: list[int]) -> None:
    """
    Draw a line from center of image to center of bounding box
    Args:

    img_hw: [x_height, y_width]
    bbox_xy: [x1,y1,x2,y2]
    """
    img_w_c, img_h_c = img_hw[0] // 2, img_hw[1] // 2
    bbox_x_center = (bbox_xy[0] + bbox_xy[2]) // 2
    cv2.line(img, pt1=(img_w_c, img_h_c), pt2=(bbox_x_center, bbox_xy[3]),
             color=RED, thickness=2)
