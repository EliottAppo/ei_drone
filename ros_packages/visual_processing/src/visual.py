import numpy as np
import cv2
from visual_processing_lib import processing, draw
from scipy.ndimage import median_filter


MIN_LEN = 20
MIN_ANGLE = 15
MAX_ANGLE = 75
SCI_VALUE = 0.3
MIN_DIST = 3


def get_vanish_point(img,
                     min_len=MIN_LEN,
                     min_angle=MIN_ANGLE,
                     max_angle=MAX_ANGLE,
                     sci_value=SCI_VALUE,
                     min_dist=MIN_DIST):
    """
    Get vanish point from `img`
    """
    lines = processing.detect_lines(img)

    if lines is not None:
        # filtering lines
        lines = processing.longer_than(lines, min_len)
        lines = processing.angle_inclination_between(
            lines, max_angle, min_angle)

        # calculating intersection
        intersects = processing.intersect(lines)

        if len(intersects) > 1:

            # smallest confidence interval
            sci_x = processing.sci(intersects[:, 0], sci_value)
            sci_y = processing.sci(intersects[:, 1], sci_value)

            # determining vanishing point
            vp_x = int(np.mean(sci_x))
            vp_y = int(np.mean(sci_y))

            # filtering lines far away from vanishing point
            lines = processing.distance_to_vp_lower_than(
                lines, (vp_x, vp_y), min_dist)

            # spliting lines between right and left lines
            right_lines, left_lines = processing.split_right_left(lines)

            # calculating right and left angles
            right_angle = processing.calculate_angle_from_lines(right_lines)
            left_angle = processing.calculate_angle_from_lines(left_lines)

            vanish_x, vanish_y = None, None

            if not(np.isnan(right_angle) or np.isnan(left_angle)):
                vanish_x, vanish_y = vp_x, vp_y

            return vanish_x, vanish_y, right_lines, left_lines, right_angle, left_angle

    return None, None, None, None, None, None


def draw_vanish_point(img, vanish_x, vanish_y, right_lines, left_lines):
    """
    Draw vanish point from `img`
    """
    if right_lines is not None and left_lines is not None:
        draw.lines(img, right_lines, color=(0, 255, 255),
                   thickness=2)  # drawing right lines
        draw.lines(img, left_lines, color=(0, 255, 0),
                   thickness=2)  # drawing left lines

    if vanish_x is not None and vanish_y is not None:
        cv2.circle(img, (vanish_x, vanish_y), radius=4, color=(
            255, 0, 0), thickness=2)  # drawing vanishing point


def check_if_door_exists(img,
                         prev_Y,
                         to_left,
                         half_width,
                         sigma,
                         rho,
                         median_filter_window,
                         maximum_shifts_mean,
                         maximum_shifts_std,
                         minimum_Y_std,
                         debug_drawings=False):
    """
        check if door exists in `img`
        `debug_drawings` shows drawings on image when True 
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    h = img.shape[0]
    r = (h+1)//2

    # considering the central strip of the image
    Y = gray[r-half_width:r+half_width, :]
    Y = np.mean(Y, axis=0)

    draw.function(
        img,
        Y,
        hmin=int(2*h/3), hmax=int(h/3),
        ymin=0, ymax=255,
        color=(255, 0, 0), thickness=1)

    if prev_Y is None:
        return False, Y

    draw.function(
        img,
        prev_Y,
        hmin=int(2*h/3), hmax=int(h/3),
        ymin=0, ymax=255,
        color=(0, 0, 255), thickness=1)

    # calculating shifts
    shifts = processing.get_shifts(
        Y, prev_Y, to_left, sigma, rho)
    shifts = median_filter(shifts, median_filter_window)

    draw.function(
        img,
        shifts,
        hmin=int(2*h/3), hmax=int(h/3),
        ymin=shifts.min(), ymax=shifts.max(),
        color=(0, 255, 0), thickness=1)

    # dividing shifts in 3 regions (left, center, right)
    s = len(shifts)
    shifts1 = shifts[:s//3]
    shifts2 = shifts[s//3:2*s//3]
    shifts3 = shifts[2*s//3:]

    # dividing Y curve in 3 regions (left, center, right)
    l = len(Y)
    Y1 = Y[:l//3]
    Y2 = Y[l//3:2*l//3]
    Y3 = Y[2*l//3:]

    overlay = img.copy()

    door_exists = False
    # detecting door in left region
    if np.mean(shifts1) < maximum_shifts_mean and \
            np.std(shifts1) < maximum_shifts_std and \
            np.std(Y1) > minimum_Y_std:

        if debug_drawings:
            cv2.rectangle(
                overlay, (0, img.shape[0]), (s//3, 0), (0, 0, 255), -1)
        if to_left:
            # return True, Y
            door_exists = True

    # detecting door in center region
    if np.mean(shifts2) < maximum_shifts_mean and \
            np.std(shifts2) < maximum_shifts_std and \
            np.std(Y2) > minimum_Y_std:

        if debug_drawings:
            cv2.rectangle(
                overlay, (s//3, img.shape[0]), (2*s//3, 0), (0, 0, 255), -1)
        # return True, Y
        door_exists = True

    # detecting door in right region
    if np.mean(shifts3) < maximum_shifts_mean and \
            np.std(shifts3) < maximum_shifts_std and \
            np.std(Y3) > minimum_Y_std:

        if debug_drawings:
            cv2.rectangle(
                overlay, (2*s//3, img.shape[0]), (img.shape[1], 0), (0, 0, 255), -1)
        if not to_left:
            # return True, Y
            door_exists = True

    # indicating presence of door
    alpha = 0.3
    if debug_drawings:
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

    return door_exists, Y
    # return False, Y
