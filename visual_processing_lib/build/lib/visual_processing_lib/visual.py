import sys
import cv2
import numpy as np


def longer_than(lines, min_len):
    """
    Filters out lines shorter than min_len

    lines: line data
    min_len: minimum length to be considered (in px)
    """

    return lines[lines[:, -1] > min_len]


def angle_inclination_between(lines, max_angle, min_angle):
    """
    Filters out lines whose inclinations (absolute values) are out of the desired interval (higher than max_angle,
    lower than min_angle)

    lines: line data
    max_angle: maximum angle to be considered (in degrees)
    min_angle: minimum angle to be considered (in degrees)
    """

    # -lines[:, 4]/lines[:, 5] -> linear coefficients

    lines = lines[
        np.logical_and(
            -lines[:, 4]/lines[:, 5] < np.tan(max_angle*np.pi/180),
            -lines[:, 4]/lines[:, 5] > np.tan(-max_angle*np.pi/180)
        )
    ]

    lines = lines[
        np.logical_or(
            -lines[:, 4]/lines[:, 5] > np.tan(min_angle*np.pi/180),
            -lines[:, 4]/lines[:, 5] < np.tan(-min_angle*np.pi/180)
        )
    ]

    return lines


def intersect(lines):
    """
    Returns intersections between a set of lines
    """

    intersections = []
    for i, si in enumerate(lines):
        for sj in lines[i+1:]:
            cross_product = np.cross(si[4:6], sj[4:6])  # [a1,b1] ^ [a2, b2]

            if cross_product != 0:
                coeff = 1.0 / cross_product

                intersections.append([coeff * np.cross(si[5:7], sj[5:7]),  # [b1, c1] ^ [b2, c2]
                                      coeff * np.cross(sj[[4, 6]], si[[4, 6]])])  # -[a1, c1] ^ [a2, c2]

    return np.array(intersections)


def detect_lines(img):
    """
    Detects the start and finish of line segments in image img
    """

    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    lsd_detector = cv2.ximgproc.createFastLineDetector()
    lsd_segments = lsd_detector.detect(gray_img)

    # if no line is detected, returns None
    if lsd_segments is None:
        return None

    lsd_segments = np.squeeze(lsd_segments, axis=1)

    # calculating coefficients a, b and c, and lengths n
    a = np.expand_dims(lsd_segments[:, 1] - lsd_segments[:, 3] + 1e-5, axis=-1)
    b = np.expand_dims(lsd_segments[:, 2] - lsd_segments[:, 0] + 1e-5, axis=-1)
    c = np.expand_dims(lsd_segments[:, 0]*lsd_segments[:, 3] -
                       lsd_segments[:, 2]*lsd_segments[:, 1], axis=-1)
    n = np.expand_dims(np.sqrt(np.power(lsd_segments[:, 0] - lsd_segments[:, 2], 2) + np.power(
        lsd_segments[:, 1] - lsd_segments[:, 3], 2)), axis=-1)

    lines = np.concatenate([lsd_segments, a, b, c, n], axis=-1)

    return lines


def sci(values, confidence):
    """
    values : an array of scalars (e.g. [1.2, 100.6, 23.78, ....])
    confidence : in [0,1], (e.g 0.5 for 50%)
    """

    nb = values.shape[0]
    values = np.sort(values)
    size = (int)(nb*confidence+.5)
    nb_iter = nb - size + 1
    sci = None
    sci_width = sys.float_info.max
    inf = 0
    sup = size
    for i in range(nb_iter):
        sciw = values[sup-1] - values[inf]
        if sciw < sci_width:
            sci = values[inf:sup]
            sci_width = sciw
        inf += 1
        sup += 1

    # The result is the array (ordered) of the values inside the sci.

    return sci


def point_to_line_distance(a, b, c, xp, yp):
    """
    Returns distance from point (xp, yp) to line described by coefficients a, b and c
    """

    return np.abs(a*xp + b*yp + c)/np.sqrt(np.power(a, 2) + np.power(b, 2))


def distance_to_vp_lower_than(lines, vp, min_dist):
    """
    Filters out lines with greater distance to the vanishing point than min_dist

    lines: line data
    vp: tuple with coordinates of the vanishing point (vp_x, vp_y)
    min_dist: minimum distance to be considered (in px)
    """

    distances = point_to_line_distance(
        lines[:, 4], lines[:, 5], lines[:, 6], vp[0], vp[1])
    return lines[distances < min_dist, :]


def split_right_left(lines):
    """
    Splits lines between right lines and left lines (based on inclination)
    """

    lin_coef = -lines[:, 4]/lines[:, 5]  # linear coefficients
    right_lines = lines[lin_coef >= 0, :]
    left_lines = lines[lin_coef < 0, :]
    return right_lines, left_lines


def calculate_angle_from_lines(lines):
    """
    Calculate mean angle (in degrees) from set of lines (line data)
    """

    a = np.mean(lines[:, 4])
    b = np.mean(lines[:, 5])
    return np.arctan(-a/b)*180/np.pi


def get_shifts(Y1, Y2, to_left, sigma, rho):
    """
    Compute local shifts between Y1 and Y2

    Shifts considered: [0, sigma]
    Local window considered: [-rho, rho]
    """

    l = len(Y1)
    Y1 = np.concatenate([np.zeros(rho), Y1, np.zeros(rho)])
    Y2 = np.concatenate([np.zeros(sigma+rho), Y2, np.zeros(sigma+rho)])

    # using np.mgrid to replace for loops
    x, s, h = np.mgrid[0:l, 0:sigma, -rho:rho]

    if to_left:  # if drone moves leftwards
        e = np.power(Y1[x+rho+h] - Y2[x+sigma+rho+h-s], 2)
    else:  # if drone moves rightwards
        e = np.power(Y1[x+rho+h] - Y2[x+sigma+rho+h+s], 2)

    e = np.sum(e, axis=-1)
    shifts = np.argmin(e, axis=-1)

    return shifts
