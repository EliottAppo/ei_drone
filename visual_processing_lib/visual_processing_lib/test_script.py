import cv2
import numpy as np
import draw
import visual

img = cv2.imread(
    "/home/hanna/Desktop/EI-Drone-Project/mercury-ws/src/visual_processing_lib/visual_processing_lib/test.png")

lines = visual.detect_lines(img)
draw.segments(img, lines)

min_len = 20
min_angle = 15
max_angle = 75
sci_value = 0.3
min_dist = 3

if lines is not None:
# if lines is  None:

    # filtering lines
    lines = visual.longer_than(lines, min_len)
    lines = visual.angle_inclination_between(
        lines, max_angle, min_angle)

    draw.lines(img, lines, color=(0, 255, 255),
                   thickness=2)

    # calculating intersection
    intersects = visual.intersect(lines)

    # if len(intersects) > 1:
    if not len(intersects) > 1:

        # smallest confidence interval
        sci_x = visual.sci(intersects[:, 0], 0.3)
        sci_y = visual.sci(intersects[:, 1], 0.3)

        # determining vanishing point
        vp_x = int(np.mean(sci_x))
        vp_y = int(np.mean(sci_y))

        # filtering lines far away from vanishing point
        lines = visual.distance_to_vp_lower_than(
            lines, (vp_x, vp_y), min_dist)

        # spliting lines between right and left lines
        right_lines, left_lines = visual.split_right_left(lines)

        # calculating right and left angles
        right_angle = visual.calculate_angle_from_lines(right_lines)
        left_angle = visual.calculate_angle_from_lines(left_lines)

        draw.lines(img, right_lines, color=(0, 255, 255),
                   thickness=2)  # drawing right lines
        draw.lines(img, left_lines, color=(0, 255, 0),
                   thickness=2)  # drawing left lines

        if not(np.isnan(right_angle) or np.isnan(left_angle)):
            cv2.circle(img, (vp_x, vp_y), radius=4, color=(
                255, 0, 0), thickness=2)  # drawing vanishing point

cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
