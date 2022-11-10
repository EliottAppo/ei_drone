import numpy as np
import cv2


def segments(img, lines, color=(255, 0, 0), thickness=2):
    """
    Draws segments from line data in image img
        x1, y1, x2, y2, a, b, c, n = line
    """

    for line in lines:
        x1, y1, x2, y2, _, _, _, _ = line
        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)


def lines(img, lines, color=(255, 0, 0), thickness=2):
    """
    Draws lines (not only segments) from line data in image img
        x1, y1, x2, y2, a, b, c, n = line
    """

    rows, cols, _ = img.shape
    for line in lines:
        _, _, _, _, a, b, c, _ = line

        x_top = -c/a  # x value when y=0
        x_top = (int(x_top), 0) if (x_top >= 0 and x_top <=
                                    cols) else None  # checks if point is inside frame

        x_bottom = -b/a*rows - c/a  # x value when y=rows
        x_bottom = (int(x_bottom), rows) if (
            x_bottom >= 0 and x_bottom <= cols) else None  # checks if point is inside frame

        y_left = -c/b  # y value when x=0
        y_left = (0, int(y_left)) if (y_left >= 0 and y_left <=
                                      rows) else None  # checks if point is inside frame

        y_right = -a/b*cols - c/b  # y value when w=cols
        y_right = (cols, int(y_right)) if (y_right >= 0 and y_right <=
                                           rows) else None  # checks if point is inside frame

        points = [x_top, x_bottom, y_left, y_right]
        # filters out points outside of picture
        points = list(filter(None, points))

        cv2.line(img, points[0], points[1], color, thickness)


def function(
        img,
        Y,                   # The values, len(Y) = img.shape[1]
        # The curve is plotted between image lines hmin and hmax.
        hmin, hmax,
        # The value range [ymin, ymax] is mapped into [hmin, hmax].
        ymin, ymax,
        color, thickness):
    """
    Draw function Y in image img
    """

    # satisfying ymin and ymax conditions
    Y = np.where(Y < ymin, ymin, Y)
    Y = np.where(Y > ymax, ymax, Y)

    # satisfying hmin and hmax conditions
    scale = (hmax - hmin)/(ymax - ymin + 1e-5)
    Y = scale*Y
    Y = Y + (hmin - scale*ymin)

    X = np.arange(len(Y))
    X = np.expand_dims(X, axis=-1)
    Y = np.expand_dims(Y, axis=-1)

    pts = np.concatenate([X, Y], axis=-1)
    pts = pts.astype(np.int32)

    cv2.polylines(img, [pts], isClosed=False, color=color, thickness=thickness)
