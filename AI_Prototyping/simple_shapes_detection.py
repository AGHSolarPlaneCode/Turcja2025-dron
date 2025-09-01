import cv2
import numpy as np


def shapes_detection(image):
    """
    Detects triangles and squares in an RGB image and marks them directly on the image.

    :param image: RGB numpy array of shape (height, width, 3) - will be modified in place
    :return: numpy array - The input image with detected shapes marked on it
    """

    result_image = image.copy()

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, 11, 2)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:

        area = cv2.contourArea(contour)
        if area < 500:
            continue

        perimeter = cv2.arcLength(contour, True)
        epsilon = 0.02 * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)

        vertices = len(approx)

        if vertices == 3:
            if _is_valid_triangle(approx, area):
                _mark_triangle(result_image, approx, contour)

        elif vertices == 4:
            if _is_square(approx, area):
                _mark_square(result_image, approx, contour)

    return result_image


def _is_valid_triangle(approx, area):
    """
    Validate if the approximated contour is a valid triangle.

    :param approx: Approximated contour points
    :param area: Contour area
    :return: bool - True if valid triangle
    """

    if not cv2.isContourConvex(approx):
        return False

    hull_area = cv2.contourArea(approx)
    if hull_area == 0:
        return False

    area_ratio = area / hull_area

    return 0.7 <= area_ratio <= 1.0


def _is_square(approx, area):
    """
    Validate if the approximated contour is a square.

    :param approx:Approximated contour points
    :param area: Contour area
    :return: bool - True if valid square
    """

    if not cv2.isContourConvex(approx):
        return False

    x, y, w, h = cv2.boundingRect(approx)

    aspect_ratio = w / h if h != 0 else 0
    if not (0.8 <= aspect_ratio <= 1.2):
        return False


    rect_area = w * h
    if rect_area == 0:
        return False

    area_ratio = area / rect_area

    if not (0.7 <= area_ratio <= 1.0):
        return False


    # return _check_right_angles(approx)
    return True


def _check_right_angles(approx):
    """
    Check if the quadrilateral has approximately right angles.

    :param approx: Approximated contour points
    :return: bool - True if angles are close to 90 degrees
    """

    if len(approx) != 4:
        return False

    points = approx.reshape(4, 2)
    angles = []

    for i in range(4):
        p1 = points[i]
        p2 = points[(i + 1) % 4]
        p3 = points[(i + 2) % 4]

        v1 = p1 - p2
        v2 = p3 - p2

        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cos_angle = np.clip(cos_angle, -1, 1)
        angle = np.arccos(cos_angle) * 180 / np.pi
        angles.append(angle)


    for angle in angles:
        if not (75 <= angle <= 105):
            return False

    return True


def _mark_triangle(image, approx, contour):
    """
    Mark a detected triangle on the image.

    :param image: Image to draw on
    :param approx: Approximated contour points
    :param contour: Original contour
    """

    cv2.drawContours(image, [approx], -1, (0, 255, 0), 3)

    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0

    cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)

    cv2.putText(image, "TRIANGLE", (cx - 40, cy - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


def _mark_square(image, approx, contour):
    """
    Mark a detected square on the image.

    :param image: Image to draw on
    :param approx: Approximated contour points
    :param contour: Original contour
    """

    cv2.drawContours(image, [approx], -1, (255, 0, 0), 3)

    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0

    cv2.circle(image, (cx, cy), 5, (255, 0, 0), -1)

    cv2.putText(image, "SQUARE", (cx - 35, cy - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)