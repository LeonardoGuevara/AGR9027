# This script uses the camera mounted on the Trilobot to detect and count how many
# objects of a chosen colour are visible in the image. It is a generic starting point
# for any task that requires counting objects (e.g. counting how many items of a
# certain colour are present) - you will need to modify it (e.g. the colour used,
# the size/shape filtering, how the count is displayed or used) to solve your own task.
# The count is printed on the console and shown as a simple live window.

import picamera
import cv2
import numpy
from trilobot import *

# create the robot object
tbot = Trilobot()

# Colour to count (choose one): "RED", "YELLOW", "GREEN", "BLUE"
color_wanted = "RED"

# Ignore detected blobs smaller than this (in pixels) - helps remove small/noisy specks
MIN_AREA = 80


def capture_image():
    """Captures a single still image from the Trilobot's camera and returns it as
    a BGR numpy array (the format OpenCV expects)."""
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)
        image = numpy.empty((240 * 320 * 3,), dtype=numpy.uint8)
        camera.capture(image, 'bgr')
        image = image.reshape((240, 320, 3))

    return image


def color_mask(image, color_wanted):
    """Converts the image to HSV and returns a black-and-white mask where pixels
    matching `color_wanted` are white (255) and everything else is black (0)."""
    ## convert to hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    ## mask of red
    mask_r_lower = cv2.inRange(hsv, (0, 0, 0), (10, 255, 255))
    mask_r_upper = cv2.inRange(hsv, (170, 0, 0), (180, 255, 255))
    mask_r = cv2.bitwise_or(mask_r_lower, mask_r_upper)
    ## mask of yellow
    mask_y = cv2.inRange(hsv, (15, 0, 0), (36, 255, 255))
    ## mask of green
    mask_g = cv2.inRange(hsv, (36, 0, 0), (70, 255, 255))
    ## mask of blue
    mask_b = cv2.inRange(hsv, (100, 0, 0), (135, 255, 255))

    if color_wanted == "RED":
        return mask_r
    elif color_wanted == "YELLOW":
        return mask_y
    elif color_wanted == "GREEN":
        return mask_g
    elif color_wanted == "BLUE":
        return mask_b


def count_objects(image, color_wanted):
    """Counts how many separate blobs of `color_wanted` are visible in the image.
    Builds a colour mask, cleans it up, then finds and counts contours above
    MIN_AREA. Returns the count and a copy of the image with each counted object
    circled (useful for displaying/debugging what was detected)."""
    mask = color_mask(image, color_wanted)
    # remove small noise from the mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, numpy.ones((3, 3), numpy.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    count = 0
    debug_image = image.copy()
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue
        (x, y), radius = cv2.minEnclosingCircle(contour)
        cv2.circle(debug_image, (int(x), int(y)), int(radius), (255, 0, 0), 2)
        count += 1

    return count, debug_image


while True or KeyboardInterrupt:
    image = capture_image()
    count, debug_image = count_objects(image, color_wanted)
    print("NUMBER OF {} OBJECTS DETECTED: {}".format(color_wanted, count))

    cv2.putText(debug_image, "Count: {}".format(count), (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imshow("Object counting", debug_image)
    cv2.waitKey(1)
