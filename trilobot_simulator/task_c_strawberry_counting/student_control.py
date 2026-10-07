"""AGR9027 Task C: Student-editable strawberry-counting and camera logic.

PURPOSE
Count ripe strawberries visible in a camera image, distinguishing ripe fruit
from semi-ripe and unripe fruit. The assessment allows an acceptable margin of
error; see the official assessment brief for its precise criteria.

STUDENT-EDITABLE FUNCTIONS
count_ripe_strawberries(image): called by the simulator with each scene image.
The default selects red HSV pixels, cleans small mask noise, counts large
connected contours, and draws a circle around each counted contour. This is a
starter method: touching/overlapping fruit can merge into one contour, and
semi-ripe fruit may contain red pixels.
process_camera_frame(image): webcam/display adapter. It calls the counting
function and returns a debug image with a short count summary.

INPUTS AND OUTPUTS
Both functions accept an OpenCV BGR image. The counting function returns
(count, debug_image); the webcam adapter returns (debug_image, summary).
There is no ultrasound distance input for this task: use image processing to
detect ripe fruits, then count the separate fruit instances rather than merely
counting red pixels or connected colour regions.

ASSESSMENT GUIDANCE
Improve the image-processing approach to identify which visible fruits are
ripe and estimate their number, including when fruits overlap. Consider colour,
shape, size, and methods for separating touching detections. Draw useful
detection marks on debug_image so you can compare the algorithm's output with
the image.

NOTES
Edit this file rather than simulator_task_c.py. The synthetic scene is a
practice aid; validate the counting logic with real strawberry images.
"""

import cv2
import numpy as np

MIN_CONTOUR_AREA = 80


def count_ripe_strawberries(image):
    """Count red connected regions as a simple ripe-fruit baseline.

    Converts the BGR image to HSV, combines two red hue masks, removes small
    isolated pixels, and treats each sufficiently large external contour as
    one fruit. It draws a blue enclosing circle for each counted contour and
    returns (count, debug_image). Merged contours can undercount overlapping
    berries, while red areas on semi-ripe fruit can cause false detections;
    improve detection and instance separation for the assessment.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_lower = cv2.inRange(hsv, (0, 90, 90), (10, 255, 255))
    mask_upper = cv2.inRange(hsv, (170, 90, 90), (180, 255, 255))
    mask = cv2.bitwise_or(mask_lower, mask_upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    debug_image = image.copy()
    count = 0
    for contour in contours:
        if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
            continue
        (x, y), radius = cv2.minEnclosingCircle(contour)
        cv2.circle(debug_image, (int(x), int(y)), int(radius), (255, 0, 0), 2)
        count += 1
    return count, debug_image


def process_camera_frame(image):
    """Adapt the counter to the webcam harness display interface.

    Calls count_ripe_strawberries and returns its debug image with a text
    estimate. The simulator calls the counting function directly; this adapter
    is used to display results from laptop webcam frames.
    """
    count, debug_image = count_ripe_strawberries(image)
    return debug_image, f"Estimated ripe strawberries: {count}"
