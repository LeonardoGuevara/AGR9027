"""AGR9027 Task A: Student-editable obstacle-avoidance and camera logic.

PURPOSE
Travel from point A to point B while steering around obstacles. The assessment
requires completing the route without hitting obstacles; see the official
assessment brief for its full success criteria.

STUDENT-EDITABLE FUNCTIONS
robot_control(tbot): called repeatedly while the simulation runs. The default
reads the ultrasonic distance, turns right when an obstacle is closer than
AVOID_DISTANCE_CM, and otherwise drives forward. It sets the LEDs red while
turning and green while driving. This is only a simple reactive example; it
does not steer toward point B or use camera detections.
process_camera_frame(image): a separate webcam/display example. The default
converts the BGR image to HSV, selects red pixels, finds connected contours,
draws outlines around sufficiently large blobs, and returns a debug image and
a text summary. It does not control the robot.

INPUTS AND OUTPUTS
The simulator supplies a tbot object to robot_control. Inside that function,
read ultrasonic distance in centimetres with tbot.read_distance(); obtain a
simulated camera image with tbot.get_camera_image() if you want to use vision.
You can pass that image to a helper such as process_camera_frame, or write a
detector that returns useful features (for example obstacle image position,
or apparent size) and use those features to choose movement.
process_camera_frame accepts an OpenCV BGR image and returns
(debug_image, summary). The returned values are not automatically passed into
robot_control.

ASSESSMENT GUIDANCE
Use the distance reading to detect nearby obstacles and/or image processing
to detect obstacles in the camera view. Combine the sensor evidence to choose
safe forward motion and turns that still make progress from A to B. Reactive
obstacle avoidance alone is not enough: after turning around an obstacle, the
robot must know which way to travel to continue toward B. Keep track of the
robot's estimated position and heading relative to A so it can continually
estimate the direction and distance to the known goal B.

This is a localization/odometry problem. The Trilobot library does not provide
wheel-encoder or odometry readings, so students must implement their own
approximate motion estimate. For example, estimate each wheel's travel from
the motor commands, elapsed time, and experimentally calibrated wheel speeds;
use the wheel spacing to estimate changes in heading, then update the robot's
position. Such an estimate will accumulate error, so consider how to limit or
correct drift. Use the estimated position and heading to steer back toward B
after each avoidance manoeuvre. The simulator's fake tbot also does not
provide odometry.

NOTES
Edit this file rather than simulator_task_a.py. The simulated hardware and
camera are approximations; test the final solution on the physical robot.
"""

import cv2

RED = (255, 0, 0)
GREEN = (0, 255, 0)
AVOID_DISTANCE_CM = 35


def robot_control(tbot):
    """Make one simple movement decision from the front distance reading.

    The simulator calls this repeatedly. The example compares the nearest
    ultrasonic return with AVOID_DISTANCE_CM: a close return triggers a right
    turn and red LEDs; otherwise the robot moves forward with green LEDs.

    For the assessment, replace or extend this rule so the robot reaches B
    while avoiding obstacles. Reactive turns alone do not guide it back toward
    B: maintain an estimate of position and heading relative to A, and use it
    to navigate toward the known goal after avoiding an obstacle. The Trilobot
    library does not provide wheel-encoder or odometry readings, so implement
    an approximate estimate from calibrated motor commands, elapsed time and
    wheel spacing, and account for accumulated error. You can use
    tbot.read_distance() (centimetres), and obtain tbot.get_camera_image() for
    image-based obstacle detections.
    """
    distance = tbot.read_distance()
    if distance < AVOID_DISTANCE_CM:
        tbot.fill_underlighting(RED)
        tbot.turn_right(0.6)
    else:
        tbot.fill_underlighting(GREEN)
        tbot.forward(0.6)


def process_camera_frame(image):
    """Find red obstacle-like blobs in a webcam BGR image for display.

    Converts the image to HSV, thresholds two red hue ranges, groups the
    selected pixels into contours, ignores small contours, and draws the
    remaining contours in yellow. Returns (debug_image, summary). This helper
    is not called by robot_control automatically; students can adapt it or
    create a detector that returns obstacle features for movement decisions.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 100, 70), (10, 255, 255))
    mask |= cv2.inRange(hsv, (170, 100, 70), (180, 255, 255))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    debug_image = image.copy()
    detections = [contour for contour in contours if cv2.contourArea(contour) >= 80]
    cv2.drawContours(debug_image, detections, -1, (0, 255, 255), 2)
    return debug_image, f"Red obstacle-like blobs: {len(detections)}"
