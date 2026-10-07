"""AGR9027 Task B: Student-editable person-following and camera logic.

PURPOSE
Follow the specified target person while ignoring decoys. The assessment
objective is to keep a safe, approximately fixed distance (0.5 m) from the
target; consult the official assessment brief for full success criteria.

STUDENT-EDITABLE FUNCTIONS
locate_colour(image, colour_name): the starter vision helper. It thresholds
the selected clothing colour in HSV and returns whether colour pixels were
found, their average horizontal image position, and their total pixel area.
robot_control(tbot, target_color): called repeatedly by the simulator. The
default ignores both inputs and simply drives forward at full normalized
command; it is intentionally not a completed person-following solution.
process_camera_frame(image, target_color): a separate webcam/display example.
It calls locate_colour, draws a vertical line at the detected colour's average
horizontal position, and returns a debug image and a short summary.

INPUTS AND OUTPUTS
The simulator supplies tbot and the target clothing-colour name to
robot_control. Inside it, read the ultrasonic distance in centimetres with
tbot.read_distance(); get a camera frame using tbot.get_camera_image() and
pass it to locate_colour or another person detector. Use the detected person's
image position to steer and the distance reading to manage following distance.
locate_colour accepts a BGR image and a colour name and returns
(found, centre_x, blob_area). process_camera_frame accepts the same inputs and
returns (debug_image, summary). Neither helper's output is automatically fed
into robot_control.

ASSESSMENT GUIDANCE
Use vision to find the person wearing target_color and distinguish that target
from decoys, including similarly coloured decoys. Combine the person's image
position with ultrasonic distance information to steer toward the target and
maintain the required safe following distance rather than continually driving
forward. Do not rely only on independent detections in each frame: keep track
of the target across successive camera frames, using its previous position and
movement to help distinguish it from decoys and recover when a detection is
missed. The ultrasonic sensor reports distance to the nearest person in its
forward sensing cone, so combine that reading with the target track to judge
whether the distance measurement likely belongs to the target.

NOTES
Edit this file rather than simulator_task_b.py. The simulated hardware and
camera are approximations; test the final solution on the physical robot.
"""

import cv2
import numpy as np

HSV_RANGES = {
    "RED": [((0, 80, 80), (10, 255, 255)), ((170, 80, 80), (180, 255, 255))],
    "YELLOW": [((15, 80, 80), (36, 255, 255))],
    "GREEN": [((36, 80, 80), (70, 255, 255))],
    "BLUE": [((100, 80, 80), (135, 255, 255))],
}


def locate_colour(image, colour_name):
    """Estimate where target-coloured pixels appear in a BGR camera frame.

    The HSV ranges below define the clothing colours. This starter version
    combines every matching pixel into one mask and uses image moments to
    estimate its average horizontal position and total area. It returns
    (found, centre_x, blob_area); it does not yet separate people or resolve
    same-colour decoys, so students may improve it for the assessment.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in HSV_RANGES[colour_name]:
        mask |= cv2.inRange(hsv, lower, upper)
    moments = cv2.moments(mask)
    if moments["m00"] > 0:
        return True, moments["m10"] / moments["m00"], moments["m00"]
    return False, image.shape[1] / 2, 0


def robot_control(tbot, target_color):
    """Make one movement decision; the current starter simply drives forward.

    The simulator calls this repeatedly and supplies the robot interface plus
    the target clothing-colour name. The default does not inspect either input.
    For the assessment, read tbot.read_distance() and obtain a camera frame
    with tbot.get_camera_image(); use target_color and image-processing results
    (for example target position/size) to follow the correct person while
    maintaining a safe distance from them. Keep target identity across frames
    rather than choosing a person from colour detection alone each time; use
    position and motion continuity to reduce switches to similarly coloured
    decoys. The distance sensor sees the nearest person in its forward cone, not
    necessarily the target, so interpret it alongside the visual track.
    """
    tbot.forward(1.0)


def process_camera_frame(image, target_color):
    """Show the starter colour detector's result on a webcam BGR frame.

    Calls locate_colour, draws a vertical guide at the detected pixels'
    horizontal average, and returns (debug_image, summary). This display
    adapter is not called by robot_control automatically.
    """
    found, cx, area = locate_colour(image, target_color)
    debug_image = image.copy()
    if found:
        cv2.line(debug_image, (int(cx), 0), (int(cx), image.shape[0] - 1), (0, 255, 255), 2)
    summary = f"{target_color} clothing pixels: {int(area)}" if found else f"{target_color} target not found"
    return debug_image, summary
