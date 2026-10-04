# =====================================================================================
# AGR9027 - Task B simulator: Person following
# =====================================================================================
# This file lets you test your Task B code on your own laptop, without the real
# Trilobot. It creates a fake robot object called "tbot" with the same methods used
# in the example scripts in /scripts (forward, turn_left, set_motor_speeds,
# read_distance, fill_underlighting, ...) plus tbot.get_camera_image(), which returns
# a numpy BGR image you can feed into the same OpenCV code used in
# scripts/trilobot/ball_tracking.py and scripts/trilobot/color_detection.py.
#
# SCENARIO
# --------
# A number of "people" (printed photos in the real assessment) are standing in front
# of the robot. Each one is drawn as a coloured rectangle in the simulated camera
# image. Exactly ONE of them is your TARGET (its colour is printed in the console and
# shown with a white border in the pygame window) - the others are decoys. The target
# wanders around slowly. Your job is to keep the robot at about 0.5 m from the target,
# even when decoys of a similar colour are also visible - exactly like the real task.
#
# HOW TO USE THIS FILE
# ---------------------
# 1. Install the requirements: pip install -r ../requirements.txt
# 2. Run this file directly: python simulator_task_b.py
# 3. Edit ONLY the function `robot_control(tbot, target_color)` below. It already
#    contains a simple working example that tracks the target using colour detection
#    (like scripts/trilobot/ball_tracking.py) and keeps a fixed distance (like
#    scripts/follow_straight.py). Replace/extend it with your own Task B algorithm.
# 4. Increase NUM_DECOYS / SIMILAR_DECOY_COLOURS for the harder levels described in
#    the assessment brief.
# =====================================================================================

import math
import random
import sys

import cv2
import numpy as np
import pygame

# --------------------------------------------------------------------------- settings
SCALE = 120
ARENA_W_M = 6.0
ARENA_H_M = 4.0
NUM_DECOYS = 2                 # extra, non-target "people" sharing the scene
SIMILAR_DECOY_COLOURS = False  # set True for the harder "similar clothes colour" level
GOAL_DISTANCE_M = 0.5          # the robot must keep ~this distance from the target
ROBOT_RADIUS_M = 0.12
PERSON_RADIUS_M = 0.22
MAX_SPEED_MPS = 0.35
WHEEL_BASE_M = 0.16
SENSOR_MAX_RANGE_CM = 300.0
SENSOR_FOV_DEG = 20
CAMERA_FOV_DEG = 70
CAMERA_SIZE = (320, 240)
FPS = 30
TARGET_WALK_SPEED_MPS = 0.15

RED = (255, 0, 0)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
BUTTON_A = "A"

COLOUR_NAMES = {
    "RED": (255, 0, 0),
    "YELLOW": (255, 255, 0),
    "GREEN": (0, 255, 0),
    "BLUE": (0, 0, 255),
}
# HSV ranges matching scripts/trilobot/color_detection.py and scripts/trilobot/ball_tracking.py
HSV_RANGES = {
    "RED": [((0, 80, 80), (10, 255, 255)), ((170, 80, 80), (180, 255, 255))],
    "YELLOW": [((15, 80, 80), (36, 255, 255))],
    "GREEN": [((36, 80, 80), (70, 255, 255))],
    "BLUE": [((100, 80, 80), (135, 255, 255))],
}


# --------------------------------------------------------------------------- the world
class Person:
    def __init__(self, x, y, colour_name, is_target):
        self.x = x
        self.y = y
        self.colour_name = colour_name
        self.colour = COLOUR_NAMES[colour_name]
        self.is_target = is_target
        self.walk_angle = random.uniform(0, 2 * math.pi)

    def wander(self, dt):
        if not self.is_target:
            return
        self.walk_angle += random.uniform(-0.3, 0.3)
        self.x += math.cos(self.walk_angle) * TARGET_WALK_SPEED_MPS * dt
        self.y += math.sin(self.walk_angle) * TARGET_WALK_SPEED_MPS * dt
        self.x = max(0.5, min(ARENA_W_M - 0.5, self.x))
        self.y = max(0.5, min(ARENA_H_M - 0.5, self.y))


def make_world():
    all_colours = list(COLOUR_NAMES.keys())
    target_colour = random.choice(all_colours)
    target = Person(ARENA_W_M - 1.0, ARENA_H_M / 2, target_colour, is_target=True)

    decoys = []
    other_colours = all_colours if SIMILAR_DECOY_COLOURS else [c for c in all_colours if c != target_colour]
    for _ in range(NUM_DECOYS):
        colour = target_colour if SIMILAR_DECOY_COLOURS else random.choice(other_colours)
        x = random.uniform(2.5, ARENA_W_M - 0.8)
        y = random.uniform(0.5, ARENA_H_M - 0.5)
        decoys.append(Person(x, y, colour, is_target=False))

    robot_start = (0.6, ARENA_H_M / 2)
    return robot_start, target, decoys


# --------------------------------------------------------------------- the fake robot
class SimTrilobot:
    """A drop-in, simplified stand-in for the real `trilobot.Trilobot` class."""

    def __init__(self, robot_start, people):
        self.x, self.y = robot_start
        self.theta = 0.0
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.people = people  # list including target + decoys
        self.underlight_colour = BLACK
        self.quit_requested = False

    def set_motor_speeds(self, left, right):
        self.left_speed = max(-1.0, min(1.0, left))
        self.right_speed = max(-1.0, min(1.0, right))

    def forward(self, speed=1.0):
        self.set_motor_speeds(speed, speed)

    def backward(self, speed=1.0):
        self.set_motor_speeds(-speed, -speed)

    def turn_left(self, speed=1.0):
        self.set_motor_speeds(-speed, speed)

    def turn_right(self, speed=1.0):
        self.set_motor_speeds(speed, -speed)

    def coast(self):
        self.set_motor_speeds(0.0, 0.0)

    def stop(self):
        self.set_motor_speeds(0.0, 0.0)

    def disable_motors(self):
        self.set_motor_speeds(0.0, 0.0)

    def read_button(self, button):
        return self.quit_requested

    def fill_underlighting(self, colour):
        self.underlight_colour = colour

    def read_distance(self, timeout=50, samples=3):
        """Simulated ultrasound: distance (cm) to the closest person in a forward cone."""
        best = SENSOR_MAX_RANGE_CM
        half_fov = math.radians(SENSOR_FOV_DEG / 2)
        for person in self.people:
            dx, dy = person.x - self.x, person.y - self.y
            dist = math.hypot(dx, dy)
            angle = math.atan2(dy, dx) - self.theta
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_fov:
                best = min(best, max(0.0, dist - PERSON_RADIUS_M) * 100)
        noise = random.uniform(-1.0, 1.0)
        return max(0.0, min(SENSOR_MAX_RANGE_CM, best + noise))

    def get_camera_image(self):
        """Simulated camera: returns a BGR image with each person drawn as a
        filled, coloured rectangle - treat these like the printed photos in the
        real assessment. Use HSV colour masks (as in scripts/trilobot/color_detection.py)
        to find them."""
        w, h = CAMERA_SIZE
        image = np.zeros((h, w, 3), dtype=np.uint8)
        half_fov = math.radians(CAMERA_FOV_DEG / 2)
        visible = []
        for person in self.people:
            dx, dy = person.x - self.x, person.y - self.y
            dist = math.hypot(dx, dy)
            angle = math.atan2(dy, dx) - self.theta
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if dist < 0.05 or abs(angle) > half_fov:
                continue
            visible.append((dist, angle, person))
        for dist, angle, person in sorted(visible, key=lambda t: -t[0]):
            u = int(w / 2 + (angle / half_fov) * (w / 2))
            v = int(h / 2)
            half_size = int(max(6, min(w / 3, (PERSON_RADIUS_M * 110) / max(dist, 0.1))))
            b, g, r = person.colour[2], person.colour[1], person.colour[0]
            cv2.rectangle(image, (u - half_size, v - half_size), (u + half_size, v + half_size), (b, g, r), -1)
        return image

    def _update(self, dt):
        v = (self.left_speed + self.right_speed) / 2.0 * MAX_SPEED_MPS
        w = (self.right_speed - self.left_speed) / WHEEL_BASE_M * MAX_SPEED_MPS
        self.theta += w * dt
        new_x = self.x + v * math.cos(self.theta) * dt
        new_y = self.y + v * math.sin(self.theta) * dt
        self.x = max(ROBOT_RADIUS_M, min(ARENA_W_M - ROBOT_RADIUS_M, new_x))
        self.y = max(ROBOT_RADIUS_M, min(ARENA_H_M - ROBOT_RADIUS_M, new_y))


# ------------------------------------------------------------------ helper for OpenCV
def locate_colour(image, colour_name):
    """Returns (found, centre_x, blob_area) for the given colour name, same approach
    as the moments-based detection used in scripts/trilobot/color_detection.py."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in HSV_RANGES[colour_name]:
        mask |= cv2.inRange(hsv, lower, upper)
    moments = cv2.moments(mask)
    if moments["m00"] > 0:
        cx = moments["m10"] / moments["m00"]
        return True, cx, moments["m00"]
    return False, image.shape[1] / 2, 0


# =====================================================================================
#  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EDIT FROM HERE  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# =====================================================================================
# `target_color` is one of "RED", "YELLOW", "GREEN", "BLUE" - it tells you the colour
# of the target you must follow (on the real robot you would decide this yourself,
# e.g. by asking the user, or by remembering the colour seen at the start).
# The example below: locate the target colour in the camera image, turn towards it,
# and use the simulated ultrasound distance to keep ~0.5 m away from the nearest
# person in front of the robot.


def robot_control(tbot, target_color):
    image = tbot.get_camera_image()
    found, cx, area = locate_colour(image, target_color)
    width = image.shape[1]

    if not found:
        tbot.fill_underlighting(BLACK)
        tbot.disable_motors()
        return

    tbot.fill_underlighting(COLOUR_NAMES[target_color])

    # steer so that the target stays centred in the image
    err_x = cx - width / 2
    turn = -float(err_x) / (width / 2)  # -1..1, negative = too far right

    # keep the goal distance using the ultrasound sensor
    distance_cm = tbot.read_distance()
    goal_cm = GOAL_DISTANCE_M * 100
    drive = max(-0.6, min(0.6, (distance_cm - goal_cm) / 100))

    left = max(-1.0, min(1.0, drive - 0.5 * turn))
    right = max(-1.0, min(1.0, drive + 0.5 * turn))
    tbot.set_motor_speeds(left, right)


# =====================================================================================
#  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EDIT UNTIL HERE  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# =====================================================================================


# --------------------------------------------------------------------------- rendering
def to_px(pos):
    return int(pos[0] * SCALE), int(pos[1] * SCALE)


def draw_world(screen, font, tbot, target, elapsed):
    screen.fill((30, 30, 30))

    for person in tbot.people:
        colour = (person.colour[0], person.colour[1], person.colour[2])
        pygame.draw.rect(screen, colour, pygame.Rect(0, 0, int(PERSON_RADIUS_M * 2 * SCALE),
                                                       int(PERSON_RADIUS_M * 2 * SCALE)).move(
            to_px((person.x, person.y))[0] - int(PERSON_RADIUS_M * SCALE),
            to_px((person.x, person.y))[1] - int(PERSON_RADIUS_M * SCALE)))
        if person.is_target:
            pygame.draw.rect(screen, (255, 255, 255), pygame.Rect(0, 0, int(PERSON_RADIUS_M * 2 * SCALE),
                                                                    int(PERSON_RADIUS_M * 2 * SCALE)).move(
                to_px((person.x, person.y))[0] - int(PERSON_RADIUS_M * SCALE),
                to_px((person.x, person.y))[1] - int(PERSON_RADIUS_M * SCALE)), 3)

    rx, ry = to_px((tbot.x, tbot.y))
    r_px = int(ROBOT_RADIUS_M * SCALE)
    tip = (rx + r_px * math.cos(tbot.theta), ry + r_px * math.sin(tbot.theta))
    left = (rx + r_px * math.cos(tbot.theta + 2.5), ry + r_px * math.sin(tbot.theta + 2.5))
    right = (rx + r_px * math.cos(tbot.theta - 2.5), ry + r_px * math.sin(tbot.theta - 2.5))
    colour = tbot.underlight_colour if tbot.underlight_colour != BLACK else (255, 255, 255)
    pygame.draw.polygon(screen, colour, [tip, left, right])

    distance_to_target = math.hypot(tbot.x - target.x, tbot.y - target.y)
    lines = [
        f"Target colour: {target.colour_name}   Elapsed: {elapsed:4.1f}s",
        f"Distance to target: {distance_to_target:4.2f} m  (goal: {GOAL_DISTANCE_M} m)",
        "Edit robot_control(tbot, target_color) in simulator_task_b.py. Press Q to quit.",
    ]
    for i, line in enumerate(lines):
        screen.blit(font.render(line, True, (230, 230, 230)), (10, 10 + 20 * i))


def main():
    random.seed()
    robot_start, target, decoys = make_world()
    people = [target] + decoys
    tbot = SimTrilobot(robot_start, people)
    print(f"Target colour to follow: {target.colour_name}")

    pygame.init()
    screen = pygame.display.set_mode((int(ARENA_W_M * SCALE), int(ARENA_H_M * SCALE)))
    pygame.display.set_caption("AGR9027 - Task B simulator: person following")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)
    start_ticks = pygame.time.get_ticks()

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key in (pygame.K_q, pygame.K_ESCAPE):
                tbot.quit_requested = True
                running = False

        robot_control(tbot, target.colour_name)
        tbot._update(dt)
        for person in tbot.people:
            person.wander(dt)

        elapsed = (pygame.time.get_ticks() - start_ticks) / 1000.0
        draw_world(screen, font, tbot, target, elapsed)
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
