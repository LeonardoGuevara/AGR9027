# =====================================================================================
# AGR9027 - Task A simulator: Obstacle avoidance (point A -> point B)
# =====================================================================================
# This file lets you test your Task A code on your own laptop, without the real
# Trilobot. It creates a fake robot object called "tbot" that behaves (as closely as
# possible) like the real `Trilobot` object you get from `from trilobot import Trilobot`.
#
# The fake robot supports the same methods used in the example scripts in /scripts:
#   tbot.forward(speed), tbot.backward(speed), tbot.turn_left(speed), tbot.turn_right(speed)
#   tbot.curve_forward_left(speed), tbot.curve_forward_right(speed)
#   tbot.curve_backward_left(speed), tbot.curve_backward_right(speed)
#   tbot.set_motor_speeds(left, right)
#   tbot.coast(), tbot.stop(), tbot.disable_motors()
#   tbot.read_distance(timeout=.., samples=..)   -> distance in cm (ultrasound sensor)
#   tbot.fill_underlighting(color)
#   tbot.read_button(BUTTON_A)                   -> True once you press "q" or close the window
#
# It also provides tbot.get_camera_image() which returns a numpy BGR image (like the
# real `picamera` capture used in the scripts), showing the obstacles as red blobs in
# front of the robot. You can feed this into the same OpenCV code used in
# scripts/trilobot/color_detection.py or scripts/trilobot/ball_tracking.py.
#
# HOW TO USE THIS FILE
# ---------------------
# 1. Install the requirements (see trilobot_simulator/requirements.txt):
#       pip install -r ../requirements.txt
# 2. Run this file directly:
#       python simulator_task_a.py
# 3. Scroll down to the function `robot_control(tbot)`. That is the ONLY part of this
#    file you need to edit. It already contains a very simple working example (drive
#    forward, turn away when something is too close). Replace/extend it with your own
#    Task A algorithm.
# 4. A pygame window will open showing a top-down view of the arena: point A (green),
#    point B (blue), the robot (yellow triangle) and the obstacles (red circles).
#
# NOTE: This is a simplified 2D simulation to help you develop and debug your decision
# -making logic. You must still test and validate your final solution on the real robot.
# =====================================================================================

import math
import random
import sys

import cv2
import numpy as np
import pygame

# --------------------------------------------------------------------------- settings
SCALE = 100          # pixels per metre, used only for drawing the window
ARENA_W_M = 6.0      # arena width in metres (point A on the left, point B on the right)
ARENA_H_M = 4.0      # arena height in metres
NUM_OBSTACLES = 5    # try increasing this for a harder scenario
GOAL_RADIUS_M = 0.5  # Task A counts as successful inside this radius of point B
ROBOT_RADIUS_M = 0.12
OBSTACLE_RADIUS_RANGE_M = (0.12, 0.28)
MAX_SPEED_MPS = 0.35        # approximate top speed of the real Trilobot
WHEEL_BASE_M = 0.16
SENSOR_MAX_RANGE_CM = 300.0
SENSOR_FOV_DEG = 10         # the real ultrasound sensor is a narrow forward-facing cone
CAMERA_FOV_DEG = 70
CAMERA_SIZE = (320, 240)    # (width, height), same resolution used in the example scripts
FPS = 30

# Underlighting colour presets, matching the names used in trilobot/scripts
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
BUTTON_A = "A"


# --------------------------------------------------------------------------- the world
class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


def make_world():
    point_a = (0.4, ARENA_H_M / 2)
    point_b = (ARENA_W_M - 0.4, ARENA_H_M / 2)
    obstacles = []
    attempts = 0
    while len(obstacles) < NUM_OBSTACLES and attempts < 500:
        attempts += 1
        x = random.uniform(1.2, ARENA_W_M - 1.2)
        y = random.uniform(0.4, ARENA_H_M - 0.4)
        r = random.uniform(*OBSTACLE_RADIUS_RANGE_M)
        # keep obstacles away from the start/goal points so the task is always solvable
        if math.hypot(x - point_a[0], y - point_a[1]) < 0.9:
            continue
        if math.hypot(x - point_b[0], y - point_b[1]) < 0.9:
            continue
        if any(math.hypot(x - o.x, y - o.y) < (r + o.radius + 0.25) for o in obstacles):
            continue
        obstacles.append(Obstacle(x, y, r))
    return point_a, point_b, obstacles


# --------------------------------------------------------------------- the fake robot
class SimTrilobot:
    """A drop-in, simplified stand-in for the real `trilobot.Trilobot` class."""

    def __init__(self, point_a, obstacles):
        self.x, self.y = point_a
        self.theta = 0.0  # heading, radians, 0 = facing along +x (towards point B)
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.obstacles = obstacles
        self.underlight_colour = BLACK
        self.quit_requested = False
        self.collided = False

    # -- motor control (same names/signatures as the real Trilobot library) --------
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

    def curve_forward_left(self, speed=1.0):
        self.set_motor_speeds(speed * 0.3, speed)

    def curve_forward_right(self, speed=1.0):
        self.set_motor_speeds(speed, speed * 0.3)

    def curve_backward_left(self, speed=1.0):
        self.set_motor_speeds(-speed * 0.3, -speed)

    def curve_backward_right(self, speed=1.0):
        self.set_motor_speeds(-speed, -speed * 0.3)

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

    # -- sensors ---------------------------------------------------------------
    def read_distance(self, timeout=50, samples=3):
        """Simulated ultrasound sensor: returns the distance (cm) to the closest
        obstacle/wall within a narrow cone in front of the robot."""
        best = SENSOR_MAX_RANGE_CM
        half_fov = math.radians(SENSOR_FOV_DEG / 2)
        for obstacle in self.obstacles:
            dx = obstacle.x - self.x
            dy = obstacle.y - self.y
            dist = math.hypot(dx, dy)
            angle_to_obj = math.atan2(dy, dx) - self.theta
            angle_to_obj = math.atan2(math.sin(angle_to_obj), math.cos(angle_to_obj))
            if abs(angle_to_obj) <= half_fov:
                edge_dist = max(0.0, dist - obstacle.radius) * 100  # m -> cm
                best = min(best, edge_dist)
        # distance to the arena walls, in case nothing else is seen
        wall_dist = self._distance_to_walls() * 100
        best = min(best, wall_dist)
        noise = random.uniform(-1.0, 1.0)
        return max(0.0, min(SENSOR_MAX_RANGE_CM, best + noise))

    def _distance_to_walls(self):
        dx, dy = math.cos(self.theta), math.sin(self.theta)
        candidates = []
        if dx > 1e-6:
            candidates.append((ARENA_W_M - self.x) / dx)
        elif dx < -1e-6:
            candidates.append((0 - self.x) / dx)
        if dy > 1e-6:
            candidates.append((ARENA_H_M - self.y) / dy)
        elif dy < -1e-6:
            candidates.append((0 - self.y) / dy)
        candidates = [c for c in candidates if c > 0]
        return min(candidates) if candidates else SENSOR_MAX_RANGE_CM / 100

    def get_camera_image(self):
        """Simulated camera: returns a BGR numpy image with the obstacles drawn as
        red blobs, similar to what scripts/trilobot/ball_tracking.py or
        scripts/trilobot/color_detection.py expect from `picamera`."""
        w, h = CAMERA_SIZE
        image = np.zeros((h, w, 3), dtype=np.uint8)
        half_fov = math.radians(CAMERA_FOV_DEG / 2)
        visible = []
        for obstacle in self.obstacles:
            dx = obstacle.x - self.x
            dy = obstacle.y - self.y
            dist = math.hypot(dx, dy)
            angle = math.atan2(dy, dx) - self.theta
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if dist < 0.05 or abs(angle) > half_fov:
                continue
            visible.append((dist, angle, obstacle))
        # draw far obstacles first so near ones are painted on top
        for dist, angle, obstacle in sorted(visible, key=lambda t: -t[0]):
            u = int(w / 2 + (angle / half_fov) * (w / 2))
            apparent_radius = int(max(4, min(w / 3, (obstacle.radius * 60) / max(dist, 0.1))))
            v = int(h / 2 + min(h / 3, dist * 10))
            cv2.circle(image, (u, v), apparent_radius, (0, 0, 255), -1)  # BGR red
        return image

    # -- physics update, called once per simulation frame -----------------------
    def _update(self, dt):
        v = (self.left_speed + self.right_speed) / 2.0 * MAX_SPEED_MPS
        w = (self.right_speed - self.left_speed) / WHEEL_BASE_M * MAX_SPEED_MPS
        self.theta += w * dt
        new_x = self.x + v * math.cos(self.theta) * dt
        new_y = self.y + v * math.sin(self.theta) * dt
        new_x = max(ROBOT_RADIUS_M, min(ARENA_W_M - ROBOT_RADIUS_M, new_x))
        new_y = max(ROBOT_RADIUS_M, min(ARENA_H_M - ROBOT_RADIUS_M, new_y))
        self.x, self.y = new_x, new_y
        for obstacle in self.obstacles:
            if math.hypot(self.x - obstacle.x, self.y - obstacle.y) < (ROBOT_RADIUS_M + obstacle.radius):
                self.collided = True


# =====================================================================================
#  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EDIT FROM HERE  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# =====================================================================================
# This function is called about 30 times per second. Use `tbot` exactly like you would
# use the real Trilobot object. The example below is a very simple obstacle avoidance
# behaviour: drive forward, and if something is detected too close in front, turn until
# the way ahead is clear again. Replace this with your own Task A algorithm (you could
# also use tbot.get_camera_image() with OpenCV, like in scripts/trilobot/color_detection.py).
AVOID_DISTANCE_CM = 35  # start turning when an obstacle is closer than this


def robot_control(tbot):
    distance = tbot.read_distance()

    if distance < AVOID_DISTANCE_CM:
        tbot.fill_underlighting(RED)
        tbot.turn_right(0.6)
    else:
        tbot.fill_underlighting(GREEN)
        tbot.forward(0.6)


# =====================================================================================
#  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EDIT UNTIL HERE  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# =====================================================================================


# --------------------------------------------------------------------------- rendering
def to_px(pos):
    return int(pos[0] * SCALE), int(pos[1] * SCALE)


def draw_world(screen, font, tbot, point_a, point_b, elapsed, finished, result_text):
    screen.fill((30, 30, 30))

    pygame.draw.circle(screen, (0, 180, 0), to_px(point_a), int(GOAL_RADIUS_M * SCALE), 2)
    pygame.draw.circle(screen, (0, 180, 0), to_px(point_a), 6)
    screen.blit(font.render("A", True, (0, 220, 0)), (to_px(point_a)[0] + 10, to_px(point_a)[1] - 25))

    pygame.draw.circle(screen, (60, 140, 255), to_px(point_b), int(GOAL_RADIUS_M * SCALE), 2)
    pygame.draw.circle(screen, (60, 140, 255), to_px(point_b), 6)
    screen.blit(font.render("B", True, (60, 140, 255)), (to_px(point_b)[0] + 10, to_px(point_b)[1] - 25))

    for obstacle in tbot.obstacles:
        pygame.draw.circle(screen, (200, 40, 40), to_px((obstacle.x, obstacle.y)), int(obstacle.radius * SCALE))

    # robot as a triangle pointing in the direction of travel
    rx, ry = to_px((tbot.x, tbot.y))
    r_px = int(ROBOT_RADIUS_M * SCALE)
    tip = (rx + r_px * math.cos(tbot.theta), ry + r_px * math.sin(tbot.theta))
    left = (rx + r_px * math.cos(tbot.theta + 2.5), ry + r_px * math.sin(tbot.theta + 2.5))
    right = (rx + r_px * math.cos(tbot.theta - 2.5), ry + r_px * math.sin(tbot.theta - 2.5))
    pygame.draw.polygon(screen, tbot.underlight_colour if tbot.underlight_colour != BLACK else (255, 255, 0),
                         [tip, left, right])

    hud_lines = [
        f"Elapsed: {elapsed:4.1f}s   Distance sensor: {tbot.read_distance():5.1f} cm",
        "Edit robot_control(tbot) in simulator_task_a.py. Press Q to quit.",
    ]
    if finished:
        hud_lines.append(result_text)
    for i, line in enumerate(hud_lines):
        screen.blit(font.render(line, True, (230, 230, 230)), (10, 10 + 20 * i))


def main():
    random.seed()
    point_a, point_b, obstacles = make_world()
    tbot = SimTrilobot(point_a, obstacles)

    pygame.init()
    screen = pygame.display.set_mode((int(ARENA_W_M * SCALE), int(ARENA_H_M * SCALE)))
    pygame.display.set_caption("AGR9027 - Task A simulator: obstacle avoidance")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)

    start_ticks = pygame.time.get_ticks()
    finished = False
    result_text = ""

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key in (pygame.K_q, pygame.K_ESCAPE):
                tbot.quit_requested = True
                running = False

        if not finished:
            robot_control(tbot)
            tbot._update(dt)

            if tbot.collided:
                finished = True
                result_text = "RESULT: COLLISION - task failed. Close the window or press Q."
            elif math.hypot(tbot.x - point_b[0], tbot.y - point_b[1]) < GOAL_RADIUS_M:
                finished = True
                result_text = "RESULT: SUCCESS - point B reached without collisions!"

        elapsed = (pygame.time.get_ticks() - start_ticks) / 1000.0
        draw_world(screen, font, tbot, point_a, point_b, elapsed, finished, result_text)
        pygame.display.flip()

        if finished and result_text:
            print(result_text)
            result_text = ""  # only print once

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
