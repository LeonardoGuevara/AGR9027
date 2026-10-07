# =============================================================================
# AGR9027 - Task B Simulator: Person Following
# =============================================================================
# PURPOSE
# Test following a moving target person while avoiding confusion with decoys,
# without the physical Trilobot.
#
# SCENARIO
# The simulator places a target and 0, 1, 3 or 5 decoys at complexity levels
# 1-4. Some higher-level decoys wear clothes in colours similar to the target.
# The 2D map shows people, robot, sensor fields of view and underlighting. A
# simulated BGR camera feed is shown below the map.
#
# STUDENT INTERFACE
# The simulator calls robot_control(tbot, target_color) from student_control.py
# repeatedly. The fake tbot provides movement, distance-sensor, underlighting
# and camera methods. The student_control.py webcam function processes a BGR
# image and target colour, returning a debug image and a short summary.
#
# HOW TO RUN
# From this folder, run: python simulator_task_b.py
# Edit student_control.py for your robot and camera logic. Keep this simulator
# file unchanged.
#
# CONTROLS
# n: new scene at the current level; 1-4: select a level and regenerate;
# d: advance a level and regenerate; q / ESC: quit.
#
# LIMITATIONS
# This simplified 2D simulation is for development only. Validate the final
# solution on the physical robot.
# =============================================================================

import math
import random
import sys

import cv2
import numpy as np
import pygame

from student_control import robot_control

# --------------------------------------------------------------------------- settings
SCALE = 120
ARENA_W_M = 6.0
ARENA_H_M = 4.0
COMPLEXITY_LEVEL = 1  # 1 (easiest) to 4 (hardest)
DECOYS_BY_LEVEL = {1: 0, 2: 1, 3: 3, 4: 5}
SIMILAR_DECOYS_BY_LEVEL = {1: 0, 2: 0, 3: 1, 4: 2}
ROBOT_RADIUS_M = 0.12
PERSON_RADIUS_M = 0.22
ROBOT_MAX_SPEED_MPS = 0.35
GOAL_DISTANCE_M = 0.5
WHEEL_BASE_M = 0.16
SENSOR_MAX_RANGE_CM = 300.0
SENSOR_FOV_DEG = 20
CAMERA_FOV_DEG = 70
CAMERA_MAX_RANGE_M = 6.0
CAMERA_SIZE = (320, 240)
CAMERA_FOCAL_LENGTH_PX = CAMERA_SIZE[0] / (2 * math.tan(math.radians(CAMERA_FOV_DEG / 2)))
FPS = 30
TARGET_WALK_SPEED_MPS = 0.15
TARGET_WALK_ANGLE_RANGE_DEG = (-20, 20)
PERSON_START_DISTANCE_M = 2.4
PERSON_HEIGHT_M = 1.7
PERSON_WIDTH_M = 0.48
MAX_PERSON_ANGLE_DEG = 26.5

RED = (255, 0, 0)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
BUTTON_A = "A"
COLOUR_NAMES = {
    "RED": RED,
    "YELLOW": YELLOW,
    "GREEN": GREEN,
    "BLUE": BLUE,
}
BASE_HUES = {"RED": 2, "YELLOW": 25, "GREEN": 52, "BLUE": 115}
HUE_LIMITS = {"RED": (0, 8), "YELLOW": (16, 35), "GREEN": (38, 68), "BLUE": (102, 132)}

# --------------------------------------------------------------------------- the world
class Person:
    def __init__(self, x, y, colour_name, is_target):
        self.x = x
        self.y = y
        self.colour_name = colour_name
        self.colour_hsv = make_clothing_hsv(colour_name)
        self.colour = hsv_to_rgb(self.colour_hsv)
        self.is_target = is_target
        self.walk_angle = random.uniform(0, 2 * math.pi)

    def wander(self, dt):
        if not self.is_target:
            return
        self.x += math.cos(self.walk_angle) * TARGET_WALK_SPEED_MPS * dt
        self.y += math.sin(self.walk_angle) * TARGET_WALK_SPEED_MPS * dt


def make_clothing_hsv(colour_name, similar_to=None):
    low_hue, high_hue = HUE_LIMITS[colour_name]
    if similar_to is None:
        hue = int(round(random.gauss(BASE_HUES[colour_name], 2)))
        hue = max(low_hue, min(high_hue, hue))
        return hue, random.randint(185, 245), random.randint(175, 250)

    target_hue, target_saturation, target_value = similar_to
    for _ in range(20):
        hue = max(low_hue, min(high_hue, target_hue + random.choice((-4, -3, -2, 2, 3, 4))))
        saturation = max(150, min(255, target_saturation + random.choice((-35, -25, 20, 30, 40))))
        value = max(145, min(255, target_value + random.choice((-40, -30, 25, 35, 45))))
        candidate = (hue, saturation, value)
        if candidate != similar_to:
            return candidate
    return target_hue, max(150, target_saturation - 35), target_value


def hsv_to_rgb(hsv):
    bgr = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
    return tuple(int(channel) for channel in bgr[::-1])


def make_world():
    all_colours = list(COLOUR_NAMES)
    target_colour = random.choice(all_colours)
    target = Person(0, 0, target_colour, is_target=True)
    decoys = []
    similar_count = SIMILAR_DECOYS_BY_LEVEL[COMPLEXITY_LEVEL]
    other_colours = [colour for colour in all_colours if colour != target_colour]
    robot_start = (0.6, ARENA_H_M / 2)
    for _ in range(similar_count):
        decoy = Person(0, 0, target_colour, is_target=False)
        decoy.colour_hsv = make_clothing_hsv(target_colour, target.colour_hsv)
        decoy.colour = hsv_to_rgb(decoy.colour_hsv)
        decoys.append(decoy)
    for _ in range(DECOYS_BY_LEVEL[COMPLEXITY_LEVEL] - similar_count):
        decoys.append(Person(0, 0, random.choice(other_colours), is_target=False))

    target.walk_angle = math.radians(random.uniform(*TARGET_WALK_ANGLE_RANGE_DEG))
    decoy_angles = []
    for index in range(len(decoys)):
        magnitude = MAX_PERSON_ANGLE_DEG * (index // 2 + 1) / math.ceil(len(decoys) / 2)
        decoy_angles.append(-magnitude if index % 2 == 0 else magnitude)
    target.x = robot_start[0] + PERSON_START_DISTANCE_M
    target.y = robot_start[1]
    for person, angle_degrees in zip(decoys, decoy_angles):
        angle = math.radians(float(angle_degrees))
        person.x = robot_start[0] + PERSON_START_DISTANCE_M * math.cos(angle)
        person.y = robot_start[1] + PERSON_START_DISTANCE_M * math.sin(angle)
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
        self.collided = False
        self.collision_person = None

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
        """Return a BGR image with colored clothes on simple human silhouettes."""
        w, h = CAMERA_SIZE
        image = np.full((h, w, 3), (28, 38, 34), dtype=np.uint8)
        half_fov = math.radians(CAMERA_FOV_DEG / 2)
        visible = []
        for person in self.people:
            dx, dy = person.x - self.x, person.y - self.y
            dist = math.hypot(dx, dy)
            angle = math.atan2(dy, dx) - self.theta
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if dist < 0.05 or dist > CAMERA_MAX_RANGE_M or abs(angle) > half_fov:
                continue
            visible.append((dist, angle, person))
        for dist, angle, person in sorted(visible, key=lambda t: -t[0]):
            u = int(w / 2 + (angle / half_fov) * (w / 2))
            v = int(h / 2)
            body_width = max(1, int(CAMERA_FOCAL_LENGTH_PX * PERSON_WIDTH_M / dist))
            body_height = max(2, int(CAMERA_FOCAL_LENGTH_PX * PERSON_HEIGHT_M / dist))
            top = v - body_height // 2
            bottom = v + body_height // 2
            skin_bgr = (145, 145, 145)
            shirt_bgr = (person.colour[2], person.colour[1], person.colour[0])
            pants_bgr = (105, 70, 35)
            line_width = max(1, int(body_width * 0.16))
            cv2.line(image, (u - body_width // 3, top + body_height // 2),
                     (u - body_width // 3, bottom), pants_bgr, line_width)
            cv2.line(image, (u + body_width // 3, top + body_height // 2),
                     (u + body_width // 3, bottom), pants_bgr, line_width)
            cv2.ellipse(image, (u, top + body_height // 2),
                        (max(1, body_width // 2), max(1, body_height // 4)), 0, 0, 360, shirt_bgr, -1)
            cv2.line(image, (u - body_width // 3, top + body_height // 4),
                     (u - body_width // 2, top + body_height * 3 // 4),
                     skin_bgr, line_width)
            cv2.line(image, (u + body_width // 3, top + body_height // 4),
                     (u + body_width // 2, top + body_height * 3 // 4),
                     skin_bgr, line_width)
            cv2.circle(image, (u, top + body_height // 8), max(1, body_width // 5), skin_bgr, -1)
        return image

    def _update(self, dt):
        v = (self.left_speed + self.right_speed) / 2.0 * ROBOT_MAX_SPEED_MPS
        # Screen y increases downward, so positive angular velocity is a right turn.
        w = (self.left_speed - self.right_speed) / WHEEL_BASE_M * ROBOT_MAX_SPEED_MPS
        self.theta += w * dt
        new_x = self.x + v * math.cos(self.theta) * dt
        new_y = self.y + v * math.sin(self.theta) * dt
        self.x = max(ROBOT_RADIUS_M, min(ARENA_W_M - ROBOT_RADIUS_M, new_x))
        self.y = max(ROBOT_RADIUS_M, min(ARENA_H_M - ROBOT_RADIUS_M, new_y))
        for person in self.people:
            if math.hypot(self.x - person.x, self.y - person.y) < ROBOT_RADIUS_M + PERSON_RADIUS_M:
                self.collided = True
                self.collision_person = person
                break


# --------------------------------------------------------------------------- rendering
def to_px(pos):
    return int(pos[0] * SCALE), int(pos[1] * SCALE)


def draw_sensor_fovs(screen, tbot):
    origin = to_px((tbot.x, tbot.y))
    overlay = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
    for fov, distance, colour in (
        (CAMERA_FOV_DEG, CAMERA_MAX_RANGE_M, (40, 120, 255, 48)),
        (SENSOR_FOV_DEG, SENSOR_MAX_RANGE_CM / 100, (255, 210, 40, 90)),
    ):
        half_angle = math.radians(fov / 2)
        points = [origin]
        for index in range(25):
            angle = tbot.theta - half_angle + 2 * half_angle * index / 24
            points.append(to_px((
                tbot.x + math.cos(angle) * distance,
                tbot.y + math.sin(angle) * distance,
            )))
        pygame.draw.polygon(overlay, colour, points)
    screen.blit(overlay, (0, 0))


def draw_camera_feed(screen, font, image):
    panel_y = int(ARENA_H_M * SCALE) + 28
    pygame.draw.rect(screen, (18, 18, 18), (0, panel_y - 24, screen.get_width(), CAMERA_SIZE[1] + 48))
    screen.blit(font.render("Simulated camera (BGR image shown as colour)", True, (230, 230, 230)),
                (10, panel_y - 21))
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    camera_surface = pygame.surfarray.make_surface(np.transpose(rgb_image, (1, 0, 2)))
    screen.blit(camera_surface, (10, panel_y))


def draw_world(screen, font, tbot, target, elapsed):
    screen.fill((30, 30, 30))
    draw_sensor_fovs(screen, tbot)

    for person in tbot.people:
        px, py = to_px((person.x, person.y))
        unit = max(3, int(PERSON_RADIUS_M * SCALE))
        skin_colour = (145, 145, 145)
        pants_colour = (35, 70, 105)
        pygame.draw.line(screen, skin_colour, (px - unit // 3, py - unit // 4),
                         (px - unit // 2, py + unit // 5), 4)
        pygame.draw.line(screen, skin_colour, (px + unit // 3, py - unit // 4),
                         (px + unit // 2, py + unit // 5), 4)
        pygame.draw.line(screen, pants_colour, (px - unit // 4, py + unit // 3),
                         (px - unit // 4, py + unit), 5)
        pygame.draw.line(screen, pants_colour, (px + unit // 4, py + unit // 3),
                         (px + unit // 4, py + unit), 5)
        pygame.draw.ellipse(screen, person.colour,
                            pygame.Rect(px - unit // 2, py - unit // 3, unit,
                                        max(6, 2 * unit // 3)))
        pygame.draw.circle(screen, skin_colour, (px, py - unit // 2), max(4, unit // 5))
        if person.is_target:
            pygame.draw.circle(screen, (255, 255, 255), (px, py), int(PERSON_RADIUS_M * SCALE), 2)

    rx, ry = to_px((tbot.x, tbot.y))
    r_px = int(ROBOT_RADIUS_M * SCALE)
    tip = (rx + r_px * math.cos(tbot.theta), ry + r_px * math.sin(tbot.theta))
    left = (rx + r_px * math.cos(tbot.theta + 2.5), ry + r_px * math.sin(tbot.theta + 2.5))
    right = (rx + r_px * math.cos(tbot.theta - 2.5), ry + r_px * math.sin(tbot.theta - 2.5))
    colour = tbot.underlight_colour if tbot.underlight_colour != BLACK else (255, 255, 255)
    pygame.draw.polygon(screen, colour, [tip, left, right])
    pygame.draw.circle(screen, colour, (rx, ry), max(3, r_px // 4))

    distance_to_target = math.hypot(tbot.x - target.x, tbot.y - target.y)
    robot_speed_mps = (tbot.left_speed + tbot.right_speed) / 2 * ROBOT_MAX_SPEED_MPS
    lines = [
        f"Target colour: {target.colour_name}   Complexity: {COMPLEXITY_LEVEL}/4   Elapsed: {elapsed:4.1f}s",
        f"Centre distance: {distance_to_target:4.2f} m (safe: "
        f"{GOAL_DISTANCE_M + PERSON_RADIUS_M:.2f} m)  "
        f"Robot: {robot_speed_mps:.2f} m/s  Target: {TARGET_WALK_SPEED_MPS:.2f} m/s",
        f"Sensor surface gap: {tbot.read_distance():5.1f} cm  "
        f"Similar-colour decoys: {SIMILAR_DECOYS_BY_LEVEL[COMPLEXITY_LEVEL]}",
        "FOV: camera blue (6 m), ultrasonic distance gold | n: new, 1-4: level, d: next",
        "Edit robot_control(tbot, target_color) in student_control.py. Press Q / ESC to quit.",
    ]
    if tbot.collided:
        person_label = "target" if tbot.collision_person.is_target else "decoy"
        lines.append(f"RESULT: COLLISION with {person_label} - simulation ended.")
    elif not (0 <= target.x <= ARENA_W_M and 0 <= target.y <= ARENA_H_M):
        lines.append("RESULT: TARGET LEFT THE MAP - simulation ended.")
    for i, line in enumerate(lines):
        screen.blit(font.render(line, True, (230, 230, 230)), (10, 10 + 20 * i))


def main():
    global COMPLEXITY_LEVEL
    random.seed()
    robot_start, target, decoys = make_world()
    people = [target] + decoys
    tbot = SimTrilobot(robot_start, people)
    print(f"Target colour to follow: {target.colour_name}")

    pygame.init()
    map_height = int(ARENA_H_M * SCALE)
    screen = pygame.display.set_mode((
        int(ARENA_W_M * SCALE),
        map_height + CAMERA_SIZE[1] + 56,
    ))
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
            if event.type == pygame.KEYDOWN and event.key not in (pygame.K_q, pygame.K_ESCAPE):
                new_level = COMPLEXITY_LEVEL
                regenerate = False
                if pygame.K_1 <= event.key <= pygame.K_4:
                    new_level = event.key - pygame.K_0
                    regenerate = True
                elif event.key == pygame.K_d:
                    new_level = min(4, COMPLEXITY_LEVEL + 1)
                    regenerate = True
                elif event.key == pygame.K_n:
                    regenerate = True
                if regenerate:
                    COMPLEXITY_LEVEL = new_level
                    robot_start, target, decoys = make_world()
                    people = [target] + decoys
                    tbot = SimTrilobot(robot_start, people)
                    start_ticks = pygame.time.get_ticks()

        if not tbot.collided and 0 <= target.x <= ARENA_W_M and 0 <= target.y <= ARENA_H_M:
            robot_control(tbot, target.colour_name)
            tbot._update(dt)
            for person in tbot.people:
                person.wander(dt)

        elapsed = (pygame.time.get_ticks() - start_ticks) / 1000.0
        draw_world(screen, font, tbot, target, elapsed)
        draw_camera_feed(screen, font, tbot.get_camera_image())
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
