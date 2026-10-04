# =====================================================================================
# AGR9027 - Task C simulator: Strawberry yield counting
# =====================================================================================
# This file lets you test your Task C code on your own laptop, without the real
# Trilobot or a printed photo. It generates a synthetic "photo" of a strawberry patch
# (ripe = red berries, unripe = green/white berries, some overlapping each other to
# simulate occlusion) and feeds it to a counting algorithm, exactly like you would do
# with a picture captured by the robot's camera (see scripts/trilobot/color_detection.py and
# scripts/trilobot/ball_tracking.py for the colour/ circle detection approach used there).
#
# HOW TO USE THIS FILE
# ---------------------
# 1. Install the requirements: pip install -r ../requirements.txt
# 2. Run this file directly: python simulator_task_c.py
# 3. A window will open showing the synthetic photo and the ripe strawberries your
#    code has detected (circled in blue), with the live count printed on the image
#    and in the terminal.
# 4. Controls:
#       n - generate a new random scene (same difficulty)
#       d - generate a new, HARDER scene (more fruits, more overlap, more unripe fruit)
#       q / ESC - quit
# 5. Edit ONLY the function `count_ripe_strawberries(image)` below. It already
#    contains a simple working example (HSV red colour mask + contour counting).
#    Replace/extend it with your own Task C algorithm.
#
# NOTE: real printed photos of strawberries for Task C will be added by your module
# leader to the /sample_images folder - this generator is only meant to help you start
# developing and testing your counting logic before those are available.
# =====================================================================================

import random

import cv2
import numpy as np

IMAGE_SIZE = (640, 480)  # (width, height)
MIN_CONTOUR_AREA = 80    # ignore tiny red specks / noise


# --------------------------------------------------------------------- scene generator
def generate_strawberry_scene(num_ripe=8, num_unripe=4, overlap_level=0.0):
    """Creates a synthetic top-down "photo" of a strawberry patch.
    overlap_level in [0, 1]: 0 = berries never touch, 1 = berries placed very close
    together so some of them occlude each other (harder to separate/count)."""
    w, h = IMAGE_SIZE
    image = np.full((h, w, 3), (40, 90, 40), dtype=np.uint8)  # green "leaves" background

    # a few leaf blobs for visual noise (not meant to be counted)
    for _ in range(15):
        cx, cy = random.randint(0, w), random.randint(0, h)
        radius = random.randint(15, 45)
        cv2.circle(image, (cx, cy), radius, (30, 120, 30), -1)

    placed = []  # (x, y, radius)
    min_gap_factor = 1.6 - 1.3 * overlap_level  # smaller => more overlap allowed

    def try_place(radius):
        for _ in range(60):
            x = random.randint(radius + 5, w - radius - 5)
            y = random.randint(radius + 5, h - radius - 5)
            if all(((x - px) ** 2 + (y - py) ** 2) ** 0.5 > (radius + pr) * min_gap_factor
                   for px, py, pr in placed):
                return x, y
        return x, y  # give up avoiding overlap, place it anyway (simulates occlusion)

    ripe_positions = []
    for _ in range(num_ripe):
        radius = random.randint(18, 30)
        x, y = try_place(radius)
        placed.append((x, y, radius))
        ripe_positions.append((x, y, radius))
        colour = (random.randint(20, 50), random.randint(20, 60), random.randint(170, 255))  # BGR red
        cv2.circle(image, (x, y), radius, colour, -1)
        # a few seed-like darker speckles for realism
        for _ in range(6):
            sx = x + random.randint(-radius // 2, radius // 2)
            sy = y + random.randint(-radius // 2, radius // 2)
            cv2.circle(image, (sx, sy), 1, (10, 200, 230), -1)

    for _ in range(num_unripe):
        radius = random.randint(16, 28)
        x, y = try_place(radius)
        placed.append((x, y, radius))
        colour = (random.randint(20, 60), random.randint(140, 200), random.randint(40, 90))  # BGR green/white
        cv2.circle(image, (x, y), radius, colour, -1)

    image = cv2.GaussianBlur(image, (3, 3), 0)
    return image, len(ripe_positions)


# =====================================================================================
#  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EDIT FROM HERE  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# =====================================================================================
# This function receives a BGR image (numpy array) and must return:
#   count       - your estimate of the number of RIPE strawberries
#   debug_image - a copy of the image with your detections drawn on it (for display)
# The example below uses an HSV colour mask for "ripe red" (like
# scripts/trilobot/color_detection.py) and counts the resulting blobs with
# cv2.findContours. LIMITATION to improve: overlapping/touching strawberries may be
# merged into a single blob and counted only once - this is exactly the kind of
# limitation you are expected to discuss/improve for the harder levels of Task C.


def count_ripe_strawberries(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_lower = cv2.inRange(hsv, (0, 90, 90), (10, 255, 255))
    mask_upper = cv2.inRange(hsv, (170, 90, 90), (180, 255, 255))
    mask = cv2.bitwise_or(mask_lower, mask_upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    debug_image = image.copy()
    count = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_CONTOUR_AREA:
            continue
        (x, y), radius = cv2.minEnclosingCircle(contour)
        cv2.circle(debug_image, (int(x), int(y)), int(radius), (255, 0, 0), 2)
        count += 1

    return count, debug_image


# =====================================================================================
#  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EDIT UNTIL HERE  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# =====================================================================================


def main():
    difficulty = {"num_ripe": 8, "num_unripe": 4, "overlap_level": 0.0}
    image, true_count = generate_strawberry_scene(**difficulty)

    print("AGR9027 - Task C simulator: strawberry counting")
    print("Keys: [n] new scene   [d] harder scene   [q]/[ESC] quit\n")

    while True:
        count, debug_image = count_ripe_strawberries(image)
        error_pct = 100 * abs(count - true_count) / max(1, true_count)

        label = f"Detected ripe: {count}  (ground truth: {true_count}, error: {error_pct:.0f}%)"
        cv2.putText(debug_image, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_image, "n: new scene   d: harder scene   q: quit", (10, IMAGE_SIZE[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        print(label)

        cv2.imshow("AGR9027 - Task C simulator: strawberry counting", debug_image)
        key = cv2.waitKey(0) & 0xFF

        if key in (ord("q"), 27):  # q or ESC
            break
        elif key == ord("n"):
            image, true_count = generate_strawberry_scene(**difficulty)
        elif key == ord("d"):
            difficulty["num_ripe"] += 4
            difficulty["num_unripe"] += 3
            difficulty["overlap_level"] = min(1.0, difficulty["overlap_level"] + 0.25)
            image, true_count = generate_strawberry_scene(**difficulty)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
