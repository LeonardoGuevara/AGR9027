# =============================================================================
# AGR9027 - Task C Simulator: Strawberry Counting
# =============================================================================
# PURPOSE
# Test ripe-strawberry counting without the physical Trilobot or a printed
# sample image.
#
# SCENARIO
# The simulator generates a synthetic patch containing ripe, semi-ripe and
# unripe strawberries. Complexity levels 1-4 contain 8, 12, 16 and 20 ripe
# fruits to count, respectively. The two windows show the student result and a
# ground-truth view marking ripe fruit.
#
# STUDENT INTERFACE
# The simulator calls count_ripe_strawberries(image) from student_control.py
# with a BGR image. It expects a count and a debug image in return. The same
# file also provides a webcam-frame processing function.
#
# HOW TO RUN
# From this folder, run: python simulator_task_c.py
# Edit student_control.py to change the counting and camera-vision logic. Keep
# this simulator file unchanged.
#
# CONTROLS
# n: new scene at the current level; 1-4: select a level and regenerate;
# d: advance a level and regenerate; q / ESC: quit.
#
# LIMITATIONS
# This synthetic image is a practice aid, not a replacement for testing with
# real strawberry images or validating the final solution on the physical robot.
# =============================================================================

import math
import random

import cv2
import numpy as np

from student_control import count_ripe_strawberries

IMAGE_SIZE = (640, 480)  # (width, height)
COMPLEXITY_LEVEL = 1
RIPE_BY_LEVEL = {1: 8, 2: 12, 3: 16, 4: 20}
NON_RIPE_BY_LEVEL = {1: 4, 2: 8, 3: 14, 4: 20}
SEMI_RIPE_BY_LEVEL = {1: 0, 2: 2, 3: 6, 4: 12}
OVERLAP_FRACTION_BY_LEVEL = {1: 0.0, 2: 0.15, 3: 0.3, 4: 1.0}


# --------------------------------------------------------------------- scene generator
def strawberry_outline(x, y, radius):
    """Make an irregular tapered silhouette with a pointed strawberry tip."""
    shape = [
        (-0.52, -0.82), (-0.22, -1.0), (0.2, -0.96), (0.58, -0.78),
        (0.86, -0.35), (0.92, 0.08), (0.65, 0.58), (0.2, 1.02),
        (0.0, 1.18), (-0.2, 1.02), (-0.65, 0.58), (-0.92, 0.08),
        (-0.86, -0.35),
    ]
    return np.array([
        (int(x + dx * radius * random.uniform(0.9, 1.1)),
         int(y + dy * radius * random.uniform(0.92, 1.08)))
        for dx, dy in shape
    ], dtype=np.int32)


def hsv_colour_to_bgr(hsv):
    colour = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
    return tuple(int(channel) for channel in colour)


def draw_strawberry(image, x, y, radius, fruit_type):
    outline = strawberry_outline(x, y, radius)
    fruit_mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(fruit_mask, [outline], 255)
    if fruit_type == "ripe":
        hsv = (random.randint(0, 4), random.randint(200, 255), random.randint(200, 255))
        image[fruit_mask != 0] = hsv_colour_to_bgr(hsv)
    elif fruit_type == "unripe":
        hsv = (random.randint(32, 72), random.randint(100, 200), random.randint(100, 200))
        image[fruit_mask != 0] = hsv_colour_to_bgr(hsv)
    elif random.random() < 0.7:
        rows, _ = np.nonzero(fruit_mask)
        top = int(rows.min())
        bottom = int(rows.max())
        transition_height = max(1, int((bottom - top + 1) * random.uniform(0.1, 0.4)))
        yellow_hue = random.randint(22, 34)
        red_hue = random.randint(0, 8)
        saturation = random.randint(175, 255)
        value = random.randint(170, 250)
        hsv_image = np.zeros_like(image)
        for row in range(top, bottom + 1):
            progress = min(1.0, (row - top) / transition_height)
            hue = int(round(yellow_hue + (red_hue - yellow_hue) * progress))
            hsv_image[row, :] = (hue, saturation, value)
        bgr_gradient = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
        image[fruit_mask != 0] = bgr_gradient[fruit_mask != 0]
    else:
        hsv = (random.randint(0, 34), random.randint(165, 245), random.randint(165, 250))
        image[fruit_mask != 0] = hsv_colour_to_bgr(hsv)

    leaf_colour = (25, 115, 35)
    for offset in (-0.3, 0.0, 0.3):
        cv2.line(image, (x, y - radius), (int(x + offset * radius), y - int(radius * 1.2)),
                 leaf_colour, max(1, radius // 7))

    for _ in range(10):
        sx = random.randint(max(0, x - radius), min(image.shape[1] - 1, x + radius))
        sy = random.randint(max(0, y - radius), min(image.shape[0] - 1, y + radius))
        if cv2.pointPolygonTest(outline, (float(sx), float(sy)), False) >= 0:
            seed_colour = (70, 190, 225) if fruit_type != "unripe" else (130, 205, 140)
            cv2.ellipse(image, (sx, sy), (1, 2), random.randint(-30, 30), 0, 360, seed_colour, -1)


def generate_strawberry_scene(num_ripe=8, num_unripe=4, num_semi_ripe=0, overlap_fraction=0.0):
    """Create a top-down synthetic patch with irregular, partially occluded fruit."""
    w, h = IMAGE_SIZE
    image = np.full((h, w, 3), (40, 90, 40), dtype=np.uint8)  # green "leaves" background

    # Leaf blobs provide background clutter.
    for _ in range(15):
        cx, cy = random.randint(0, w - 1), random.randint(0, h - 1)
        radius = random.randint(15, 45)
        cv2.ellipse(image, (cx, cy), (radius, int(radius * 0.55)), random.randint(0, 180),
                    0, 360, (30, 120, 30), -1)

    fruit_types = ["ripe"] * num_ripe + ["semi-ripe"] * num_semi_ripe + ["unripe"] * num_unripe
    overlap_count = min(len(fruit_types), round(len(fruit_types) * overlap_fraction))
    if overlap_count % 2:
        overlap_count += 1 if overlap_count < len(fruit_types) else -1

    fruit_by_type = {
        fruit_type: [fruit_type] * fruit_types.count(fruit_type)
        for fruit_type in ("ripe", "semi-ripe", "unripe")
    }
    for fruits in fruit_by_type.values():
        random.shuffle(fruits)

    overlap_groups = []
    remaining_overlap_count = overlap_count
    for fruit_type in ("ripe", "semi-ripe", "unripe"):
        available = fruit_by_type[fruit_type]
        selected_count = min(len(available), remaining_overlap_count)
        selected_count -= selected_count % 2
        if selected_count == 0:
            continue

        selected = available[:selected_count]
        del available[:selected_count]
        while len(selected) >= 2:
            overlap_groups.append(selected[:2])
            del selected[:2]
        remaining_overlap_count -= selected_count

    overlap_count = sum(len(group) for group in overlap_groups)
    random.shuffle(overlap_groups)
    overlapping_types = [fruit_type for group in overlap_groups for fruit_type in group]
    non_overlapping_types = [
        fruit_type
        for fruit_type, fruits in fruit_by_type.items()
        for _ in fruits
    ]
    fruit_types = overlapping_types + non_overlapping_types
    radii = [random.randint(18, 30) for _ in fruit_types]
    placements = [None] * len(fruit_types)
    placed = []

    # Keep pairs spatially separate and group fruits by ripeness so ripe berries
    # overlap other ripe berries instead of semi-ripe or unripe fruit.
    cluster_anchors = [(x, y) for y in (65, 182, 298, 415)
                       for x in (65, 192, 320, 448, 575)]
    random.shuffle(cluster_anchors)
    next_index = 0
    for group_index, group in enumerate(overlap_groups):
        anchor_x, anchor_y = cluster_anchors[group_index]
        group_radii = radii[next_index:next_index + len(group)]
        group_radius = sum(group_radii) / len(group)
        angle = random.uniform(0, 2 * math.pi)
        for member_index, radius in enumerate(group_radii):
            member_angle = angle + 2 * math.pi * member_index / len(group)
            distance = group_radius * 0.45
            x = int(anchor_x + math.cos(member_angle) * distance)
            y = int(anchor_y + math.sin(member_angle) * distance)
            placements[next_index + member_index] = (x, y)
            placed.append((x, y, radius))
        next_index += len(group)

    for index in range(overlap_count, len(fruit_types)):
        radius = radii[index]
        for _ in range(500):
            x = random.randint(radius + 5, w - radius - 5)
            y = random.randint(radius + 10, h - int(radius * 1.2) - 5)
            if all(math.hypot(x - px, y - py) > (radius + other_radius) * 1.12
                   for px, py, other_radius in placed):
                placements[index] = (x, y)
                placed.append((x, y, radius))
                break
        if placements[index] is None:
            raise RuntimeError(
                f"Could not place strawberry {index + 1} without unintended overlap."
            )

    ripe_markers = []
    for (x, y), radius, fruit_type in zip(placements, radii, fruit_types):
        draw_strawberry(image, x, y, radius, fruit_type)
        if fruit_type == "ripe":
            ripe_markers.append((x, y, max(1, int(radius * 1.2))))

    image = cv2.GaussianBlur(image, (3, 3), 0)
    return image, num_ripe, ripe_markers


def make_ground_truth_view(image, ripe_markers):
    """Return a scene copy with only the ripe fruit marked by green circles."""
    view = image.copy()
    for x, y, radius in ripe_markers:
        cv2.circle(view, (x, y), radius, (0, 255, 0), 2)
    return view


def main():
    complexity_level = COMPLEXITY_LEVEL

    def new_scene(level):
        num_semi_ripe = SEMI_RIPE_BY_LEVEL[level]
        return generate_strawberry_scene(
            num_ripe=RIPE_BY_LEVEL[level],
            num_unripe=NON_RIPE_BY_LEVEL[level] - num_semi_ripe,
            num_semi_ripe=num_semi_ripe,
            overlap_fraction=OVERLAP_FRACTION_BY_LEVEL[level],
        )

    image, true_count, ripe_markers = new_scene(complexity_level)

    print("AGR9027 - Task C simulator: strawberry counting")
    print("Keys: [n] new scene   [1-4] select complexity   [d] next level   [q]/[ESC] quit\n")
    result_window = "AGR9027 - Task C simulator: student result"
    truth_window = "AGR9027 - Task C simulator: ripe-fruit ground truth"
    cv2.namedWindow(result_window)
    cv2.namedWindow(truth_window)
    cv2.moveWindow(result_window, 40, 40)
    cv2.moveWindow(truth_window, IMAGE_SIZE[0] + 70, 40)

    while True:
        count, debug_image = count_ripe_strawberries(image)
        truth_image = make_ground_truth_view(image, ripe_markers)
        error_pct = 100 * abs(count - true_count) / max(1, true_count)

        label = f"Detected ripe: {count}  (ground truth: {true_count}, error: {error_pct:.0f}%)"
        cv2.putText(debug_image, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_image, f"Complexity {complexity_level}/4 | n: new scene  1-4: level  d: next level  q: quit",
                    (10, IMAGE_SIZE[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(truth_image, "Ground-truth ripe fruit", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        print(label)

        cv2.imshow(result_window, debug_image)
        cv2.imshow(truth_window, truth_image)
        key = cv2.waitKey(0) & 0xFF

        if key in (ord("q"), 27):  # q or ESC
            break
        elif key == ord("n"):
            image, true_count, ripe_markers = new_scene(complexity_level)
        elif key == ord("d"):
            complexity_level = min(4, complexity_level + 1)
            image, true_count, ripe_markers = new_scene(complexity_level)
        elif ord("1") <= key <= ord("4"):
            complexity_level = key - ord("0")
            image, true_count, ripe_markers = new_scene(complexity_level)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
