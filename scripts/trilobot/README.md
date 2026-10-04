# AGR9027 Trilobot example scripts

This folder contains example Python scripts for the real Trilobot robot, used for
Assessment 2. They require the real robot hardware (the `trilobot` and `picamera`
packages, camera, ultrasound sensor, motors, underlighting) and should be run on the
Raspberry Pi inside the robot - **not** on your own laptop. If you want to develop and
test code without the real robot, use the [trilobot_simulator/](../../trilobot_simulator)
folder instead.

This is one of three subfolders inside [scripts/](..):
`trilobot/` (this folder, robot hardware examples), `computer_vision/` and
`machine_learning/` (basics covered in separate taught sessions). See the
[main README](../../README.md) for an overview of the whole repository.

These scripts are meant to be read, copied and modified - they are simple, generic
building blocks, not finished solutions to Tasks A, B or C. You are expected to combine
and adapt them (and add your own logic/algorithms) to complete the assessment.

## How the scripts build on each other (basic -> complex)

```
Level 0 - hardware basics (no camera, just motors/sensors/lights)
  movements.py            motor control only
  print_distance.py       ultrasound sensor only
  show_underlighting.py   underlighting only
       |
       v
Level 1 - combining two basic hardware features
  distance_lights.py      ultrasound sensor + underlighting
  follow_straight.py      ultrasound sensor + motor control
  obstacle_avoidance.py   ultrasound sensor + motor control (reactive avoidance)
       |
       v
Level 2 - introducing the camera and basic image processing
  color_detection.py      camera + HSV colour masking (what colour is in front of me?)
  face_detection.py       camera + Haar cascade (how many faces can I see?)
  ball_detection.py       camera + Hough circle transform (how many round objects?)
  object_counting.py      camera + colour masking + contour counting (how many
                           objects of colour X can I see?)
       |
       v
Level 3 - combining camera-based perception with motor control/decision-making
  ball_tracking.py        camera (colour + circle detection) + motor control, to
                           rotate and keep a chosen coloured ball centred in view
  color_based_operation.py camera (colour + circle detection) + several possible
                           motor actions, chosen depending on the colour detected
```

Each level reuses the ideas (and often the exact function names/structure) from the
level before it, so it's worth reading them roughly in the order above, even if you
only need one of them for your task.

## Script descriptions

### Level 0 - hardware basics

* **`movements.py`** - Demonstrates all the basic ways to move the robot: `forward()`,
  `backward()`, `turn_left()`, `turn_right()`, the `curve_*` methods, `coast()` and
  `stop()`, each at full or partial speed. Good starting point to understand how the
  robot physically moves, with no sensors or camera involved.

* **`print_distance.py`** - Reads the ultrasound distance sensor repeatedly and prints
  the distance (in cm) to the console. Shows the two main ways of reading the sensor
  (quick/rough vs slower/more accurate), useful to understand how reliable/noisy the
  sensor is before using it for decision-making.

* **`show_underlighting.py`** - Demonstrates the different ways of setting the six
  underlighting LEDs (individually or all at once; using RGB values, hex colour codes,
  named tuples, or HSV values). Purely visual/output, no sensors or logic involved.

### Level 1 - combining two basic hardware features

* **`distance_lights.py`** - Reads the distance sensor and colours the underlighting
  from green (far) to red (close), fading through yellow in between. A simple example
  of turning a sensor reading into a visual/analogue response.

* **`follow_straight.py`** - Reads the distance sensor and drives forward/backward to
  keep an object at a fixed distance (`GOAL_DISTANCE`) directly in front of the robot.
  This is the core idea behind "keeping a fixed distance" needed for Task B, but
  without any camera/person-identification - it will follow *anything* in front of it.

* **`obstacle_avoidance.py`** - The most basic version of reactive obstacle avoidance:
  drive forward, and if the distance sensor detects something closer than
  `AVOID_DISTANCE`, stop and turn until the way is clear, then continue forward. This
  is a direct starting point for Task A, but it only uses the distance sensor (no
  camera) and always turns the same way - you will need to extend it (e.g. using the
  camera too, or making smarter turning/path decisions) to reliably navigate from
  point A to point B around multiple obstacles.

### Level 2 - introducing the camera and basic image processing

* **`color_detection.py`** - Captures a camera image and checks whether a known
  colour (red/yellow/green/blue) is present near the centre of the image, only when
  something is within `30cm` of the distance sensor. Lights the underlighting to match
  the colour found. Demonstrates the core HSV colour-masking technique used by several
  other scripts.

* **`face_detection.py`** - Captures a camera image and uses a pre-trained Haar
  cascade classifier (`haarcascade_frontalface_default.xml`) to detect and count
  human faces, lighting the underlighting green if any are found. An alternative,
  non-colour-based way of detecting "something" in the image - useful if you want to
  explore detection methods beyond colour masking.

* **`ball_detection.py`** - Captures a camera image (only when something is within
  `50cm`) and uses OpenCV's Hough circle transform to detect and count circular
  shapes, regardless of their colour. Shows how to detect objects by *shape* rather
  than by *colour*.

* **`object_counting.py`** - Captures a camera image, builds an HSV colour mask for a
  chosen colour (`color_wanted`), and counts how many separate blobs of that colour
  are present using contour detection, drawing a circle around each one and showing a
  live count. A generic template for "count how many objects of colour X are in view",
  useful as a starting point for any counting-based task - note that overlapping
  objects of the same colour may be merged into a single blob, which is a limitation
  you may need to address.

### Level 3 - combining perception with motor control/decision-making

* **`ball_tracking.py`** - Combines circle detection (`ball_detection.py`) and colour
  detection to find a ball of a chosen colour (`color_wanted`) and rotate the robot so
  that ball stays centred in the camera image, lighting the underlighting to match.
  This is the core idea behind "visually following/tracking a target" needed for
  Task B, combining shape and colour detection with motor control - but it does not
  manage distance (that part is demonstrated separately in `follow_straight.py`) or
  handle multiple similarly-coloured balls/decoys.

* **`color_based_operation.py`** - The most complete example: detects a coloured ball
  and, depending on its colour, performs a different pre-programmed action (drive
  forward, follow a square path, follow a circular path, or track the ball by
  rotating towards it). Demonstrates how to structure code that chooses between
  several possible robot behaviours based on what is perceived, and resets itself once
  the ball is removed from view.

## A note on function comments

Every function in these scripts has a short comment (docstring) explaining what it
does, its inputs and what it returns. Read these first if you are trying to understand
or reuse a function, before reading its full implementation.
