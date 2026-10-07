# AGR9027 Trilobot simulators

These simulators let you develop and test your Assessment 2 code **on your own
laptop**, without needing the real Trilobot or access to the University lab. There is
**one simulator per task** (A, B, C) because each task needs a different test scenario,
plus a live-webcam vision harness.
You don't need any robotics/Raspberry Pi hardware or the `trilobot`/`picamera` Python
packages to run these - only a normal laptop with Python installed.

> **Important:** the simulators are simplified, 2D approximations meant to help you
> write and debug your decision-making logic (obstacle avoidance, following, counting)
> before trying it on the real robot. They are **not** a substitute for testing on the
> real Trilobot - the brief requires your final report and live demonstration to use
> the real robot.

## Simulator overview

Each simulator provides a different practice scenario. For all tasks, edit the
task's `student_control.py`; leave `simulator_task_*.py` unchanged. The official
assessment brief remains the authority for exact requirements and marking criteria.

* **Task A - obstacle avoidance**
  * **Objective:** Navigate from known point A to point B without colliding with
    obstacles.
  * **Scenario and display:** A 2D arena shows A, B, obstacles, the robot, transparent
    camera and ultrasound fields of view, underlighting, live distance reading and a
    320x240 synthetic BGR camera image. Obstacles are preferentially placed near the
    A-to-B route; their apparent camera size is projected from their configured
    radius and varies with distance. The camera range is 6 m.
  * **Student inputs:** `robot_control(tbot)` can read ultrasound distance with
    `tbot.read_distance()` and camera images with `tbot.get_camera_image()`. Students
    can process images for obstacle detections and use those results with distance
    readings to decide how to move. Reaching B requires more than reactive avoidance:
    students must estimate position and heading while travelling around obstacles.
    The Trilobot library and simulator do not provide odometry, so students need to
    implement an approximate motion estimate themselves.

* **Task B - person following**
  * **Objective:** Follow the designated person while distinguishing them from decoys
    and maintaining the required following distance (0.5 m).
  * **Scenario and display:** A 2D arena shows the robot, a target and decoys, camera
    and ultrasound fields of view, underlighting, live distance reading and a
    320x240 synthetic BGR camera image. People are rendered as simple human
    silhouettes with grey skin and colour-coded clothing; their apparent size is
    projected from their configured dimensions and varies with distance. The camera
    range is 6 m.
  * **Student inputs:** `robot_control(tbot, target_color)` can read ultrasound
    distance with `tbot.read_distance()` and camera images with
    `tbot.get_camera_image()`. Students can process images to detect and locate the
    target person, then use those results with distance readings to steer toward the
    correct person and manage following distance. A single-frame detection may
    confuse a decoy for the target; maintain a target track across successive frames
    and use motion/position continuity to help preserve the target's identity.

* **Task C - strawberry counting**
  * **Objective:** Identify and count the ripe strawberries visible in the image,
    excluding unripe and semi-ripe fruit as required by the assessment.
  * **Scenario and display:** The simulator generates synthetic strawberries with
    varied shapes, colours and ripeness, including overlapping fruit. The result
    window shows the student's detections, count and error; a second window shows the
    same scene with ground-truth ripe fruit marked.
  * **Student inputs:** `count_ripe_strawberries(image)` receives a synthetic BGR
    camera image and returns a count with a debug image. Students should use image
    processing to identify ripe fruit and distinguish/count individual berries,
    including when fruits overlap. There is no ultrasound distance input for this
    task.

See the [main README](../README.md) for an overview of the whole repository.

## Folder structure

```
trilobot_simulator/
  requirements.txt                   <- Python packages needed (see setup below)
  task_a_obstacle_avoidance/
    simulator_task_a.py              <- protected simulator: point A -> point B
    student_control.py               <- edit this: Task A robot and vision logic
  task_b_person_following/
    simulator_task_b.py              <- protected simulator: follow the target person
    student_control.py               <- edit this: Task B robot and vision logic
  task_c_strawberry_counting/
    simulator_task_c.py              <- protected simulator and scene generator
    student_control.py               <- edit this: Task C counting logic
  webcam_camera.py                   <- run student vision code on a laptop webcam
```

The `simulator_task_*.py` files contain the simulation code and call the corresponding
`student_control.py`. Students should make changes in `student_control.py`, not in the
simulator:

```python
# Task A/B: robot_control(tbot) or robot_control(tbot, target_color)
# Task C:   count_ripe_strawberries(image)
```

The simulator files can therefore be rerun or updated without overwriting student
logic. The same student vision functions can also be exercised with the webcam harness.

## Setup (do this once)

We recommend installing everything inside a **Python virtual environment** (often
called a "venv"). This keeps the packages needed for the simulator in a separate,
self-contained folder instead of mixing them with other Python projects on your
laptop, which avoids version conflicts and makes it easy to start fresh if something
goes wrong (just delete the `venv` folder and repeat step 2).

1. Install Python 3.10+ from [python.org](https://www.python.org/downloads/) (tick
   "Add Python to PATH" during installation on Windows) if you don't already have it.
2. Open a terminal **in the `trilobot_simulator` folder** and create the virtual
   environment (you only need to do this once):

   **Windows (PowerShell):**
   ```powershell
   cd trilobot_simulator
   python -m venv venv
   ```

   **macOS / Linux:**
   ```bash
   cd trilobot_simulator
   python3 -m venv venv
   ```

   This creates a `trilobot_simulator/venv` folder containing its own copy of Python.
   It is not part of the module materials, so it's fine (and recommended) to leave it
   out of any work you submit or commit to git.

3. Activate the virtual environment (you need to do this every time you open a new
   terminal to work on the simulator):

   **Windows (PowerShell):**
   ```powershell
   venv\Scripts\Activate.ps1
   ```
   > If you get an error about "running scripts is disabled on this system", run
   > `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` first, then try again.

   **Windows (cmd.exe):**
   ```bat
   venv\Scripts\activate.bat
   ```

   **macOS / Linux:**
   ```bash
   source venv/bin/activate
   ```

   Your terminal prompt should now start with `(venv)`, confirming the virtual
   environment is active.

4. With the virtual environment active, install the required packages:

   ```bash
   pip install -r requirements.txt
   ```

When you're done working, you can leave the virtual environment with the `deactivate`
command (or just close the terminal).

## Running a simulator

Make sure the virtual environment is activated first (prompt starts with `(venv)`;
if not, repeat step 3 above), then:

```bash
cd task_a_obstacle_avoidance
python simulator_task_a.py
```

(similarly for `task_b_person_following` and `task_c_strawberry_counting`). A window
will show the simulated map and camera view. For Tasks A and B, press `n` for a new
scene at the current level, `1`-`4` to select a level and regenerate, or `d` to advance
to the next level and regenerate. Press `Q`/`ESC` (or close the window) to quit. Task C
uses the same `n`, `1`-`4`, `d`, and `q`/`ESC` controls.

### Complexity levels

Set `COMPLEXITY_LEVEL = 1`, `2`, `3` or `4` near the top of each Task A/B simulator,
or press the matching number while it is running. Press `d` to advance to the next
level and regenerate the scene.
Task C starts at level 1 and lets you select levels with `1`-`4` while it is running.

* **Task A:** the levels generate 1, 3, 5 and 8 obstacles, respectively. Obstacles are
  sampled around the line from A to B, while keeping the start and goal areas clear.
* **Task B:** levels generate 0, 1, 3 and 5 extra people. All people start at the same
  distance from the robot and within the camera field of view; the target then walks
  away at a constant angle randomly selected from -20° to +20° for each scene, so it
  may walk straight ahead.
  Similar-colour decoys use different shades of the target colour.
  The hardest level includes at least two similar-colour decoys; level 3 includes one.
* **Task C:** the levels have 8, 12, 16 and 20 ripe berries to count, respectively.
  Total unripe and semi-ripe berries increase from 4 to 8, 14 and 20; semi-ripe
  counts are 0, 2, 6 and 12, and their yellow/red coloring varies. Overlap clusters
  contain no more than two berries and use matching ripeness classes, prioritizing
  ripe-with-ripe pairs. At level 4 every berry overlaps in a pair. The simulator
  displays the student's result and a ground-truth view with circles around ripe fruit
  in side-by-side windows.

### Live webcam vision

From the `trilobot_simulator` folder, run one of:

```powershell
python webcam_camera.py --task a
python webcam_camera.py --task b --target-colour RED
python webcam_camera.py --task c
```

The harness opens the laptop's default camera (use `--camera 1` to select another
device), calls the selected task's `process_camera_frame` in `student_control.py`, and
shows its output live. For Task B, choose `RED`, `YELLOW`, `GREEN` or `BLUE` with
`--target-colour`. This is a vision-only tool: it does not connect webcam frames to
the simulated robot's map or distance sensor. Press `Q` or `ESC` to close it.

## How the fake robot ("tbot") relates to the real Trilobot

Each simulator creates an object called `tbot` that supports the **same method names**
used in the example scripts in [scripts/trilobot/](../scripts/trilobot): `forward()`,
`backward()`, `turn_left()`, `turn_right()`, `set_motor_speeds()`, `read_distance()`,
`fill_underlighting()`, `read_button()`, etc. It also provides `tbot.get_camera_image()`
which returns a BGR image (a `numpy` array) you can process with the same OpenCV code
used in `scripts/trilobot/color_detection.py` / `scripts/trilobot/ball_tracking.py`.

This means the control logic you write and test in the simulator (the code inside
`robot_control`) should need only small changes to run on the real robot:

* Replace `tbot = SimTrilobot(...)` with `tbot = Trilobot()` (from `from trilobot import
  Trilobot`).
* Replace `tbot.get_camera_image()` with a real camera capture (see
  `scripts/trilobot/color_detection.py` for the `picamera` capture pattern).
* Remove the pygame window code (only needed for the simulator's visualisation).

## Limitations to keep in mind

* The simulators use simplified 2D physics/kinematics and synthetic camera images.
  Task A obstacles are red circular blobs sized by apparent distance; Task B uses
  simple grey-skin, coloured-clothing silhouettes. They are not photographs.
* The webcam harness can process real objects, printed pictures or fruit, but it has no
  simulated distance measurement and does not move the simulator robot. Real lighting,
  camera distortion and background clutter will still affect detections.
* Collisions, sensor readings and the "success" conditions are approximations of the
  real robot's capabilities, used only to help you iterate quickly.
