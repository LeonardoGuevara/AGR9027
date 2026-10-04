# AGR9027 Trilobot simulators

These simulators let you develop and test your Assessment 2 code **on your own
laptop**, without needing the real Trilobot or access to the University lab. There is
**one simulator per task** (A, B, C) because each task needs a different test scenario.
You don't need any robotics/Raspberry Pi hardware or the `trilobot`/`picamera` Python
packages to run these - only a normal laptop with Python installed.

> **Important:** the simulators are simplified, 2D approximations meant to help you
> write and debug your decision-making logic (obstacle avoidance, following, counting)
> before trying it on the real robot. They are **not** a substitute for testing on the
> real Trilobot - the brief requires your final report and live demonstration to use
> the real robot.

See the [main README](../README.md) for an overview of the whole repository.

## Folder structure

```
trilobot_simulator/
  requirements.txt                   <- Python packages needed (see setup below)
  task_a_obstacle_avoidance/
    simulator_task_a.py              <- run this: point A -> point B, avoid obstacles
  task_b_person_following/
    simulator_task_b.py              <- run this: follow the target "person"
  task_c_strawberry_counting/
    simulator_task_c.py              <- run this: count ripe strawberries in a photo
```

Each `simulator_task_*.py` file is self-contained (just run it). Inside every file you
will find a clearly marked section:

```python
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EDIT FROM HERE  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
def robot_control(tbot):
    ...
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EDIT UNTIL HERE  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
```

That is the **only** part of each file you need to change. Everything above/below it
is the simulator itself (the fake robot, the fake world, the pygame/OpenCV window) -
you don't need to understand or modify it, although you are welcome to look at it.

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
will open showing the simulated world. Press `Q` (or close the window) to quit a
pygame simulator; for Task C, press `n`/`d`/`q` as shown in the on-screen instructions.

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

## What each simulator covers

* **Task A - obstacle avoidance** (`task_a_obstacle_avoidance/simulator_task_a.py`):
  a 2D arena with point A, point B and red obstacles of random size placed between
  them. You get a simulated ultrasound sensor (`tbot.read_distance()`) and a simulated
  forward camera view with the obstacles as red blobs. Increase `NUM_OBSTACLES` at the
  top of the file for a harder scenario. The simulator reports success (reached B
  within 0.5 m without collisions) or failure (collision) in the window and console.

* **Task B - person following** (`task_b_person_following/simulator_task_b.py`):
  a target "person" (coloured square) wanders around the arena along with some decoy
  people. The target's colour is printed in the console at start-up. Your code must
  keep the robot about 0.5 m from the target using the simulated camera (colour
  detection) and ultrasound sensor. Set `NUM_DECOYS` and `SIMILAR_DECOY_COLOURS = True`
  at the top of the file to try the harder levels described in the assessment brief.

* **Task C - strawberry counting**
  (`task_c_strawberry_counting/simulator_task_c.py`): generates a synthetic photo of a
  strawberry patch (red = ripe, green = unripe, with some overlap/occlusion) and lets
  you test a counting algorithm against it, showing the live count and the estimation
  error compared to the known ground truth. Press `d` to make the scene harder (more
  fruit, more overlap) to test the robustness of your algorithm, matching the
  "different levels of complexity" mentioned in the brief. Once your module leader adds
  real strawberry photos to [sample_images/](../sample_images), you can test your code
  directly on those (load with `cv2.imread(...)`) alongside the synthetic scenes.

## Limitations to keep in mind

* The simulators use simplified 2D physics/kinematics and synthetic "camera" images
  (flat-coloured shapes) rather than real pictures - real lighting, camera distortion,
  and background clutter will make the real robot more challenging.
* Task A/B camera images only contain the coloured shapes needed for detection; they
  do not look like real photographs.
* Collisions, sensor readings and the "success" conditions are approximations of the
  real robot's capabilities, used only to help you iterate quickly.
