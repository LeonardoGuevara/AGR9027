# Github repository for AGR9027

Teaching materials for **AGR9027 - Agri-robotics in digitalisation** (MSc Agri-food
Technology, University of Lincoln). This repository contains example Python scripts,
sample images and laptop-friendly simulators to help you learn the basics of robotics
programming, computer vision and machine learning, and to prepare for **Assessment 2**
(a Trilobot robotics project with Tasks A, B and C).

No prior experience with robotics, Python or machine learning is assumed - the example
scripts are meant to be simple, heavily commented, and easy to copy and modify as a
starting point for your own work.

## Repository structure

```
AGR9027/
  sample_images/           <- Sample images for offline image-processing practice
  scripts/
    trilobot/               <- Example scripts for the real Trilobot robot (hardware)
    computer_vision/        <- Computer vision basics (taught session, coming soon)
    machine_learning/       <- Machine learning basics (taught session, coming soon)
  trilobot_simulator/      <- Laptop simulators for Tasks A, B and C (no robot needed)
```

* **[sample_images/](sample_images)** - A collection of sample images (e.g. coloured
  shapes) you can load and process with OpenCV without needing a camera or the robot, you can also print them and show them to the robot/laptop camera, useful for testing/practising image-processing code offline.

* **[scripts/](scripts)** - Example Python scripts, organised into three subfolders:
  * **[scripts/trilobot/](scripts/trilobot)** - Simple, generic scripts that run on
    the real Trilobot robot (motors, ultrasound sensor, underlighting, camera). See
    its own [README](scripts/trilobot/README.md) for a full description of every
    script and how they build on each other from basic to more complex. These are
    building blocks for Assessment 2, **not** finished solutions to Tasks A, B or C.
  * **[scripts/computer_vision/](scripts/computer_vision)** - *Placeholder, coming
    soon.* Will contain sample scripts covering computer vision basics, for a
    dedicated taught session.
  * **[scripts/machine_learning/](scripts/machine_learning)** - *Placeholder, coming
    soon.* Will contain sample scripts covering machine learning basics, for a
    dedicated taught session.

* **[trilobot_simulator/](trilobot_simulator)** - Simulators for Tasks A, B and C,
  plus a webcam vision harness, to let you develop and test your Assessment 2 logic
  on your own laptop without needing the real robot or access to the University lab.
  See its own [README](trilobot_simulator/README.md) for setup and usage.

## Assessment 2 overview

Assessment 2 asks you to use a real Trilobot robot to complete three tasks:

* **Task A** - Reactive obstacle avoidance: drive the robot from a start point to an
  end point while avoiding obstacles placed in between.
* **Task B** - Person following: keep the robot at a fixed distance (0.5 m) from a
  target person, while ignoring decoy people/objects.
* **Task C** - Ripe strawberry counting: count the number of ripe strawberries visible
  to the robot's camera, within an acceptable margin of error.

Refer to the official assessment brief provided on Blackboard for the full task
descriptions, marking criteria and submission requirements. The `scripts/trilobot/`
and `trilobot_simulator/` folders in this repository provide starting points and
practice tools only.

## Connecting to the real Trilobot (laboratory PCs)

> **Placeholder - to be completed.**
>
> This section will explain how to connect to and run code on the real Trilobot robot
> using the PCs in the University laboratory (e.g. network/Wi-Fi setup, SSH/remote
> access details, how to transfer your scripts to the robot, and any lab-specific
> login or booking instructions). Check back here before your first lab session, or
> ask your module leader if you need this information sooner.