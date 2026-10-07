"""Run a task's student camera-vision code on live laptop webcam frames."""

import argparse
import importlib.util
from pathlib import Path

import cv2

TASK_MODULES = {
    "a": Path("task_a_obstacle_avoidance") / "student_control.py",
    "b": Path("task_b_person_following") / "student_control.py",
    "c": Path("task_c_strawberry_counting") / "student_control.py",
}
TARGET_COLOURS = ("RED", "YELLOW", "GREEN", "BLUE")


def load_student_module(task):
    module_path = Path(__file__).parent / TASK_MODULES[task]
    spec = importlib.util.spec_from_file_location(f"webcam_task_{task}", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load student camera module: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("a", "b", "c"), required=True,
                        help="which student's camera-vision function to run")
    parser.add_argument("--camera", type=int, default=0, help="webcam device index (default: 0)")
    parser.add_argument("--target-colour", choices=TARGET_COLOURS, default="RED",
                        help="Task B clothing colour to detect (default: RED)")
    args = parser.parse_args()

    module = load_student_module(args.task)
    capture = cv2.VideoCapture(args.camera)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(
            f"Could not open webcam index {args.camera}; check the device or try --camera 1."
        )

    title = f"AGR9027 - Task {args.task.upper()} live webcam vision"
    print(f"Live camera vision for Task {args.task.upper()}. Press Q or ESC to quit.")
    try:
        while True:
            success, frame = capture.read()
            if not success:
                raise RuntimeError("The webcam opened but failed to provide a frame.")
            if args.task == "b":
                debug_image, summary = module.process_camera_frame(frame, args.target_colour)
            else:
                debug_image, summary = module.process_camera_frame(frame)
            cv2.putText(debug_image, summary, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (255, 255, 255), 2)
            cv2.imshow(title, debug_image)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
