import math
import time
from booster_robotics_sdk_python.move_controller import MoveController

def main():
    robot = MoveController(ip="")

    try:
        print("\n>>> Task 1: Go to Start Point (1, 3)")
        robot.MoveToTarget(1.0, 3.0, 0.5)
        time.sleep(0.5)

        print("\n>>> Task 2: Relative Move (+1.0, 2.0)")
        robot.MoveToRelative(dx=1.0, dy=2.0, vel=0.5)
        time.sleep(0.5)

        print("\n>>> Task 3: Relative Move (-0.5, +0.5)")
        robot.MoveToRelative(dx=-0.5, dy=0.5, vel=0.4)
        time.sleep(0.5)

        print("\n>>> Task 4: Turn 90")
        robot.TurnAround(math.radians(90), 0.8)

        print("\n>>> Task 5: Go to 0,0 Point (0, 0)")
        robot.MoveToTarget(0.0, 0.0, 0.5)
        time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        robot.Close()

if __name__ == "__main__":
    main()