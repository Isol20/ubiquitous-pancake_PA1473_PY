#!/usr/bin/env pybricks-micropython
import sys

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, Colorsensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase

lift = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears = [12, 36])

left_motor = Motor(Port.C, positive_direction=Direction.CUNTERCLOCKWISE, gears = [12, 20])
right_motor = Motor(Port.B, positive_direction=Direction.CUNTERCLOCKWISE, gears = [12, 20])

bobbe = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 120)

def main():

    bobbe.straight(1000)
    lift.run_untill_stall(50, then = Stop.HOLD)

    return 0

if __name__ == '__main__':
    sys.exit(main())