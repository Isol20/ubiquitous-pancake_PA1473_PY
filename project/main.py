#!/usr/bin/env pybricks-micropython
import sys

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase

lift = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears = [12, 36])
descend = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE, gears = [12, 36])

forksensor = TouchSensor(Port.S1)

left_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE, gears = [12, 20])
right_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE, gears = [12, 20])

bobbe = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 120)

def pick_up_pallet():
    while forksensor.pressed() == False:
        bobbe.straight(50)
    lift.run_until_stalled(30, then = Stop.HOLD, duty_limit = 60)


def main():
    descend.run_until_stalled(-30, then = Stop.HOLD, duty_limit = 60)
    while forksensor.pressed() == False:
        bobbe.straight(50)
    #bobbe.straight(500)
    #lift.run_time(60, 3000, then=Stop.HOLD, wait=True)
    lift.run_until_stalled(90, then = Stop.HOLD, duty_limit = 10000)
    #descend.run_until_stalled(30, then = Stop.HOLD, duty_limit = 60)
    #pick_up_pallet()
    return 0

if __name__ == '__main__':
    sys.exit(main())


