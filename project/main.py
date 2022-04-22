#!/usr/bin/env pybricks-micropython
import sys

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

lift = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears = [12, 36])
descend = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE, gears = [12, 36])

forksensor = TouchSensor(Port.S1)
light = ColorSensor(Port.S3)

left_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE, gears = [12, 20])
right_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE, gears = [12, 20])

robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 120)

def pick_up_pallet():
    while forksensor.pressed() == False:
        robot.straight(50)
    lift.run_until_stalled(30, then = Stop.HOLD, duty_limit = 60)

def predetermined_path():
    while True:
        if light.color() == Color.BLACK:  ## Välj rätt färg! Nu är det svart
            robot.drive(100, 35)
        else:
            robot.drive(100, -35)  ## -35 betyder att den vänder sig mot den svarta linjen

def main():
    #predetermined_path()
    #descend.run_until_stalled(-30, then = Stop.HOLD, duty_limit = None)
    #while forksensor.pressed() == False:
        #robot.straight(50)
    #robot.straight(500)
    #lift.run_time(60, 100000, then=Stop.HOLD, wait=True)
    #lift.run_target(10,45, then=Stop.HOLD, wait=True)
    #lift.run_until_stalled(90, then = Stop.HOLD, duty_limit = None)
    #descend.run_until_stalled(30, then = Stop.HOLD, duty_limit = 60)
    #pick_up_pallet()
    #robot.drive(100, 35)
    return 0

if __name__ == '__main__':
    sys.exit(main())


