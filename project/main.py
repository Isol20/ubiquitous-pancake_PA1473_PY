#!/usr/bin/env pybricks-micropython
import sys
import time

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

lift = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE, gears = [12, 36])
descend = Motor(Port.A, positive_direction=Direction.CLOCKWISE, gears = [12, 36])

ev3 = EV3Brick()
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, wheel_diameter=47, axle_track = 120)
light = ColorSensor(Port.S3) 
forksensor = TouchSensor(Port.S1)
sensor = Ev3devSensor(Port.S1)
Ultrasensor = UltrasonicSensor(Port.S4)

# FÄRGER REFLECTION
# grön = 12
# lila = 15
# röd = 84 fel
# blå = 13
# brun = 20
# gul  70
# grå = 92
# svart = 4 
# vit = 84


def drive(linecolor): # Follow line (predetermined path)
    threshold = (linecolor + 84) / 2
    DRIVE_SPEED =-30
    PROPORTIONAL_GAIN = 1.2
    while True:
        # Calculate the deviation from the threshold.
        deviation = light.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = -(PROPORTIONAL_GAIN * deviation)

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
    robot.stop()

def from_green_2_blue(): #Drives from the green lines and drives to the blue line
    linecolor=1
    threshold = (linecolor + 84) / 2
    DRIVE_SPEED =-50
    PROPORTIONAL_GAIN = 1.2
    while light.color() != Color.BROWN:#Kör grön
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
    countdown_turn(3)
    linecolor=20 #kör brun 
    threshold = (linecolor + 84) / 2
    #while light.rgb() != (10, 18, 27): 
    while light.reflection() != 13:
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, -turn_rate)
        wait(10)
    print("OUT")
    countdown_turn(2)
    linecolor=13 #kör blå
    threshold = (linecolor + 84) / 2
    while light.color() != Color.BLACK: 
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, -turn_rate)
        wait(10)
    robot.stop()

def from_Purple_2_blue(): #Drives from Purple to Blue line
    linecolor=12
    threshold = (linecolor + 84) / 2
    DRIVE_SPEED =-50
    PROPORTIONAL_GAIN = 1.2
    while light.color() != Color.BROWN:#Kör lila
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
    robot.stop()
    linecolor=20 #kör brun 
    threshold = (linecolor + 84) / 2
    while light.reflection() != 13: 
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        wait(10)
    robot.stop()
    linecolor=13 #kör blå
    threshold = (linecolor + 84) / 2
    while True: 
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)

def pickup_pallet_grey(linecolor): #Picks up pallet at grey platform by following yellow line
    threshold = (linecolor + 9) / 2
    DRIVE_SPEED =-70
    PROPORTIONAL_GAIN = 1.2
    #while countdown(10) > 0:
    while True:
        while forksensor.pressed() == False:
            deviation = light.reflection() - threshold
            turn_rate = PROPORTIONAL_GAIN * deviation
            robot.drive(DRIVE_SPEED, -turn_rate)
            wait(10)
        robot.stop()
        lift.reset_angle(0)
        lift.run_angle(200,180, then=Stop.HOLD, wait=True)

def countdown_turn(t): #Turns the robot to find new color
    while t:
        mins, secs = divmod(t, 60)
        timer = '{:02d}:{:02d}'.format(mins, secs)
        print(timer, end="\r")
        time.sleep(1)
        robot.drive(-20, 180)
        t -= 1

def main():
    #drive(13) #Följ blå Test
    #drive(20) #Följ lila Test
    #pickup_pallet_grey(70) #test
    #from_Purple_2_blue() #TEst
    from_green_2_blue()
    pickup_pallet_grey(70)
    #pickup_pallet_new(70) #test
    #print(light.rgb()) #test
    return 0

if _name_ == '_main_':
    sys.exit(main())
