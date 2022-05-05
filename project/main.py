#!/usr/bin/env pybricks-micropython
import sys

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


def pick_up_pallet():
    while forksensor.pressed() == False:
        robot.straight(-50)
    lift.run_until_stalled(30, then = Stop.HOLD, duty_limit = 60)

def predetermined_path(): # Follow predettermined path
    if light.color() == Color.BROWN:  # Välj rätt färg! Nu är det svart
        robot.drive(-30, 35)
    #elif light.color() == Color.BLUE:
        #robot.drive(-40, -35)
        #ev3.speaker.beep(frequency=500, duration=100)
        #lift.run_angle(100, 180, then=Stop.HOLD, wait=False)
    else:
        robot.drive(-30, -35)  # -35 betyder att den vänder sig -35 grader 
    
def predetermined_path_reflection():
    #while robot.distance >= 1000:
        #correction = (30 - light.reflection())*2
        #robot.drive(100,correction)
    if light.reflection() <= 92:
        robot.drive(-30, -35)
    else:
        robot.drive(-30, 35)

def green_path():
    while forksensor.pressed() == False:
        if light.color() == Color.GREEN:
            robot.drive(-20, -35)
        else: 
            robot.drive(-20, 35)

def blue_path():
    while forksensor.pressed() == False:
        if light.color() == Color.BLUE:
            robot.drive(-30, -35)
        else: 
            robot.drive(-30, 35)
        
def main():
    #while forksensor.pressed() == False:
        #predetermined_path()
    #robot.stop()
    #lift.run_angle(100, 180, then=Stop.HOLD, wait=False)
    while light.color() != Color.BLUE: 
        green_path()
    robot.stop()
    while forksensor.pressed() == False:
        blue_path()
        #ev3.speaker.beep(frequency=500, duration=100)
    lift.run_angle(100, 180, then=Stop.HOLD, wait=False)
    robot.stop()

    #descend.run_angle(5, 70, then=Stop.HOLD, wait=False)

    return 0

#if _name_ == '_main_':
#    sys.exit(main())

def drive(linecolor): # Follow green line (predetermined path)
    threshold = (linecolor + 92) / 2
    DRIVE_SPEED =-30
    PROPORTIONAL_GAIN = 1.2
    while light.color() != Color.BROWN:
        # Calculate the deviation from the threshold.
        deviation = light.reflection() - threshold

        # Calculate the turn rate.
        turn_rate = -(PROPORTIONAL_GAIN * deviation)

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        # color_list = []
        # color = color_sensor.color()
        # color_list.append(color)
        # You can wait for a short time or do other things in this loop.
        wait(10)
    robot.stop()
# Blå = 14-15
# Röd = 89-90

# grön = 12
# lila = 15
# röd = 84
# blå = 13
# brun = 20
# gul  70
# grå = 9
# svart = 4 
# vit = 84
def from_Purple_2_green():
    linecolor=15
    threshold = (linecolor + 84) / 2
    DRIVE_SPEED =-50
    PROPORTIONAL_GAIN = 1.2
    while light.color() != Color.BROWN:#Kör lila
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, -turn_rate)
        wait(10)
    robot.stop()
    linecolor=20 #kör brun 
    threshold = (linecolor + 84) / 2
    while light.reflection() != 12: 
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
    robot.stop()
    linecolor=12 #kör grön
    threshold = (linecolor + 84) / 2


def from_Purple_2_blue():
    linecolor=15
    threshold = (linecolor + 84) / 2
    DRIVE_SPEED =-50
    PROPORTIONAL_GAIN = 1.2
    while light.color() != Color.BROWN:#Kör lila
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, -turn_rate)
        wait(10)
    robot.stop()
    linecolor=20 #kör brun 
    threshold = (linecolor + 84) / 2
    n=0
    while light.reflection() != 12: 
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        if n>20:
            turn_rate=-(PROPORTIONAL_GAIN * deviation)
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)
        n=+1
    robot.stop()
    linecolor=12 #kör grön
    threshold = (linecolor + 84) / 2
    while True: 
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)


def pickup_pallet_new(linecolor):
    threshold = (linecolor + 9) / 2
    DRIVE_SPEED =-70
    PROPORTIONAL_GAIN = 1.2
    while forksensor.pressed() == False:
        deviation = light.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, -turn_rate)
        wait(10)
    robot.stop()
    lift.reset_angle(0)
    lift.run_angle(80,-180, then=Stop.HOLD, wait=True)
pickup_pallet_new(70)

#print(light.reflection())
#from_Purple_2_green()
#funk()
