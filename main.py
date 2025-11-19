#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()


left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
line_sensor = ColorSensor(Port.S2)
line_sensor2 = ColorSensor(Port.S1)

robot = DriveBase(left_motor, right_motor, 55.5, 104)


threshold = 10
kp = 1.2
for i in range(1):
    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        if right_reflection < 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        if right_reflection > 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

robot.straight(5)

now_dir = 1
target_dir = 2

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

for i in range(2):
    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        if right_reflection < 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        if right_reflection > 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    
robot.straight(5)

now_dir = 1
target_dir = 4

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

for i in range(2):
    while True:
        left_reflection = line_sensor2.reflection()
        right_reflection = line_sensor.reflection()
        if right_reflection < 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    while True:
        left_reflection = line_sensor2.reflection()
        right_reflection = line_sensor.reflection()
        if right_reflection > 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

robot.straight(5)

now_dir = 1
target_dir = 4

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

for i in range(2):
    while True:
        left_reflection = line_sensor2.reflection()
        right_reflection = line_sensor.reflection()
        if right_reflection < 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    while True:
        left_reflection = line_sensor2.reflection()
        right_reflection = line_sensor.reflection()
        if right_reflection > 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

robot.straight(5)


now_dir = 1
target_dir = 2

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
wait(10)

for i in range(1):
    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        if right_reflection < 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    while True:
        left_reflection = line_sensor2.reflection()
        right_reflection = line_sensor.reflection()
        if right_reflection > 25:
            robot.stop()    

            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)
