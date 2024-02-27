#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

class data_collector:
    default_speed = 30
    def __init__(self, motor_left, motor_right, color_left, color_center, color_right, ultrasonic):
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.color_left = color_left
        self.color_center = color_center
        self.color_right = color_right
        self.ultrasonic = ultrasonic
        
        self.drive_base = DriveBase(self.motor_left, self.motor_right, wheel_diameter=47, axle_track=170)
    
    def calibrate_white(self):
        white_reflection_values = list()
        
        for i in range(300):
            self.drive_base.drive(self.default_speed, 0)
            white_reflection_values.append(self.color_center.reflection())

        white_medium_value = sum(white_reflection_values) / len(white_reflection_values)
        
        print(f"Il valore medio per la riflessione del colore bianco è: {white_medium_value}")
        
        white_reflection_value.clear()
        
    def calibrate_black(self):
        balck_reflection_values = list()
        
        for i in range(300):
            self.drive_base.drive(self.default_speed, 0)
            black_reflection_values.append(self.color_center.reflection())

        black_medium_value = sum(white_reflection_values) / len(white_reflection_values)
        
        print(f"Il valore medio per la riflessione del colore bianco è: {black_medium_value}")
        
        black_reflection_value.clear()
    
    def calibrate_green_left(self):
        left_green_reflection_values = list()
        
        for i in range(5):
            for j in range(50):
                left_green_reflection_values.append(self.color_left.rgb())
        
        rgb_medium_values = [0, 0, 0]
        
        for i in range(len(left_green_reflection_values)):
            for j in range(3):
                rgb_medium_values[j] += left_green_reflection_values[i][j]
        
        for i in range(3):
            rgb_medium_values[i] / len(left_green_reflection_values)

        print(f"Valore medio canale rosso: {rgb_medium_values[0]} \nValore medio canale verde: {rgb_medium_values[2]} \nValore medio canale blu {rgb_medium_values[3]}")

        left_green_reflection_values.clear()

    def calibrate_green_right(self):
        right_green_reflection_values = list()
        
        for i in range(5):
            for j in range(50):
                right_green_reflection_values.append(self.color_right.rgb())
        
        rgb_medium_values = [0, 0, 0]
        
        for i in range(len(right_green_reflection_values)):
            for j in range(3):
                rgb_medium_values[j] += left_green_reflection_values[i][j]
        
        for i in range(3):
            rgb_medium_values[i] / len(right_green_reflection_values)

        print(f"Valore medio canale rosso: {rgb_medium_values[0]} \nValore medio canale verde: {rgb_medium_values[2]} \nValore medio canale blu {rgb_medium_values[3]}")
        
        right_green_reflection_values.clear()
