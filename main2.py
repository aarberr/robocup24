#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

class Robot:

  DEFAULT_SPEED = 40
  COLOR_BLACK = 4
  COLOR_WHITE = 45
  DEVIATION = 20
  RGB_UPPER = [10, 100, 2]
  RGB_LOWER = [0, 15, 20]

  def __init__(self, motor_left, motor_right, color_left, color_center, color_right, ultrasonic):
    self.color_left = color_left
    self.color_center = color_center
    self.color_right = color_right
    self.ultrasonic = ultrasonic
    self.motor_left = motor_left
    self.motor_right = motor_right
    self.drive_base = DriveBase(self.motor_left, self.motor_right, wheel_diameter=47, axle_track=170)
    self.brick = EV3Brick()
    self.kp = 2
    self.target = ((self.COLOR_BLACK + self.COLOR_WHITE) / 2)

  def find_line(self):
    self.drive_base.stop()
    self.drive_base.reset()
    found_line = False
    while self.drive_base.angle() <= 150:
      if self.color_center.reflection() <= 20:
        found_line = True
        self.drive_base.stop()
        break
      else:
        self.drive_base.drive(10, 30)
    if not found_line:
      self.drive_base.reset()
      while abs(self.drive_base.turn()) <= 150:
          self.drive_base.drive(-10, -30)
        while not (self.COLOR_BLACK - self.DEVIATION) < self.color_reflection.reflection() < (self.COLOR_BLACK + self.DEVIATION):
            self.drive_base.drive(self.DEFAULT_SPEED, 0)

  def follow_line(self):
    if (self.COLOR_WHITE - self.DEVIATION) < self.color_center.reflection() < (self.COLOR_WHITE + self.DEVIATION):
      self.drive_base.stop()
      self.find_line()
    else:
      deviation = self.color_center.reflection() - self.target
      turn_rate = deviation * self.kp
      self.drive_base.drive(self.DEFAULT_SPEED, turn_rate)

  def search_green(self):
    if self.color_left.color() == Color.GREEN or self.color_right == Color.GREEN:
      self.drive_base.stop()
      wait(500)
      self.drive_base.straight(5)
      wait(500)
      if self.color_left.color() == Color.GREEN or self.color_right.color() == Color.GREEN:
        return True 
      else:
        return False
    else:
      return False

  def verify_intersection(self):
    left_color = self.color_left.rgb()
    right_color = self.color_right.rgb()
    if (self.RGB_LOWER[0] <= left_color[0] <= self.RGB_UPPER[0]) and (self.RGB_LOWER[1] <= left_color[1] <= self.RGB_UPPER[1]) and (self.RGB_LOWER[2] <= left_color[2] <= self.RGB_UPPER[2]):
      left_green = True
    else:
      left_green = False
    if (self.RGB_LOWER[0] <= right_color[0] <= self.RGB_UPPER[0]) and (self.RGB_LOWER[1] <= right_color[1] <= self.RGB_UPPER[1]) and (self.RGB_LOWER[2] <= right_color[2] <= self.RGB_UPPER[2]):
      right_green = True
    else:
      right_green = False
    if left_green and not(right_green):
      return 1
    elif not(left_green) and right_green:
      return 2
    elif left_green and right_green:
      return 3
    else:
      return 0

  def turn_intersection(self, direction):
    self.drive_base.stop()
    self.drive_base.straight(100)
    if direction == 1:
      self.drive_base.turn(-45)
    elif direction == 2:
      self.drive_base.turn(45)
    elif direction == 3:
      self.drive_base.turn(135)
      self.drive_base.straight(100)
    while not(self.COLOR_BLACK - self.DEVIATION < self.color_center.reflection() < self.COLOR_BLACK + self.DEVIATION):
      if direction == 1:
        self.drive_base.drive(0, -20)
      else:
        self.drive_base.drive(0, 20)

  def avoid_obstacle(self, pass_direction):
    if pass_direction == 1:
      self.drive_base.stop()
      wait(50)
      self.drive_base.turn(-45)
      wait(50)
      self.drive_base.straight(150)
      wait(50)
      self.drive_base.turn(45)
      wait(50)
      self.drive_base.straight(200)
      wait(50)
      self.drive_base.turn(45)
      wait(50)
    else:
      self.drive_base.stop()
      wait(50)
      self.drive_base.turn(45)
      wait(50)
      self.drive_base.straight(150)
      wait(50)
      self.drive_base.turn(-45)
      wait(50)
      self.drive_base.straight(200)
      wait(50)
      self.drive_base.turn(-45)
      wait(50)
    while not(self.COLOR_BLACK - self.DEVIATION <= self.color_center.reflection() <=self.COLOR_BLACK + self.DEVIATION):
      self.drive_base.drive(self.DEFAULT_SPEED, 0)  
    
  def run(self):
    while True:
      if self.ultrasonic.distance() <= 50:
        self.avoid_obstacle(1)
      else:
        found_green = self.search_green()
        if found_green:
          intersection = self.verify_intersection()
          if intersection == 0:
            self.follow_line()
          else:
            self.turn_intersection(intersection)
        else:
          self.follow_line()
        wait (10)

eugenio = Robot(Motor(Port.A), Motor(Port.D), ColorSensor(Port.S1), ColorSensor(Port.S2), ColorSensor(Port.S3), UltrasonicSensor(Port.S4))
eugenio.run()
