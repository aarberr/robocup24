#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

class Robot:
   SPEED_DEFAULT = 300
   MIN_SPEED = 15
   TURN_DEFAULT = 100
   COLOR_WHITE = 54
   COLOR_BLACK = 4
   DEVIATION = 7
   MIN_ERROR = 20
   RGB_UPPER = (6, 36, 18)
   RGB_LOWER = (2, 12, 0)
   
   def __init__(self, motorLeft, motorRight, colorLeft, colorRight, ultrasonic):
      self.motorLeft = motorLeft
      self.motorRight = motorRight
      self.colorLeft = colorLeft
      self.colorRight = colorRight
      self.ultrasonic = ultrasonic
      
      self.driveBase = DriveBase(self.motorLeft, self.motorRight, wheel_diameter = 140, axle_track = 170)
      # self.driveBase = DriveBase(self.motorLeft, self.motorRight, wheel_diameter = 45, axle_track = 220)
      self.brick = EV3Brick()
      
      self.kp = 2.6
      self.ki = 0
      self.kd = 0
      
      self.integral = 0
      self.prevError = 0
      
   def followLine(self):
      error = sum(self.colorLeft.rgb()) - sum(self.colorRight.rgb())
      if -self.MIN_ERROR <= error <= self.MIN_ERROR:
         if self.colorLeft.color() == Color.RED and self.colorRight.color() == Color.RED:
            self.driveBase.stop()
         self.driveBase.drive(self.SPEED_DEFAULT, 0)
      else:
         self.integral += error
         derivate = error - self.prevError
         turnRate = (error * self.kp) + (self.integral * self.ki) + (derivate * self.kd)
         self.driveBase.drive(30, turnRate)
   
   def findLine(self):
      print("searching for line")
      self.driveBase.reset()
      foundLine = False
      
      while abs(self.driveBase.angle()) <= 150:
         if sum(self.colorRight.rgb()) <= (self.COLOR_BLACK + self.DEVIATION):
            self.driveBase.stop()
            foundLine = True
            break
         else:
            self.driveBase.drive(self.MIN_SPEED, self.TURN_DEFAULT)
      
      print("line not found on right\n   searching on left")
      if not foundLine:
         self.driveBase.reset()
         
         self.driveBase.reset()
         while abs(self.driveBase.angle()) <= 150:
            self.driveBase.drive(-self.MIN_SPEED, self.TURN_DEFAULT)            
         while sum(self.colorLeft.rgb()) <= (self.COLOR_BLACK + self.DEVIATION) or sum(self.colorRight.rgb()) <= (self.COLOR_BLACK + self.DEVIATION):
            self.driveBase.drive(self.MIN_SPEED, 0)
   
   def searchForGreen(self):
      greenLeft = True
      greenRight = True
      
      for i in range(3):
         if not(self.RGB_LOWER[i] <= self.colorLeft.rgb()[i] <= self.RGB_UPPER[i]):
               greenLeft = False
               
         if not(self.RGB_LOWER[i] <= self.colorRight.rgb()[i] <= self.RGB_UPPER[i]):
               greenRight = False
               
      if greenLeft or greenRight:
         self.driveBase.stop()
         print("possible intersection")
         wait(10)
         self.driveBase.straight(10)
         wait(10)
         
         confirmGreenLeft = True
         confirmGreenRight = True
         
         for i in range(3):
               if not(self.RGB_LOWER[i] <= self.colorLeft .rgb()[i] <= self.RGB_UPPER[i]):
                  confirmGreenLeft = False
                  
               if not(self.RGB_LOWER[i] <= self.colorRight.rgb()[i] <= self.RGB_UPPER[i]):
                  confirmGreenRight = False
                  
         if confirmGreenLeft or confirmGreenRight:
               print("intersection found")
               return True
         else:
               print("intersection not found")
               return False
      else:
         return False
   
   def indicateDirection(self):
      
      if (self.colorLeft.color() == Color.GREEN) and not (self.colorRight.color() == Color.GREEN):
         print("turning left")
         return 1
      elif not (self.colorLeft.color() == Color.GREEN) and (self.colorRight.color() == Color.GREEN):
         print("turning right")
         return 2
      elif (self.colorLeft.color() == Color.GREEN) and (self.colorRight.color() == Color.GREEN):
         print("going backward")
         return 3
      elif not (self.colorLeft.color() == Color.GREEN) and not (self.colorRight.color() == Color.GREEN):
         print("no intersection") 
         return 0
   
   def turnIntersection(self, direction):
      if direction == 1:
         self.driveBase.straight(150)
         while sum(self.colorLeft.rgb()) >= (self.COLOR_BLACK + self.DEVIATION):
            self.driveBase.drive(0, -self.TURN_DEFAULT)
      elif direction == 2:
         self.driveBase.straight(150)
         while sum(self.colorRight.rgb()) >= (self.COLOR_BLACK + self.DEVIATION):
            self.driveBase.drive(0, self.TURN_DEFAULT)
      if direction == 3:
            self.driveBase.straight(150)
            self.driveBase.turn(720)
            self.driveBase.straight(150)
   
   def pass_left(self):
      print("passing left")
      self.driveBase.stop()
      self.driveBase.turn(-350)
      self.driveBase.stop()
      self.driveBase.straight(550)
      self.driveBase.stop()
      self.driveBase.turn(360)
      self.driveBase.stop()
      self.driveBase.straight(1300)
      self.driveBase.stop()
      self.driveBase.turn(360)
      self.driveBase.stop()
   
   def pass_right(self):
      print("passing right")
      self.driveBase.stop()
      self.driveBase.turn(360)
      self.driveBase.stop()
      self.driveBase.straight(530)
      self.driveBase.stop()
      self.driveBase.turn(-360)
      self.driveBase.stop()
      self.driveBase.straight(1300)
      self.driveBase.stop()
      self.driveBase.turn(-360)
      self.driveBase.stop()

   def avoidObstacle(self, pass_direction):
      if sum(self.colorLeft.rgb()) > sum(self.colorRight.rgb()):
         while abs(sum(self.colorLeft.rgb()) - sum(self.colorRight.rgb())) > 0:
            self.driveBase.drive(0, self.TURN_DEFAULT)
      else:
         while abs(sum(self.colorLeft.rgb()) - sum(self.colorRight.rgb())) > 0:
            self.driveBase.drive(0, -self.TURN_DEFAULT)
         
      if pass_direction == 1:
         self.pass_left()
      else:
         self.pass_right()

      while sum(self.colorLeft.rgb()) >= (self.COLOR_BLACK + self.DEVIATION) and sum(self.colorRight.rgb()) >= (self.COLOR_BLACK + self.DEVIATION):
         self.driveBase.drive(self.SPEED_DEFAULT, 0)
      self.driveBase.straight(50)
      self.driveBase.stop()
      if pass_direction == 1:
         self.driveBase.turn(-360)
      else:
         self.driveBase.turn(360)
      self.driveBase.stop()
         
   
   def run(self):
      while True:
         if self.ultrasonic.distance() <= 80:
            print("obstacle")
            self.avoidObstacle(1)
         else:
            foundGreen = self.searchForGreen()
            
            if foundGreen:
               direction = self.indicateDirection()
               
               if direction != 0:
                  self.turnIntersection(direction)
               else:
                  self.followLine()
            else:
               self.followLine()
         
         wait(10)

eugenio = Robot(Motor(Port.A), Motor(Port.D), ColorSensor(Port.S1), ColorSensor(Port.S2), UltrasonicSensor(Port.S3))
eugenio.run()
