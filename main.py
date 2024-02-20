#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

# inizializzazione dell'oggetto robot
class Robot:
  
  # costanti di sistema (DA CALIBRARE)
  DEFAULT_SPEED = 30 # velocità di default
  COLOR_BLACK = 0 # valore di riflessione del nero
  COLOR_WHITE = 100 # valore di riflessione del bianco
  DEVIATION = 5 # errore di misura
  
  # inizializzazione dell'oggetto
  def __init__(self, motor_left, motor_right, color_left, color_center, color_right, ultrasonic):
    
    # dichiarazione dei motori
    self.motor_left = motor_left # mtore di sinistra
    self.motor_right = motor_right # motore di destra
    
    # dichiarazione dei sensori
    self.color_left = color_left # sensore di colore sinistro (controllo verde)
    self.color_center = color_center # sensore di colore centrale (seguilinea)
    self.color_right = color_right # sensore di colore destro (controllo verde)
    
    # dichiarazione della drive base (drive base è un oggetto)
    self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter=55, axle_track=127) 
    
    self.kp = 1 # coefficiente proporzionale di sterzata
    self.target = (self.COLOR_BLACK + self.COLOR_WHITE) / 2 # valore target del seguilinea
    
  # metodo per seguire la linea
  def follow_line(self):
    
    # se il sensore destro e il sensore sinistro vede il bianco e il sensore centrale vede il bianco entro l'errore di misura
    if  (self.COLOR_WHITE - self.DEVIATION) < self.color_center.reflection() < (self.COLOR_WHITE + self.DEVIATION):
      self.drive_base.stop()
      self.motor_left.hold()
      self.motor_right.hold()
      
      self.drive_base.reset()
      
      found_line = False
      
      while self.drive_base.distance() <= 150:
        if (self.COLOR_BLACK - self.DEVIATION) < self.color_center.reflection() < (self.COLOR_BLACK + self.DEVIATION):
          found_line = True
          self.drive_base.stop()
          self.motor_left.hold()
          self.motor_right.hold()
          break
        else:
          self.drive_base.drive(self.DEFAULT_SPEED, 0)
          
      
      if found_line == False:
        self.drive_base.reset()
        while abs(self.drive_base.distance()) <= 150:
          self.drive_base.drive(-self.DESFAULT_SPEED, 0)
        
        while not ((self.COLOR_BLACK - self.DEVIATION) < self.color_center.reflection() < (self.COLOR_BLACK + self.DEVIATION)):
          self.drive_base(0, 20)
          
    else:
      # calacola la deviazione dalla linea in base a quanto cambia dal valore target
      deviation = self.target - self.color_center.reflection()
      turn_rate = deviation * self.kp # calcola il rapporto di sterzata
      self.drive_base.drive(self.DEFAULT_SPEED, turn_rate) # gira con il rapporto di setrzata a velocità predefinita
      
  # metodo per rilevare l'intersezione
  def detect_intersection(self):
    if self.color_left.color() == Color.GREEN and self.color_right.color() == Color.GREEN:
      return 1 # indietro
    elif self.color_left.color() == Color.GREEN and not (self.color_right.color() == Color.GREEN):
      return 2 # sinistra
    elif not (self.color_left.color() == Color.GREEN) and self.color_right.color() == Color.GREEN:
      return 3 # destra
    elif not (self.color_left.color() == Color.GREEN) and not (self.color_right.color() == Color.GREEN):
      return 0 # continua
  
  # metodo per girare   
  def turn_intersection(self, direction):
    if direction == 2: # se la direzione è sinistra
      self.motor_left.hold() # ferma motore sinistro
      self.motor_right.hold() # ferma motore destro
      time.sleep(0.3) # aspetta
      self.drive_base.turn(90) # fira di 90 gradi a sinistra
    elif directiont == 3: # se la direzione è destra
      self.motor_left.hold() # ferma motore sinistro 
      self.motor_right.hold() # ferma motore destro
      time.sleep(0.3) # aspetta
      self.drive_base.turn(-90) # gira di 90 gradi a destra
    elif direction == 1:
      self.motor_left.dc(self.DEFAULT_SPEED) # imposta velocità destra 
      self.motor_right.dc(self.DEFAULT_SPEED) # imposta velocità sinistra
      time.sleep(0.5) # aspetta
      self.drive_base.turn(180) # gira di 180 gradi a sinistra
      
  # loop principale
  def run(self):
    while True:
      # controlla la distanza dell'ultrasuoni
      if self.ultrasonic.distance() < 50:
        # self.avoid_obstacle() # evita l'ostacolo
      else: 
        intersection_result = self.detect_intersection() # rileva l'ostacolo
        # se non c'è l'ostacolo segui la linea
        if intersection_result == 0:
          self.follow_line()
        else:
          # se l'intersezione c'è gira nella direzione dell'intersezione
          self.turn_intersection(intersection_result)
          
      wait(10)


ev3 = EV3Brick()  # Avvio l'oggetto EV3
ev3.speaker.beep()  # Verifico che si sia avviato facendo un beep

eugenio = Robot(Motor(Port.A), Motor(Port.D), ColorSensor(Port.S1), ColorSensor(Port.S4), ColorSensor(Port.S2), UltrasonicSensor(Port.S3))
eugenio.run()
