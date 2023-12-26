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
    
    # Dichiarazione delle costanti
    SPEED_NORMAL = 30  # Velocità predefinita
    LIGHT_THRESHOLD_HIGHER = 50  # Limite riflesso canale rosso e blu
    LIGHT_THRESHOLD_LOWER = 40  # Minimo canale verde
    
    # Inizializzazione dell'oggetto robot
    def __init__(self, motor_left, motor_right, light_left, light_right, ultrasonic):
        
        # Assegnazione di motori e sensori
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.light_left = light_left
        self.light_right = light_right
        self.ultrasonic = ultrasonic

        # Assegnazione delle variabili del PID
        self.kp = 1  # Costante proporzionale
        self.ki = 0  # Costante integrale
        self.kd = 0  # Costante derivativa
        self.prev_error = 0  # Errore precedente
        self.integral = 0  # Valore integrale

    # Metodo per seguire la linea (PID)
    def follow_line(self):
        
        # Calcolo dell'errore
        error = sum(self.light_left.rgb()) - sum(self.light_right.rgb())  # Somma rgb_sinistro - somma rgb_destro

        # Verifico se i due sensori vedono lo stesso colore BIANCO/NERO
        if -15 <= error <= 15:
            # Se VERO, vado dritto
            self.motor_left.dc(self.SPEED_NORMAL)
            self.motor_right.dc(self.SPEED_NORMAL)
        else:
            # Se FALSO, calcolo la velocità da assegnare ai motori
            self.integral += error  # Calcolo l'integrale tenendo conto del nuovo errore
            derivative = error - self.prev_error  # Calcolo della derivata come differenza dell'errore attuale con quello precedente
            velocity = (error * self.kp) + (self.integral * self.ki) + (derivative * self.kd)  # Calcolo la velocità con la somma pesata di errore, integrale e derivata 
            self.motor_left.dc(self.SPEED_NORMAL + velocity)  # Assegno la velocità al motore sinistro
            self.motor_right.dc(self.SPEED_NORMAL - velocity)  # Assegno la velocità al motore destro
            self.prev_error = error  # Aggiorno il valore dell'errore precedente che mi servirà nel loop successivo
    
    # Metodo per verificare la presenza delle intersezioni
    def detect_intersection(self):
        
        # Ottengo i valori dei sensori
        color_left = self.light_left.rgb()
        color_right = self.light_right.rgb()

        # Se il sensore SINISTRO VEDE il VERDE e il sensore DESTRO NON VEDE il VERDE
        if (
            (color_left[0] <= self.LIGHT_THRESHOLD_LOWER and color_left[1] >= self.LIGHT_THRESHOLD_HIGHER and color_left[2] <= self.LIGHT_THRESHOLD_LOWER) and
            not (color_right[0] <= self.LIGHT_THRESHOLD_LOWER and color_right[1] >= self.LIGHT_THRESHOLD_HIGHER and color_right[2] <= self.LIGHT_THRESHOLD_LOWER)
        ):
            return 'left'  # Comando: gira a SINISTRA
        
        # Se il sensore SINISTRO NON VEDE il VERDE e il sensore DESTRO VEDE il VERDE
        elif (
            not (color_left[0] <= self.LIGHT_THRESHOLD_LOWER and color_left[1] >= self.LIGHT_THRESHOLD_HIGHER and color_left[2] <= self.LIGHT_THRESHOLD_LOWER) and
            (color_right[0] <= self.LIGHT_THRESHOLD_LOWER and color_right[1] >= self.LIGHT_THRESHOLD_HIGHER and color_right[2] <= self.LIGHT_THRESHOLD_LOWER)
        ):
            return 'right'  # Comando: gira a DESTRA
        
        # Se il sensore SINISTRO VEDE il VERDE e il sensore DESTRO VEDE il VERDE
        elif (
            (color_left[0] <= self.LIGHT_THRESHOLD_LOWER and color_left[1] >= self.LIGHT_THRESHOLD_HIGHER and color_left[2] <= self.LIGHT_THRESHOLD_LOWER) and
            (color_right[0] <= self.LIGHT_THRESHOLD_LOWER and color_right[1] >= self.LIGHT_THRESHOLD_HIGHER and color_right[2] <= self.LIGHT_THRESHOLD_LOWER)
        ):
            return 'back'  # Comando: gira INDIETRO
        
        # Se NESSUNO dei casi precedenti è verificato, allora NON CE L'INTERSEZIONE
        else:
            return 'noIntersection'  # Comando: NON ESISTE INTERSEZIONE
    
    # Metodo per girare (viene chiamato solo se CE l'intersezione)
    def turn_intersection(self, direction):
        
        # Se comando: gira a SINISTRAå
        if direction == 'left':
            self.motor_left.run_time(-150, 1000, then=Stop.HOLD, wait=True)
            self.motor_right.run_time(150, 1000, then=Stop.HOLD, wait=True)
        
        # Se comando: gira a DESTRA
        elif direction == 'right':
            self.motor_right.run_time(-150, 1000, then=Stop.HOLD, wait=True)
            self.motor_left.run_time(150, 1000, then=Stop.HOLD, wait=True)
            
        # Se comando: gira INDIETRO
        else:
            self.motor_right.run_time(-150, 1000, then=Stop.HOLD, wait=True)
            self.motor_left.run_time(150, 1000, then=Stop.HOLD, wait=True)
            
            
    # Metodo per evitare l'ostacolo       
    def avoid_obstacle(self):
        
        # Gira a SINISTRA di 45 gradi
        self.motor_left.run_time(-150, 1000, then=Stop.HOLD, wait=True)
        self.motor_right.run_time(150, 1000, then=Stop.HOLD, wait=True)
        
        # Vai avanti per 1 secondo
        self.motor_left.dc(30)
        self.motor_right.dc(30)
        time.sleep(1)
        
        # Gira a DESTRA di 45 gradi
        self.motor_right.run_time(-150, 1000, then=Stop.HOLD, wait=True)
        self.motor_left.run_time(150, 1000, then=Stop.HOLD, wait=True)
        
        # Vai avanti finché uno dei due sensori non vede la linea nera
        while sum(self.light_left.rgb()) >= 50 or sum(self.light_left.rgb()) >= 50:
            self.motor_left.dc(30)
            self.motor_right.dc(30)
            
    # Metodo principale 
    def run(self):
        while True:
            
            # Controlla se ce l'ostacolo
            if self.ultrasonic.distance() <= 50:
                
                # Se VERO, chiama il metodo per evitarlo
                self.avoid_obstacle()
            else:
                
                # Se FALSO, chiama il metodo per controllare l'incrocio
                intersection_result = self.detect_intersection()  # Ricorda il risultato

                # Controlla il risultato
                if intersection_result != 'noIntersection':
                    
                    # Se l'intersezione non ce, segui la linea normalmente
                    self.follow_line()
                else:
                    # Se l'incrocio ce, chiama il metodo per girare e passa la direzione
                    self.turn_intersection(intersection_result)
                    
# Main

ev3 = EV3Brick()  # Avvio l'oggetto EV3
ev3.speaker.beep()  # Verifico che si sia avviato facendo un beep

robot = Robot(Motor(Port.A), Motor(Port.D), ColorSensor(Port.S1), ColorSensor(Port.S4), UltrasonicSensor(Port.S3))  # Avvia l'oggetto robot

robot.run()  # Chiama il metodo principale per avviare il programma
