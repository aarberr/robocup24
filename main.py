#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

# oggetto robot
class Robot:
    
    # costanti
    DEFAULT_SPEED = 40  # velocità predefinita per seguire la linea e altre operazioni
    COLOR_BLACK = 4  # valore di riflessione per il nero
    COLOR_WHITE = 45  # valore di riflessione per il bianco
    DEVIATION = 15  # possibile errore di misura oppure deviazione dalla linea
    RGB_UPPER = (100, 100, 100) # limite superiore per i valori RGB
    RGB_LOWER = (0, 0, 0) # limite inferiore per i valori RGB
    
    # inizializzazione del robot
    def __init__(self, motor_left, motor_right, color_left, color_center, color_right, ultrasonic):
        
        self.color_left = color_left  # sensore di colore sinistro (usato per il controllo del verde)
        self.color_center = color_center  # sensore di colore centrale (usato per segure la linea)
        self.color_right = color_right  # sensore di colore destro (usato per il controllo del verde)
        self.ultrasonic = ultrasonic  # sensore a ultrasuoni (usato per rilevare l'ostacolo)
        
        self.drive_base = DriveBase(self.motor_left, self.motor_right, wheel_diameter=47, axle_track=170)  # inizializzazione della drive base
        
        self.kp = 4.5  # coefficiente proporzionale di sterzata
        self.target = ((self.COLOR_BLACK + self.COLOR_WHITE) / 2) # valore target: valore medio tra bianco e nero
    
    # metodo per trovare la linea nera se fa fuori corso
    def find_line(self):
        self.drive_base.stop() # ferma la drive base
        self.drive_base.reset() # reimposta la distanza totale percorsa e l'angolo della drive base
        
        found_line = False # variabile di appoggio booleana per vedere se ho trovato la linea o meno
        
        #################################################
        # ciclo per anadre avanti                       #
        # per il regolamento l'interruzione delle linea #
        # nera può essere al massimo di 150 mm          #
        #################################################
        
        while self.drive_base.distance() <= 150:
            if self.color_center.reflection() <= 20: # se hai trovato la linea nera per valore di rifelssione
                found_line = True # imposta la variabile di appoggio su True
                self.drive_base.stop() # ferma la drive base
                break # esci dal ciclo
            else: # se ancora non hai ritrovato la linea nera
                self.drive_base.drive(self.DEFAULT_SPEED, 0) # vai avanti con la velocità predefinita
        
        #################################################################
        # se hai gia percorso 150mm e non hai trovato la linea nera     #
        # significa che sei andato fuori corso e dovevi girare a destra #
        #################################################################
        
        if not found_line:  
            self.drive_base.reset() # reimposta la distanza totale percorsa e l'angolo della drive base
            
            ###########################################################################
            # il ciclo prende in considerazione il valore assoluto della distanza     #
            # percorsa dala dive base per controllare di quanto deve tornare indietro #
            # tona indietro sempre della stessa distanca percorsa prima               #
            ###########################################################################
            
            while abs(self.drive_base.distance()) <= 150:
                self.drive_base.drive(-(self.DEFAULT_SPEED), 0) # torna indietro con la velocità predefinita
                
            #######################################################################
            # se non trovi la linea nera entro i valori di deviazione significa   #
            # che la linea nera sta a destra e devi ancora girare per incorciarla #
            #######################################################################
            
            while not ((self.COLOR_BLACK - self.DEVIATION) < self.color_center.reflection() < (self.COLOR_BLACK + self.DEVIATION)):
                self.drive_base.drive(0, 20) # gira a destra con la velocità predefinita stando fermo
    
    # metodo per seguire la linea
    def follow_line(self):
        
        #############################################################################
        # se vedi riflesso il bianco entro i valori di deviazione allora significa: #
        # 1) È un anglo retto verso destra                                          #
        # 2) La linea si è interrotta perche è tratteggiata                         #
        #############################################################################
        
        if  (self.COLOR_WHITE - self.DEVIATION) < self.color_center.reflection() < (self.COLOR_WHITE + self.DEVIATION): # in tal caso
            self.drive_base.stop() # ferma la drive base
            self.find_line() # inizia a cercare per la linea
        else: # se sei sulla linea
            deviation = self.color_center.reflection() - self.target # calcola di quanto stai deviando dal valore target
            turn_rate = deviation * self.kp # calcola il rapporto do sterzata di conseguenza
            self.drive_base.drive(self.DEFAULT_SPEED, turn_rate) # vai avanti mentre giri con il rapporto di sterzata calcolato prma
    
    # metodo per cervare il verde
    def search_green(self):
        ################################################################################
        # se uno dei due sensori vede il verde allora significa:                       #
        # 1) stai effettivamente vedendo il verde                                      #
        # 2) stai in un'itersezione dove il contrasto tra bianco e nero emula il verde # 
        # 3) C'è un punto angloso che emula il verde                                   #
        ################################################################################
        
        if self.color_left.color() == Color.GREEN or self.color_right == Color.GREEN: # in tal caso
            self.drive_base.stop() # ferma la drive base
            wait(50) # aspetta
            self.drive_base.straight(5) # vai avanti per 5 mm
            
            ###########################################################################
            # andare avanti dovrebbe eliminare l'errore di rilevazione del caso 2 e 3 #
            # quindi adesso verifichiamo effettivamente che ci sia il verde           #
            ###########################################################################
            
            if self.color_left.color() == Color.GREEN or self.color_right.color() == Color.GREEN: # se ancora vediamo il verde
                return True 
            else: # altrimenti significa che era un errore di rilevazione da parte del sensore
                return False
    
    # metodo per verificare la direzione indicata dall'incrocio
    def verify_intersection(self):
        left_color = self.color_left.rgb() # lista dei valori rgb per il sesnore sinistro
        right_color = self.color_right.rgb() # lista dei valori rgb per il sensore destro
        
        #####################################################################################
        # le seguenti condizioni verificacno che il valore rilevato si entro i limiti dove: #
        # RGB_UPPER è il limite superiore consentito per il canale del colore considerato   #
        # RGB_LOWER è il limite inferiore consentito per il canale del colore considerato   #
        # se il valore rilevato in quel canale rispetta tutti i limiti sigifica che è verde #
        #####################################################################################
        
        left_green = all(self.RGB_LOWER[i] <= left_color[i] <= self.RGB_UPPER[i] for i in range(3))
        right_green = all(self.RGB_LOWER[i] <= right_color[i] <= self.RGB_UPPER[i] for i in range(3))
            
        
        if left_green and not(right_green):
            return 1 # gira a sinistra
        elif not(left_green) and right_green:
            return 2 # gira a destra
        elif left_green and right_green:
            return 3 # torna indietro
        else:
            return 0 # l'intersezione non c'è
        
    # metodo per girare all'intersezione
    def turn_intersection(self, direction):
        
        ##############################################################################
        # tutti i rami della seguente condizione si basabìno sulle stesso principio: #
        # 1) la drive base si ferma temporaneamente                                  #
        # 2) si va in avanti per compensare la sterzata                              #
        # 3) in base al numero che viene passato si gira nella direzione indicata    #
        ##############################################################################
        self.drive_base.stop()
        self.drive_base.straight(100)
        
        if direction == 1:
            self.drive_base.turn(-70)
        elif direction == 2:
            self.drive_base.turn(70)
        elif direction == 3:
            self.drive_base.turn(180)
            self.drive_base.straight(100)
    
    # metodo pe evitare l'ostacolo nella direzione indicata
    def avoid_obstacle(self, pass_direction):
        
        ###############################################################################################
        # entrambi i rami della seguente condizioni funzionano allo stesso modo con l'eccezione       #
        # che girano in parti opposte in base alla direzione ndella quale devono aggiarare l'ostacolo #
        # 1) al drive base si ferma                                                                   #
        # 2) si gira verso l'esterno dell'ostacolo                                                    # 
        # 3) si provede dritto fino a una distanza sicura                                             #
        # 4) si gira venro la direzione della linea                                                   #
        # 5) si va avanti finché non si è superato l'ostacolo                                         #
        # 6) si fira verso l'interno                                                                  #
        ###############################################################################################
        
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
        
        # vai avanti finche non incontri la linea nera
        while not(self.COLOR_BLACK - self.DEVIATION <= self.color_center.reflection() <=self.COLOR_BLACK + self.DEVIATION):
            self.drive_base.drive(self.DEFAULT_SPEED, 0)  
    
    # loop principale
    def run(self):
        while True:
            if self.ultrasonic.distance() <= 50: # controlla la distanza, se è minore della soglia c'è l'ostacolo
                self.avoid_obstacle(1) # direzione per evitare l'ostacolo: 1 = sinistra, 0 = destra
            else:
                found_green = self.search_green() # controlla se vedi il verde
                
                if found_green: # se vero
                    
                    intersection = self.verify_intersection() # verifica la direzione dell'intersezione
                    
                    if intersection == 0: # se l'intersezione non c'è 
                        self.follow_line()  # segui la linea
                    else: # se l'intesezione c'è
                        self.turn_intersection(intersection) # gira nella direzione indicata
                else: # se il verde non c'è
                    self.follow_line() # segui la linea
                
                wait (10) # aspeta prima del prossimo ciclo


ev3 = EV3Brick() # init oggetto brick
ev3.speaker.beep() # verifico l'init con un beep

eugenio = Robot(Motor(Port.A), Motor(Port.D), ColorSensor(Port.S1), ColorSensor(Port.S2), ColorSensor(Port.S4), UltrasonicSensor(Port.S3)) # init oggetto robot
eugenio.run() # inizio programma
