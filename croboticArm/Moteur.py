from time import sleep         # Importer la bibliotheque de gestion du temps 
import RPi.GPIO as GPIO        # Importer la bibliotheque de gestion des GPIO
import csv
import numpy as np
# 34 cm = 27000 pas

import smbus, time, math

class Moteur():    
    ENABLE =4 # Active les moteurs
    VITESSE_AXE = 0.00002
    VITESSE_ANGLE = 0.0001
    ACCELERATION = 1.003
    
    def __init__(self,STEP,DIR):
        self.STEP=STEP
        self.DIR=DIR
        print(self.DIR,self.STEP)
        GPIO.setmode(GPIO.BCM)        
        GPIO.setwarnings(False)        
        # Initialisation des ports        
        
        GPIO.setup(self.STEP, GPIO.OUT)     # GPIO STEP configuration en sortie
        GPIO.setup(self.DIR, GPIO.OUT)      # GPIO DIR configuration© en sortie
        GPIO.setup(self.ENABLE, GPIO.OUT)
        
        GPIO.output(self.DIR, GPIO.HIGH)
        GPIO.output(self.ENABLE, GPIO.HIGH)

        self.x=0
        self.nb_pas=0

    def one_step(self,direction, vitesse):
        try:
            # si c'est un moteur angle lire la valeur du capteur
            if (self.DIR== 24 or self.DIR== 22):
                angle = self.read_true_angle()
                print(f"DEBUG: angle lu sur le capteur AS5600 {self.DIR}: {angle}")
            print(f"DEBUG: one_step  direction={direction}, vitesse={vitesse} sur pin STEP={self.STEP}, DIR={self.DIR}")
            # S'assurer que ENABLE est activé (LOW active les moteurs dans la plupart des pilotes)
            
            if direction == 1:
                GPIO.output(self.DIR, GPIO.HIGH)
                print(f"HIGH")
            else:
                GPIO.output(self.DIR, GPIO.LOW)
                print(f"LOW")          
            GPIO.output(self.STEP, GPIO.HIGH)
            sleep(vitesse)
            GPIO.output(self.STEP, GPIO.LOW)
            sleep(vitesse)
            
        except Exception as e:
            print(f"ERREUR dans one_step: {e}")

    
                
class MoteurAxe(Moteur):
    FC=6 # fin de course
    def __init__(self,STEP,DIR):
        print("init moteur axe")
        super().__init__(STEP,DIR)
        GPIO.setup(self.FC, GPIO.IN)
        self.zero()
    
    def zero(self):
        print(GPIO.input(self.FC))
        while(GPIO.input(self.FC)==0):
            self.one_step(0,self.VITESSE_AXE)
        while(GPIO.input(self.FC)==1):
            self.one_step(1,self.VITESSE_AXE*4)
        for i in range(300):
            self.one_step(1,self.VITESSE_AXE*4)
        while(GPIO.input(self.FC)==0):
            self.one_step(0,self.VITESSE_AXE*6)
        self.x=0
        print("zero")
        
    def position(self,distance):
        mode=0
        tmpX1 = self.x
        tmpX2 = self.x
        if (distance > 34):
            distance = 34
        if (distance < 0):
            distance = 0
        nb = int(distance*27000/34)
        speed=self.VITESSE_AXE*10
        while(nb!=self.x and nb>=0):
            if (speed>self.VITESSE_AXE and mode ==0 and abs(nb-tmpX1)/2>abs(self.x-tmpX1)):
                speed = speed / self.ACCELERATION
                tmpX2 = self.x
            elif (mode == 0 and abs(nb-tmpX1)/2<=abs(self.x-tmpX1)):
                mode = 2
            elif mode == 0 :
                speed = self.VITESSE_AXE
                mode = 1
            if mode == 1 :
                if (abs(nb-self.x) < abs(tmpX2-tmpX1)):
                    speed = speed * self.ACCELERATION
            if mode == 2 :
                speed = speed * self.ACCELERATION

                
                
            if(nb>self.x):
                self.one_step(0,speed)     
                self.x+=1
            elif(nb<self.x):
                self.one_step(1,speed)     
                self.x-=1
    
    
class MoteurAngle(Moteur):
    sensor_AS5600 = 0x36
    numBus = 0
    def __init__(self,STEP,DIR,numBus,diff):
        self.numBus = numBus
        super().__init__(STEP,DIR)
        self.bus = smbus.SMBus(numBus)
        self.zero(diff)
        

    def zero(self, diff):
        max_attempts = 10
        attempts = 0
        while True:
            raw_angle = self.read_raw_angle()
            if raw_angle == -1:
                print(f"Abandon de la calibration : impossible de lire l'AS5600 sur le bus {self.numBus}.")
                break
            if raw_angle == int(diff*4096/360):
                break
            print(raw_angle)
            step_per_rev = 6400  # Nombre de pas par tour (dépend du moteur)
            gear_ratio = 1
            current_angle = self.read_true_angle()
            error = diff - current_angle
            direction = GPIO.HIGH if error > 0 else GPIO.LOW  # Choix du sens de rotation
            steps = int(abs(error) * (step_per_rev / 360) * gear_ratio)
            for i in range(steps):
                self.one_step(direction, self.VITESSE_ANGLE*3)
            attempts += 1
            if attempts >= max_attempts:
                print(f"Calibration interrompue après {max_attempts} tentatives.")
                break
        self.angle = 0
        self.x = 0
    

    def positionAngle(self,target_angle):
        step_per_rev = 6400  # Nombre de pas par tour (dépend du moteur)
        gear_ratio = 3
        mode=0
        tmpX1 = self.x
        tmpX2 = self.x
        speed=self.VITESSE_ANGLE*10
        nb = int(target_angle*step_per_rev/360*gear_ratio)
        speed=0.001
        while(nb!=self.x ):
            if (speed>self.VITESSE_ANGLE and mode ==0 and abs(nb-tmpX1)/2>abs(self.x-tmpX1)):
                speed = speed / self.ACCELERATION
                tmpX2 = self.x
            elif (mode == 0 and abs(nb-tmpX1)/2<=abs(self.x-tmpX1)):
                mode = 2
            elif mode == 0 :
                speed = self.VITESSE_ANGLE
                mode = 1
            if mode == 1 :
                if (abs(nb-self.x) < abs(tmpX2-tmpX1)):
                    speed = speed * self.ACCELERATION
            if mode == 2 :
                speed = speed * self.ACCELERATION

                
                
            if(nb>self.x):
                self.one_step(0,speed)     
                self.x+=1
            elif(nb<self.x):
                self.one_step(1,speed)     
                self.x-=1
        self.angle = target_angle

           
            
        
        
        
    # ------------------------------------------------------------------------ #
	#  github.com/Mathias-Wandel/AS5600-Raspberry-Pi-Python	

    def read_raw_angle(self):
        """
        Lit l'angle brut (0 à 4096) du capteur AS5600.
        Gestion d'erreur I2C : affiche un message si la lecture échoue.
        """
        for attempt in range(3):
            try:
                read_bytes = self.bus.read_i2c_block_data(self.sensor_AS5600, 0x0C, 2)
                return (read_bytes[0] << 8) | read_bytes[1]
            except OSError as e:
                print(f"Erreur I2C lors de la lecture de l'angle brut (tentative {attempt+1}/3) : {e}")
                sleep(0.1)
        print("Echec de lecture I2C sur le capteur AS5600 (bus {}), vérifiez le câblage ou l'alimentation.".format(self.numBus))
        return -1
        
    def read_true_angle(self):
        raw_angle = self.read_raw_angle()
        return float(raw_angle * 360.0 / 4096)
        
    def read_magnitude(self):
        """
        Lit la magnitude du champ magnétique du capteur AS5600.
        """
        read_bytes0 = self.bus.read_i2c_block_data(self.sensor_AS5600, 0x1B, 2)
        raw0 = (read_bytes0[0] << 8) | read_bytes0[1]
        return raw0
    # ------------------------------------------------------------------------ #
    def testAS5600(self,moteur):
       
        raw_angle = self.read_raw_angle()
        magnitude = self.read_magnitude()
        angle_deg = raw_angle * 360.0 / 4096

        print("Bus {:1d} : RawA: {:5d} | Magnitude: {:5d} | Angle: {:7.2f}°".format(moteur,raw_angle, magnitude, angle_deg))

