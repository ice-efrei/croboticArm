import RPi.GPIO as GPIO  
from time import sleep 
import smbus, time, math
from Moteur import *
import threading

# Adresse I2C par défaut du capteur AS5600



class Bras():
	PIN_AIMANT = 5

	def __init__(self):
		print("--------------------------INITIALISATION-----------------------")
		   
		GPIO.setmode(GPIO.BCM)         # Paramétrage de la numérotation des GPIO en mode BCM
		GPIO.setwarnings(True) 
		"""
		print(">>> Initialisation Aimant")
		GPIO.setup(self.PIN_AIMANT, GPIO.OUT)
		GPIO.output(self.PIN_AIMANT, GPIO.HIGH)
		"""
		print(">>> Initialisation Moteurs")
		
		self.moteurAxe=MoteurAxe(27,17)
		self.moteur1=MoteurAngle(23,22,1,266)
		self.moteur2=MoteurAngle(24,25,0,333)
		
		# self.coordonnees(0,12,-7)
	
	def test_as5600(self):
		print("--------------------------Test AS5600--------------------------")
		while(1):
			self.moteur1.testAS5600(1)
			self.moteur2.testAS5600(2)
			print("_________________________________________________________") 
			sleep(0.5)


	def activerAimant(self, state):
		if state==0:
			GPIO.output(self.PIN_AIMANT, GPIO.LOW)
		else :
			GPIO.output(self.PIN_AIMANT, GPIO.HIGH)

	def testMoteurAxe(self):
		print("--------------------------Test MOTEUR AXE--------------------------")
		while(1):
			x=float(input("Distance cm : "))
			self.moteurAxe.position(x)
			
			
	
	def testMoteurAngle(self):
		print("--------------------------Test MOTEUR ANGLE--------------------------")
		for i in range(0,6400):
			self.moteur1.one_step(-1,0.0005)
			
			
	
	def testAimant(self):
		print("--------------------------Test AIMANT--------------------------")
		while(1):
			a=int(input("entrez un chiffre : "))
			self.activerAimant(a)
	
	
	def testMoteur(self):
		self.moteur1.testAS5600()

	def coordonnees(self, x, y, z):
		longueur_bras = 25
		longueur_coude = 25
		#Calcul de l'angle du moteur 1
		distance = math.sqrt(z**2 + y**2)
		if distance == 0:
			print("Erreur : distance = 0")
			return
		a1 = math.atan2(z, y)
		a2 = math.acos((longueur_bras**2 + distance**2 - longueur_coude**2)/(2*longueur_bras*distance))
		a = a1 + a2
		a = math.degrees(a)
		#Calcul de l'angle du moteur 2
		b = math.acos((longueur_bras**2 + longueur_coude**2 - distance**2)/(2*longueur_bras*longueur_coude))
		b = math.degrees(b)
		a= 90 - a
		b = 90 - b + a

		
		self.moveMultiMoteurs(x, a, b)

	def moveMultiMoteurs(self,x,a,b): #x distance, a angle moteur1, b angle moteur2
		threads = []
		t1 = threading.Thread(target=self.moteur1.positionAngle, args=(-a,))
		threads.append(t1)
		t2 = threading.Thread(target=self.moteur2.positionAngle, args=(b,))
		threads.append(t2)
		t3 = threading.Thread(target=self.moteurAxe.position, args=(x,))
		threads.append(t3)

		for t in threads:
			t.start()
		for t in threads:
			t.join()
		print("Fin deplacement")
	def testmoveMultiMoteurs(self):
		print("--------------------------Test Multi Moteurs--------------------------")
		while(1):
			x=float(input("Distance cm : "))
			a=float(input("Angle moteur 1 : "))
			b=float(input("Angle moteur 2 : "))
			self.moveMultiMoteurs(x,a,b)

	def testcoordonnees(self):
		print("--------------------------Test coordonnees--------------------------")
		while(1):
			x=float(input("Distance x cm : "))
			a=float(input("Distance y cm : "))
			b=float(input("Distance z cm : "))
			self.coordonnees(x,a,b)
 
