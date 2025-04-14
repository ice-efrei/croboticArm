from time import sleep  # Importer la bibliothèque de gestion du temps

import RPi.GPIO as GPIO
import math

class Servo:

  SERVO_EXTENSION_PIN = 13  # Broche pour le servomoteur d'extension

  SERVO_HAUTEUR_PIN = 18  # Broche pour le servomoteur de hauteur
  def __init__(self):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.SERVO_EXTENSION_PIN, GPIO.OUT)
    GPIO.setup(self.SERVO_HAUTEUR_PIN, GPIO.OUT)

    # Fréquence PWM pour les servos (généralement 50Hz)
    self.pwm_extension = GPIO.PWM(self.SERVO_EXTENSION_PIN, 50)
    self.pwm_hauteur = GPIO.PWM(self.SERVO_HAUTEUR_PIN, 50)

    # Démarrage du PWM avec un duty cycle initial de 0 (aucun signal)
    self.pwm_extension.start(0)
    self.pwm_hauteur.start(0)

  def set_servo_angle(self, angles):
    duty1 = 2.5 + (int(angles[0]) / 180.0) * 10
    duty2 = 2.5 + (int(angles[1]) / 180.0) * 10

    self.pwm_extension.ChangeDutyCycle(duty1)
    self.pwm_hauteur.ChangeDutyCycle(duty2)

  def move_to_coordinates(self, x, y, l1=19, l2=19):
    """
    Déplace le bras robotique vers les coordonnées (x, y)
    en calculant les angles nécessaires pour les servomoteurs.
    l1 et l2 sont les longueurs des segments du bras.
    """
    d = math.sqrt(x**2 + y**2)
    if d > (l1 + l2) or d < abs(l1 - l2):
        print("Position hors de portée")
        return

    cos_angle2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    angle2 = math.acos(cos_angle2) * 180 / math.pi
    angle1 = math.atan2(y, x) - math.atan2(l2 * math.sin(math.radians(angle2)), l1 + l2 * math.cos(math.radians(angle2)))
    angle1 = math.degrees(angle1)

    self.set_servo_angle([angle1, angle2])
    sleep(0.5)