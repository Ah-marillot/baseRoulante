from machine import Pin, PWM, I2C
import math
import utime

# from motor import Motor  # Importer la classe Motor

PWM_FREQ = 1000  # Constante pour la fréquence PWM

# Adresse I2C de l'esclave
I2C_SLAVE_ADDR = 0x42

# Définition des constantes pour les broches des moteurs
MOTOR_FL_PWM = 4
MOTOR_FL_DIR = 5
MOTOR_FL_ENC = 6
MOTOR_FL_DIR_MULT = -1

MOTOR_RL_PWM = 9
MOTOR_RL_DIR = 7
MOTOR_RL_ENC = 8
MOTOR_RL_DIR_MULT = 1

MOTOR_RR_PWM = 18
MOTOR_RR_DIR = 19
MOTOR_RR_ENC = 20
MOTOR_RR_DIR_MULT = 1

MOTOR_FR_PWM = 26
MOTOR_FR_DIR = 21
MOTOR_FR_ENC = 22
MOTOR_FR_DIR_MULT = -1

# Constantes du système
PWM_FREQ = 1000
WHEEL_RADIUS = 0.05  # rayon des roues en mètres
ROBOT_WIDTH = 0.2    # largeur du robot en mètres (centre à centre des roues)
ROBOT_LENGTH = 0.2   # longueur du robot en mètres
ENCODER_RESOLUTION = 1000  # Résolution en ticks par rotation



class Motor:
    def __init__(self, pwm_pin, dir_pin_A, dir_pin_B, direction=1):
        self.pwm = PWM(Pin(pwm_pin))
        self.dir_A = Pin(dir_pin_A, Pin.OUT)
        self.dir_B = Pin(dir_pin_B, Pin.OUT)
        self.direction = direction
        self.pwm.freq(PWM_FREQ)

    def set_speed(self, speed):
        speed *= self.direction
        if speed >= 0:
            self.dir_A.value(1)
            self.dir_B.value(0)
            self.pwm.duty_u16(int(speed * 65535))
        else:
            self.dir_A.value(0)
            self.dir_B.value(1)
            self.pwm.duty_u16(int(-speed * 65535))



# Configuration de la LED intégrée
led = Pin(25, Pin.OUT)

# Allumer la LED
led.value(1)

# Les moteurs sont tournent vers l'intérerieur pour des valeurs positives
# si ont veut que tout les moteurrs tournent dans le meme sent pour une valeur positive il faut inverser le sens de rotation de deux moteurs

# Variables pour les encodeurs
encoder_counts_A = [0, 0, 0, 0]

# Déclaration des encodeurs
encoderFL_A = Pin(2, Pin.IN, Pin.PULL_UP) # Encodeur avant gauche
encoderFL_B = Pin(3, Pin.IN, Pin.PULL_UP)
encoderRL_A = Pin(10, Pin.IN, Pin.PULL_UP) # Encodeur arrière gauche
encoderRL_B = Pin(11, Pin.IN, Pin.PULL_UP)
encoderRR_A = Pin(16, Pin.IN, Pin.PULL_UP) # Encodeur arrière droit
encoderRR_B = Pin(17, Pin.IN, Pin.PULL_UP)
encoderFR_A = Pin(27, Pin.IN, Pin.PULL_UP) # Encodeur avant droit
encoderFR_B = Pin(28, Pin.IN, Pin.PULL_UP)

# Callback pour compter les impulsions des encodeurs
def encoder_callback(pin):
    global encoder_counts
    if pin == encoderFL_A and encoderFL_B.value():
        encoder_counts_A[0] += 1
    elif pin == encoderFL_B and encoderFL_A.value():
        encoder_counts_A[0] -= 1
    elif pin == encoderRL_A and encoderRL_B.value():
        encoder_counts_A[1] += 1
    elif pin == encoderRL_B and encoderRL_A.value():
        encoder_counts_A[1] -= 1
    elif pin == encoderRR_A and encoderRR_B.value():
        encoder_counts_A[2] += 1
    elif pin == encoderRR_B and encoderRR_A.value():
        encoder_counts_A[2] -= 1
    elif pin == encoderFR_A and encoderFR_B.value():
        encoder_counts_A[3] += 1
    elif pin == encoderFR_B and encoderFR_A.value():
        encoder_counts_A[3] -= 1

encoderFL_A.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)
encoderFL_B.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)
encoderRL_A.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)
encoderRL_B.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)
encoderRR_A.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)
encoderRR_B.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)
encoderFR_A.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)
encoderFR_B.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)


# Configuration des moteurs
motorFL = Motor(MOTOR_FL_PWM, MOTOR_FL_DIR, MOTOR_FL_ENC, MOTOR_FL_DIR_MULT)  # Moteur avant gauche
motorRL = Motor(MOTOR_RL_PWM, MOTOR_RL_DIR, MOTOR_RL_ENC, MOTOR_RL_DIR_MULT)  # Moteur arrière gauche
motorRR = Motor(MOTOR_RR_PWM, MOTOR_RR_DIR, MOTOR_RR_ENC, MOTOR_RR_DIR_MULT)  # Moteur arrière droit
motorFR = Motor(MOTOR_FR_PWM, MOTOR_FR_DIR, MOTOR_FR_ENC, MOTOR_FR_DIR_MULT)  # Moteur avant droit

# Fonction pour contrôler les roues Mecanum
def move(x_speed, y_speed, rotation_speed):
    # Calcul des vitesses des moteurs
    fl_speed = x_speed + y_speed + rotation_speed
    rl_speed = x_speed - y_speed - rotation_speed
    rr_speed = x_speed + y_speed - rotation_speed
    fr_speed = x_speed - y_speed + rotation_speed

    max_speed = max(abs(fl_speed), abs(rl_speed), abs(rr_speed), abs(fr_speed), 1)
    fl_speed /= max_speed
    rl_speed /= max_speed
    rr_speed /= max_speed
    fr_speed /= max_speed

    motorFL.set_speed(fl_speed)
    motorRL.set_speed(rl_speed)
    motorRR.set_speed(rr_speed)
    motorFR.set_speed(fr_speed)

# Fonction pour calculer la distance restante
def calculate_distance(target_x, target_y, current_x, current_y):
    return math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

# Fonction pour générer une rampe progressive de vitesse
def ramp_speed(current_speed, target_speed, ramp_rate):
    if abs(current_speed - target_speed) < ramp_rate:
        return target_speed
    elif current_speed < target_speed:
        return current_speed + ramp_rate
    else:
        return current_speed - ramp_rate

# Fonction pour aller à une position (x, y)
def go_to_position(target_x, target_y, ramp_rate=0.1, max_speed=1.0):
    current_x, current_y = 0, 0  # Position actuelle (remplacer par des données d'odométrie si disponibles)
    current_speed = 0  # Vitesse initiale

    while calculate_distance(target_x, target_y, current_x, current_y) > 0.01:  # Tolérance de 1 cm
        distance = calculate_distance(target_x, target_y, current_x, current_y)
        
        # Calcul de l'angle vers la cible
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        x_speed = math.cos(angle_to_target) * current_speed
        y_speed = math.sin(angle_to_target) * current_speed

        # Ajustement de la vitesse avec une rampe
        target_speed = min(max_speed, distance)  # Limite la vitesse proche de la cible
        current_speed = ramp_speed(current_speed, target_speed, ramp_rate)

        # Mouvement du robot
        move(x_speed, y_speed, 0)  # Pas de rotation ici
        utime.sleep(0.1)

        # Mettre à jour la position actuelle (approximée ici)
        # Vous pouvez utiliser les encodeurs pour une mesure plus précise
        current_x += x_speed * 0.1
        current_y += y_speed * 0.1

    # Arrêter le robot
    move(0, 0, 0)

# Configuration de l'I2C en tant qu'esclave
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)

# Fonction pour traiter les instructions reçues
def process_instruction(instruction):
    try:
        parts = instruction.split()
        if parts[0] == 'M' and len(parts) == 4:
            x_speed = float(parts[1])
            y_speed = float(parts[2])
            rotation_speed = float(parts[3])
            move(x_speed, y_speed, rotation_speed)
        else:
            print("Instruction inconnue")
    except Exception as e:
        print("Erreur de traitement de l'instruction:", e)

# Boucle principale
run = True
while(run):
    if i2c.scan():
            try:
                data = i2c.readfrom(I2C_SLAVE_ADDR, 1)  # Lire 1 octet de données
                process_instruction(data)
            except OSError:
                pass
    utime.sleep(0.1)

