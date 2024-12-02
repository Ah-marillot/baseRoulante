from machine import Pin, PWM
import math
import utime

# Configuration de la LED intégrée
led = Pin(25, Pin.OUT)

# Allumer la LED
led.value(1)

# Configuration des broches pour les moteurs et les encodeurs
motorFL_pwm = PWM(Pin(4))  # Moteur avant gauche
motorRL_pwm = PWM(Pin(9))  # Moteur arrière gauche
motorRR_pwm = PWM(Pin(18))  # Moteur arrière droit
motorFR_pwm = PWM(Pin(26))  # Moteur avant droit

motorFL_dir_A = Pin(5, Pin.OUT) # Moteur avant gauche
motorFL_dir_B = Pin(6, Pin.OUT)
motorRL_dir_A = Pin(7, Pin.OUT) # Moteur arrière gauche 
motorRL_dir_B = Pin(8, Pin.OUT)
motorRR_dir_A = Pin(19, Pin.OUT) # Moteur arrière droit
motorRR_dir_B = Pin(20, Pin.OUT)
motorFR_dir_A = Pin(21, Pin.OUT) # Moteur avant droit
motorFR_dir_B = Pin(22, Pin.OUT)

encoderFL_A = Pin(2, Pin.IN, Pin.PULL_UP) # Encodeur avant gauche
encoderFL_B = Pin(3, Pin.IN, Pin.PULL_UP)
encoderRL_A = Pin(10, Pin.IN, Pin.PULL_UP) # Encodeur arrière gauche
encoderRL_B = Pin(11, Pin.IN, Pin.PULL_UP)
encoderRR_A = Pin(16, Pin.IN, Pin.PULL_UP) # Encodeur arrière droit
encoderRR_B = Pin(17, Pin.IN, Pin.PULL_UP)
encoderFR_A = Pin(27, Pin.IN, Pin.PULL_UP) # Encodeur avant droit
encoderFR_B = Pin(28, Pin.IN, Pin.PULL_UP)


# Constantes du système
PWM_FREQ = 1000
WHEEL_RADIUS = 0.05  # rayon des roues en mètres
ROBOT_WIDTH = 0.2    # largeur du robot en mètres (centre à centre des roues)
ROBOT_LENGTH = 0.2   # longueur du robot en mètres
ENCODER_RESOLUTION = 1000  # Résolution en ticks par rotation

# Initialisation des PWM
motorFL_pwm.freq(PWM_FREQ)
motorRL_pwm.freq(PWM_FREQ)
motorRR_pwm.freq(PWM_FREQ)
motorFR_pwm.freq(PWM_FREQ)

# Variables pour les encodeurs
encoder_counts_A = [0, 0, 0, 0]


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

# Fonction pour configurer la direction et la vitesse des moteurs
def set_motor(motor_pwm, motor_dir_A, motor_dir_B, speed):
    if speed >= 0:
        motor_dir_A.value(1)
        motor_dir_B.value(0)
        motor_pwm.duty_u16(int(speed * 65535))
    else:
        motor_dir_A.value(0)
        motor_dir_B.value(1)
        motor_pwm.duty_u16(int(-speed * 65535))

# Fonction pour contrôler les roues Mecanum
def move(x_speed, y_speed, rotation_speed):
    m1_speed = y_speed + x_speed + rotation_speed
    m2_speed = y_speed - x_speed - rotation_speed
    m3_speed = y_speed - x_speed + rotation_speed
    m4_speed = y_speed + x_speed - rotation_speed

    max_speed = max(abs(m1_speed), abs(m2_speed), abs(m3_speed), abs(m4_speed), 1)
    m1_speed /= max_speed
    m2_speed /= max_speed
    m3_speed /= max_speed
    m4_speed /= max_speed

    set_motor(motorRL_pwm, motorFL_dir_A, motorFL_dir_B, m1_speed)
    set_motor(motorFL_pwm, motorRL_dir_A, motorRL_dir_B, m2_speed)
    set_motor(motorRR_pwm, motorRR_dir_A, motorRR_dir_B, m3_speed)
    set_motor(motorFR_pwm, motorFR_dir_A, motorFR_dir_B, m4_speed)

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

# Exemple d'utilisation
try:
    #go_to_position(1.0, 0, ramp_rate=0.05, max_speed=0.5)  # Aller à (1 m, 1 m)
    set_motor(motorFL_pwm, motorFL_dir_A, motorFL_dir_B, 0.5)
except KeyboardInterrupt:
    move(0, 0, 0)
    print("Arrêt du robot.")
