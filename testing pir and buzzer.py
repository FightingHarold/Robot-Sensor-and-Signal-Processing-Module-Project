import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Pin Definitions
LED_LEFT_PIN = 22
LED_RIGHT_PIN = 27
TRIG_PIN = 23
ECHO_PIN = 24
SERVO_PIN = 17
PIR_PIN = 25
BUZZER_PIN = 18

safe_threshold = 15  # Distance in cm

# Outputs - SETUP YOUR OUTPUTS BEFORE PWM INIT!
GPIO.setup(LED_LEFT_PIN, GPIO.OUT)
GPIO.setup(LED_RIGHT_PIN, GPIO.OUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)  # <- Make sure this is done before PWM!
GPIO.setup(BUZZER_PIN, GPIO.OUT)

GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(PIR_PIN, GPIO.IN)

# Initialize PWM AFTER setting up the pin as output
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)
buzzer_pwm = GPIO.PWM(BUZZER_PIN, 1000)
buzzer_pwm.start(0)

def buzzer_off():
    buzzer_pwm.ChangeDutyCycle(0)

def buzzer_on(duration=2):
    buzzer_pwm.ChangeDutyCycle(50)
    time.sleep(duration)
    buzzer_pwm.ChangeDutyCycle(0)

def obstacle_avoidance_alert():
    print("[Alert] obstacle avoidance alert - LEDs blinking at 1s rate")
    for i in range(3):
        GPIO.output(LED_LEFT_PIN, True)
        GPIO.output(LED_RIGHT_PIN, True)
        time.sleep(0.5)
        GPIO.output(LED_LEFT_PIN, False)
        GPIO.output(LED_RIGHT_PIN, False)
        time.sleep(0.5)
    print("[Alert] obstacle alert finished")

def intruder_alert():
    print("[Alert] Activating alarm")
    for i in range(4):
        GPIO.output(LED_LEFT_PIN, True)
        GPIO.output(LED_RIGHT_PIN, True)
        buzzer_pwm.ChangeDutyCycle(50)
        time.sleep(0.5)
        GPIO.output(LED_LEFT_PIN, False)
        GPIO.output(LED_RIGHT_PIN, False)
        buzzer_pwm.ChangeDutyCycle(0)
        time.sleep(0.5)
    buzzer_pwm.ChangeDutyCycle(0)
    print("[Intruder alert] Alarm finished")

def check_pir_sensor():
    return GPIO.input(PIR_PIN) == 1

def set_servo_angle(angle):
    angle = max(-90, min(90, angle))
    duty_cycle = (angle / 18.0) + 7.5
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)

def distance_measurement():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.000002)
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    pulse_start = time.time()
    pulse_end = time.time()
    timeout_start = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        if (time.time() - timeout_start) > 0.1:
            return 999
    timeout_end = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        if (time.time() - timeout_end) > 0.1:
            break
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

def forward():
    print("forward movement")
    GPIO.output(LED_LEFT_PIN, True)
    GPIO.output(LED_RIGHT_PIN, True)

def backwards():
    print("backward movement")
    for _ in range(10):
        GPIO.output(LED_LEFT_PIN, True)
        GPIO.output(LED_RIGHT_PIN, True)
        time.sleep(0.05)
        GPIO.output(LED_LEFT_PIN, False)
        GPIO.output(LED_RIGHT_PIN, False)
        time.sleep(0.05)

def turn_left():
    print("turning left")
    GPIO.output(LED_RIGHT_PIN, False)
    for _ in range(50):
        GPIO.output(LED_LEFT_PIN, True)
        time.sleep(0.01)
        GPIO.output(LED_LEFT_PIN, False)
        time.sleep(0.01)

def turn_right():
    print("turning right")
    GPIO.output(LED_LEFT_PIN, False)
    for _ in range(50):
        GPIO.output(LED_RIGHT_PIN, True)
        time.sleep(0.01)
        GPIO.output(LED_RIGHT_PIN, False)
        time.sleep(0.01)

def stop():
    print("Stopping")
    GPIO.output(LED_LEFT_PIN, False)
    GPIO.output(LED_RIGHT_PIN, False)

def scan_obstacle():
    print("\n[Scanning] obstacle detected Searching for safe route")
    distance_readings = {}
    print("Scanning right")
    set_servo_angle(90)
    distance_readings['right'] = distance_measurement()
    print(f" Right distance: {distance_readings['right']:.1f} cm")
    time.sleep(0.3)
    print("Scanning center")
    set_servo_angle(0)
    distance_readings['center'] = distance_measurement()
    print(f" Center distance: {distance_readings['center']:.1f} cm")
    time.sleep(0.3)
    print("Scanning left")
    set_servo_angle(-90)
    distance_readings['left'] = distance_measurement()
    print(f" Left distance: {distance_readings['left']:.1f} cm")
    time.sleep(0.3)
    set_servo_angle(0)
    time.sleep(0.3)
    if distance_readings['right'] > safe_threshold:
        print(f"[DECISION] RIGHT is clear ({distance_readings['right']:.1f} cm)")
        return 1
    elif distance_readings['left'] > safe_threshold:
        print(f"[DECISION] LEFT is clear ({distance_readings['left']:.1f} cm)")
        return -1
    elif distance_readings['center'] > safe_threshold:
        print(f"[DECISION] CENTER is clear ({distance_readings['center']:.1f} cm)")
        return 0
    else:
        print("[DECISION] all directions blocked! Reversing")
        return None

def check_obstacle():
    distance = distance_measurement()
    if distance < safe_threshold:
        return True, distance
    return False, distance

def avoid_obstacle():
    print("\n" + "="*50)
    print("{obstacle avoidance sequence initiated}")
    print("="*50)
    stop()
    time.sleep(0.5)
    safe_dir = scan_obstacle()
    time.sleep(0.5)
    print("\n[reversing]")
    backwards()
    time.sleep(1)
    stop()
    if safe_dir == 1:
        print("turning right")
        turn_right()
        time.sleep(1)
    elif safe_dir == -1:
        print("turning left")
        turn_left()
        time.sleep(1)
    else:
        print("Blocked or Center clear. Pausing before resuming forward.")
        stop()
        time.sleep(0.5)
    print("[Avoidance complete - resuming patrol]\n")

def patrol():
    print("\n" + "="*50)
    print("[starting]")
    print("="*50 + "\n")
    prev_pir_state = False
    try:
        forward()
        patrol_start = time.time()
        patrol_duration = 60
        while (time.time() - patrol_start) < patrol_duration:
            pir_state = check_pir_sensor()
            if pir_state and not prev_pir_state:
                print("\n[PIR sensor] motion detected")
                stop()
                intruder_alert()
                forward()
            prev_pir_state = pir_state
            obstacle_found, distance = check_obstacle()
            print(f"distance: {distance:.1f}cm", end="\r")
            if obstacle_found:
                stop()
                time.sleep(0.2)
                obstacle_avoidance_alert()
                avoid_obstacle()
                forward()
            time.sleep(0.1)
        stop()
        print("\nPatrol duration complete.")
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")

if __name__ == "__main__":
    try:
        print("initializing servo to center position (0 degrees)")
        set_servo_angle(0)
        time.sleep(1)
        patrol()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if servo_pwm:
            servo_pwm.stop()
        if buzzer_pwm:
            buzzer_pwm.stop()
        GPIO.cleanup()
        print("GPIO pins cleaned up. Program ended.")
