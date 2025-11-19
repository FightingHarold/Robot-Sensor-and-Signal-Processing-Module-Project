import RPi.GPIO as GPIO
import time
import spidev

GPIO.setmode(GPIO.BCM)

# Pin Definitions
LED_LEFT_PIN = 22
LED_RIGHT_PIN = 27
TRIG_PIN = 23
ECHO_PIN = 24
SERV0_PIN = 17
PIR_PIN = 25
BUZZER_PIN = 18
SPI_PORT = 0
SPI_DEVICE = 0
MCP_CHANNEL = 0
VOLTAGE_THRESHOLD = 50

servo_pwm = None
buzzer_pwm = None

# Outputs
GPIO.setup(LED_LEFT_PIN, GPIO.OUT)
GPIO.setup(LED_RIGHT_PIN, GPIO.OUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(SERV0_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Inputs
GPIO.setup(PIR_PIN, GPIO.IN)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Servo and Buzzer PWM setup (create once)
servo_pwm = GPIO.PWM(SERV0_PIN, 50)
servo_pwm.start(0)

buzzer_pwm = GPIO.PWM(BUZZER_PIN, 1000)
buzzer_pwm.start(0)  # start with 0 duty cycle (off)

try:
    spi = spidev.SpiDev()
    spi.open(SPI_PORT, SPI_DEVICE)
    spi.max_speed_hz = 1350000
    print("[SPI] mcp3008 initialized successfully")
except Exception as e:
    print(f"[Error] failed to initialize SPI: {e}")
    spi = None

safe_threshold = 15  # Distance in cm

def read_mcp_3008_channel(channel):
    if spi is None:
        print("[Error] spi not initialized")
        return 0
    adc_value = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc_value[1] & 3) << 8) + adc_value[2]
    return data

def get_voltage():
    adc_value = read_mcp_3008_channel(MCP_CHANNEL)
    voltage = (adc_value * 3.3 / 1023) * 2
    print(f"Simulated Battery voltage: {voltage:.2f} V")
    return voltage

def buzzer_off():
    buzzer_pwm.ChangeDutyCycle(0)

def buzzer_on():
    buzzer_pwm.ChangeDutyCycle(50)
    time.sleep(0.2)
    buzzer_pwm.ChangeDutyCycle(0)
    time.sleep(0.2)

def battery_low_alert():
    print("[BATTERY] Battery level LOW!")
    for _ in range(5):
        GPIO.output(LED_LEFT_PIN, True)
        GPIO.output(LED_RIGHT_PIN, True)
        buzzer_pwm.ChangeDutyCycle(50)
        time.sleep(0.3)
        GPIO.output(LED_LEFT_PIN, False)
        GPIO.output(LED_RIGHT_PIN, False)
        buzzer_pwm.ChangeDutyCycle(0)
        time.sleep(0.2)
    buzzer_pwm.ChangeDutyCycle(0)

# ... (other functions remain unchanged) ...

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
    if GPIO.input(PIR_PIN) == 1:
        return True
    return False

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

# ... (rest of functions unchanged) ...

def patrol():
    print("\n" + "="*50)
    print("[Starting]")
    print("="*50 + "\n")
    try:
        forward()
        patrol_start = time.time()
        patrol_duration = 60
        while (time.time() - patrol_start) < patrol_duration:
            voltage = get_voltage()
            if voltage < VOLTAGE_THRESHOLD:
                battery_low_alert()
                print("[BATTERY] Battery is too LOW! Stopping patrol.")
                stop()
                break
            if check_pir_sensor():
                print("\n[PIR sensor] motion detected")
                stop()
                intruder_alert()
                forward()
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
                                                                        