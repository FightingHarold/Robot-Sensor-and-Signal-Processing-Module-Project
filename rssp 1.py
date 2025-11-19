import RPi.GPIO as GPIO
import time
import spidev
import os
from picamera2 import Picamera2
from picamera2.outputs import FileOutput
from threading import Thread
from datetime import datetime

# ===== GPIO SETUP =====
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

LED_LEFT = 22
LED_RIGHT = 27
TRIG = 23
ECHO = 24
SERVO_PIN = 17
BUZZER = 18
PIR = 25
SPOTLIGHT_PIN = 20
BATTERY_CHANNEL = 1
EMERGENCY_BUTTON = 6

GPIO.setup(LED_LEFT, GPIO.OUT)
GPIO.setup(LED_RIGHT, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(PIR, GPIO.IN)
GPIO.setup(SPOTLIGHT_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(EMERGENCY_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

servo = GPIO.PWM(SERVO_PIN, 50)
servo.start(0)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

PHOTO_DIR = "intruder_photos"
VIDEO_DIR = "intruder_videos"

try:
    os.makedirs(PHOTO_DIR, exist_ok=True)
    os.makedirs(VIDEO_DIR, exist_ok=True)
    print(f"[INIT] Photos: {os.path.abspath(PHOTO_DIR)}")
    print(f"[INIT] Videos: {os.path.abspath(VIDEO_DIR)}")
except Exception as e:
    print(f"[ERROR] Directory creation: {e}")

camera_available = False
picam2 = None
recording_active = False

try:
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"size": (1920, 1080)},
        lores={"size": (640, 480)}
    )
    picam2.configure(config)
    picam2.start()
    print("[INIT] Camera: READY")
    camera_available = True
except Exception as e:
    print(f"[INIT] Camera: UNAVAILABLE ({e})")
    camera_available = False

emergency_latched = False
square_side_count = 0      # current side (0-3)
forward_steps = 0          # steps on current side
STEPS_PER_SIDE = 3         # steps per side

# ===============================
# MOVEMENT AND CONTROL FUNCTIONS
# ===============================

def move_forward(duration=1):
    GPIO.output(LED_LEFT, GPIO.HIGH)
    GPIO.output(LED_RIGHT, GPIO.HIGH)
    print("→ FORWARD")
    time.sleep(duration)
    GPIO.output(LED_LEFT, GPIO.LOW)
    GPIO.output(LED_RIGHT, GPIO.LOW)

def move_reverse(blink_time=0.1, cycles=10):
    print("← REVERSE")
    for _ in range(cycles):
        GPIO.output(LED_LEFT, GPIO.HIGH)
        GPIO.output(LED_RIGHT, GPIO.HIGH)
        time.sleep(blink_time)
        GPIO.output(LED_LEFT, GPIO.LOW)
        GPIO.output(LED_RIGHT, GPIO.LOW)
        time.sleep(blink_time)

def turn_left(blink_time=0.03, cycles=20):
    print("↺ TURN LEFT (90°)")
    GPIO.output(LED_RIGHT, GPIO.HIGH)
    for _ in range(cycles):
        GPIO.output(LED_LEFT, GPIO.HIGH)
        time.sleep(blink_time)
        GPIO.output(LED_LEFT, GPIO.LOW)
        time.sleep(blink_time)
    GPIO.output(LED_RIGHT, GPIO.LOW)

def turn_right(blink_time=0.03, cycles=20):
    print("↻ TURN RIGHT (90°)")
    GPIO.output(LED_LEFT, GPIO.HIGH)
    for _ in range(cycles):
        GPIO.output(LED_RIGHT, GPIO.HIGH)
        time.sleep(blink_time)
        GPIO.output(LED_RIGHT, GPIO.LOW)
        time.sleep(blink_time)
    GPIO.output(LED_LEFT, GPIO.LOW)

def stop():
    GPIO.output(LED_LEFT, GPIO.LOW)
    GPIO.output(LED_RIGHT, GPIO.LOW)

def set_servo_angle(angle):
    duty = 2.5 + (angle / 18.0)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.4)
    servo.ChangeDutyCycle(0)

def get_distance():
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)
    pulse_start = time.time()
    pulse_end = time.time()
    timeout = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start - timeout > 0.1:
            return 999
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end - pulse_start > 0.1:
            return 999
    duration = pulse_end - pulse_start
    distance = duration * 17150
    return round(distance, 2)

def scan_for_obstacle():
    print("  [SCAN] Analyzing environment...")
    set_servo_angle(135)
    dist_left = get_distance()
    print(f"  [SCAN] Left: {dist_left:.1f}cm")
    set_servo_angle(45)
    dist_right = get_distance()
    print(f"  [SCAN] Right: {dist_right:.1f}cm")
    set_servo_angle(90)
    dist_center = get_distance()
    print(f"  [SCAN] Center: {dist_center:.1f}cm")
    dists = {'left': dist_left, 'center': dist_center, 'right': dist_right}
    best = max(dists, key=dists.get)
    print(f"  [SCAN] Best direction: {best.upper()}")
    return best

def obstacle_avoidance_alert():
    print("  [ALERT] Obstacle warning!")
    for _ in range(3):
        GPIO.output(LED_LEFT, GPIO.HIGH)
        GPIO.output(LED_RIGHT, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(LED_LEFT, GPIO.LOW)
        GPIO.output(LED_RIGHT, GPIO.LOW)
        time.sleep(0.5)

def obstacle_avoidance():
    print("\n[OBSTACLE DETECTED] Initiating avoidance...")
    stop()
    obstacle_avoidance_alert()
    best = scan_for_obstacle()
    if best == 'left':
        print("  [AVOIDANCE] Turning left")
        turn_left()
    elif best == 'right':
        print("  [AVOIDANCE] Turning right")
        turn_right()
    else:
        print("  [AVOIDANCE] Reversing and turning right")
        move_reverse()
        turn_right()
    print("[OBSTACLE AVOIDED] Resuming patrol\n")

# ===============================
# CAMERA, SPOTLIGHT, ALERTS
# ===============================

def spotlight_on():
    GPIO.output(SPOTLIGHT_PIN, GPIO.HIGH)

def spotlight_off():
    GPIO.output(SPOTLIGHT_PIN, GPIO.LOW)

def capture_intruder_photo():
    if not camera_available:
        return None
    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        photo_filename = f"intruder_{timestamp}.jpg"
        photo_path = os.path.join(PHOTO_DIR, photo_filename)
        picam2.capture_file(photo_path)
        print(f"  [CAMERA] Photo: {photo_filename}")
        return photo_path
    except Exception as e:
        print(f"  [CAMERA ERROR] {e}")
        return None

def record_intruder_video(duration=15):
    global recording_active
    if not camera_available or recording_active:
        return None
    recording_active = True
    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_filename = f"intruder_{timestamp}.h264"
        video_path = os.path.join(VIDEO_DIR, video_filename)
        print(f"  [VIDEO] Recording ({duration}s): {video_filename}")
        output = FileOutput(video_path)
        picam2.start_recording(output)
        time.sleep(duration)
        picam2.stop_recording()
        print(f"  [VIDEO] Complete: {video_filename}")
        return video_path
    except Exception as e:
        print(f"  [VIDEO ERROR] {e}")
        return None
    finally:
        recording_active = False

def intruder_alert():
    print("\n" + "!"*70)
    print("!!![INTRUDER DETECTED]!!!")
    print("!"*70)
    stop()
    print("  [ALERT] Spotlight ON")
    spotlight_on()
    print("  [ALERT] Sounding alarm...")
    buzzer = GPIO.PWM(BUZZER, 1000)
    for _ in range(8):
        buzzer.start(50)
        time.sleep(0.2)
        buzzer.stop()
        time.sleep(0.2)
    GPIO.output(BUZZER, GPIO.LOW)
    if camera_available:
        capture_intruder_photo()
        video_thread = Thread(target=record_intruder_video, args=(15,), daemon=True)
        video_thread.start()
    print("  [ALERT] Maintaining surveillance...")
    time.sleep(10)
    spotlight_off()
    print("!"*70)
    print("!!![ALERT COMPLETE]!!!")
    print("!"*70 + "\n")

# ===============================
# BATTERY AND ADC
# ===============================

def read_adc(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def get_battery_voltage():
    adc_value = read_adc(BATTERY_CHANNEL)
    voltage = (adc_value * 3.3 / 1023) * 2
    return voltage

def low_battery_alert():
    print("\n" + ("⚠"*70))
    print("[LOW BATTERY ALERT RECHARGE WHEN POSSIBLE]")
    print("⚠"*70)
    buzzer = GPIO.PWM(BUZZER, 1000)
    for _ in range(5):
        GPIO.output(LED_LEFT, GPIO.HIGH)
        GPIO.output(LED_RIGHT, GPIO.HIGH)
        buzzer.start(50)
        time.sleep(0.3)
        GPIO.output(LED_LEFT, GPIO.LOW)
        GPIO.output(LED_RIGHT, GPIO.LOW)
        buzzer.stop()
        time.sleep(0.3)
    GPIO.output(BUZZER, GPIO.LOW)
    print("⚠"*70 + "\n")

# =============================
# SQUARE PATROL LOGIC (NEW)
# =============================

def execute_square_patrol():
    global square_side_count, forward_steps
    move_forward(duration=1)
    forward_steps += 1
    if forward_steps >= STEPS_PER_SIDE:
        forward_steps = 0
        square_side_count += 1
        print(f"[SQUARE] Corner reached - Turning left")
        turn_left()
        time.sleep(0.5)
        if square_side_count >= 4:
            square_side_count = 0
            print("\n" + "="*70)
            print("[SQUARE]  One complete square patrol finished!")
            print("="*70 + "\n")

# ========================
# MAIN PROGRAM LOOP
# ========================

if __name__ == "__main__":
    try:
        print("\n" + "="*70)
        print("    SURVEILLANCE ROBOT - SQUARE PATROL MODE")
        print("="*70)
        print("[PATROL] Pattern: Square (4 sides × 3 steps per side)")
        print("[SENSORS] Monitoring: Obstacles | Intruders | Battery")
        print("[CONTROL] Emergency button: GPIO6 (Press to pause/resume)")
        print("="*70 + "\n")
        set_servo_angle(90)
        time.sleep(1)
        pir_cooldown = 0
        while True:
            button_pressed = GPIO.input(EMERGENCY_BUTTON) == GPIO.LOW
            time.sleep(0.05)
            if button_pressed:
                while GPIO.input(EMERGENCY_BUTTON) == GPIO.LOW:
                    time.sleep(0.05)
                emergency_latched = not emergency_latched
                if emergency_latched:
                    print("\n[EMERGENCY] SYSTEM PAUSED - Press button to resume")
                    stop()
                    spotlight_off()
                else:
                    print("[EMERGENCY] System resuming...\n")
            if emergency_latched:
                time.sleep(0.5)
                continue

            voltage = get_battery_voltage()
            if voltage < 2.0:
                print(f"[BATTERY] ⚠ Low: {voltage:.2f}V")
                low_battery_alert()
                time.sleep(2)
                continue

            distance = get_distance()
            if distance < 15:
                print(f"[OBSTACLE] Distance: {distance:.1f}cm - TOO CLOSE!")
                obstacle_avoidance()
                continue

            if GPIO.input(PIR) == GPIO.HIGH:
                if (time.time() - pir_cooldown) > 5:
                    print("[PIR]  Motion detected!")
                    intruder_alert()
                    pir_cooldown = time.time()
                    continue

            side_name = ["Side 1 (North)", "Side 2 (East)", "Side 3 (South)", "Side 4 (West)"]
            current_side = side_name[square_side_count]
            print(f"[PATROL] {current_side} | Step {forward_steps + 1}/{STEPS_PER_SIDE} | Distance: {distance:.1f}cm | Battery: {voltage:.2f}V")
            execute_square_patrol()
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Stopped by user (Ctrl+C)")
    finally:
        print("\n[SYSTEM] Shutting down...")
        stop()
        spotlight_off()
        GPIO.output(BUZZER, GPIO.LOW)
        servo.stop()
        spi.close()
        if camera_available and picam2:
            try:
                if recording_active:
                    picam2.stop_recording()
                picam2.stop()
                print("[CAMERA] Stopped")
            except:
                pass
        GPIO.cleanup()
        print("[SYSTEM] GPIO cleanup complete")
        print("[SYSTEM] Program ended safely\n")
