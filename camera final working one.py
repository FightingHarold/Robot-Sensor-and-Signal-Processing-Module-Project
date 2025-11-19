import RPi.GPIO as GPIO
import time
import os
from picamera2 import Picamera2
from picamera2.outputs import FileOutput
from threading import Thread
from datetime import datetime

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # Suppress GPIO warnings

# ===== PIN DEFINITIONS =====
PIR_PIN = 25              # Motion sensor input
SPOTLIGHT_PIN = 20        # Green LED spotlight (GPIO20) 
BUZZER_PIN = 18          # Buzzer for alarm output
LED_LEFT_PIN = 22        # Left indicator LED
LED_RIGHT_PIN = 27       # Right indicator LED

# ===== GPIO SETUP =====
GPIO.setup(PIR_PIN, GPIO.IN)
GPIO.setup(SPOTLIGHT_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LED_LEFT_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LED_RIGHT_PIN, GPIO.OUT, initial=GPIO.LOW)

# ===== CAMERA SETUP =====
# Create folders for saving footage in current directory (no permissions issues)
PHOTO_DIR = "intruder_photos"
VIDEO_DIR = "intruder_videos"

try:
    os.makedirs(PHOTO_DIR, exist_ok=True)
    os.makedirs(VIDEO_DIR, exist_ok=True)
    print(f"[FOLDERS] Photos directory: {os.path.abspath(PHOTO_DIR)}")
    print(f"[FOLDERS] Videos directory: {os.path.abspath(VIDEO_DIR)}")
except Exception as e:
    print(f"[ERROR] Failed to create directories: {e}")

# Initialize camera
camera_available = False
picam2 = None

try:
    picam2 = Picamera2()
    # Configure for photo capture
    config = picam2.create_still_configuration(
        main={"size": (1920, 1080)},
        lores={"size": (640, 480)}
    )
    picam2.configure(config)
    picam2.start()
    print("[CAMERA] Raspberry Pi Camera Module 2 initialized")
    camera_available = True
except Exception as e:
    print(f"[CAMERA ERROR] Camera initialization failed: {e}")
    print("[CAMERA] Continuing without camera functionality")
    camera_available = False

# ===== GLOBAL FLAGS =====
motion_alert_active = False
recording_active = False
system_running = True

# ===== SPOTLIGHT FUNCTIONS (GPIO20) =====

def spotlight_on():
    """Turn on green spotlight LED"""
    GPIO.output(SPOTLIGHT_PIN, GPIO.HIGH)
    print("[SPOTLIGHT] ON (GPIO20)")

def spotlight_off():
    """Turn off green spotlight LED"""
    GPIO.output(SPOTLIGHT_PIN, GPIO.LOW)
    print("[SPOTLIGHT] OFF (GPIO20)")

def spotlight_pulse(times=3, duration=0.3):
    """Pulse spotlight for visual alert"""
    print(f"[SPOTLIGHT] Pulsing {times} times")
    for i in range(times):
        spotlight_on()
        time.sleep(duration)
        spotlight_off()
        time.sleep(duration)

# ===== BUZZER FUNCTIONS =====

def buzzer_on():
    """Turn on buzzer"""
    GPIO.output(BUZZER_PIN, GPIO.HIGH)

def buzzer_off():
    """Turn off buzzer"""
    GPIO.output(BUZZER_PIN, GPIO.LOW)

def buzzer_beep(beeps=5, duration=0.2):
    """Beep buzzer multiple times"""
    for i in range(beeps):
        buzzer_on()
        time.sleep(duration)
        buzzer_off()
        time.sleep(duration)

# ===== LED INDICATOR FUNCTIONS =====

def alert_leds_on():
    """Turn on both indicator LEDs"""
    GPIO.output(LED_LEFT_PIN, GPIO.HIGH)
    GPIO.output(LED_RIGHT_PIN, GPIO.HIGH)

def alert_leds_off():
    """Turn off both indicator LEDs"""
    GPIO.output(LED_LEFT_PIN, GPIO.LOW)
    GPIO.output(LED_RIGHT_PIN, GPIO.LOW)

def alert_leds_blink(times=5, duration=0.3):
    """Blink indicator LEDs"""
    for i in range(times):
        alert_leds_on()
        time.sleep(duration)
        alert_leds_off()
        time.sleep(duration)

# ===== CAMERA CAPTURE FUNCTIONS =====

def capture_intruder_photo():
    """
    Capture a single photo when intruder detected
    Saves with timestamp in local directory
    """
    if not camera_available:
        print("[CAMERA] Camera not available")
        return None
    
    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        photo_filename = f"intruder_{timestamp}.jpg"
        photo_path = os.path.join(PHOTO_DIR, photo_filename)
        
        print(f"[CAMERA] Capturing photo: {photo_filename}")
        picam2.capture_file(photo_path)
        print(f"[CAMERA] Photo saved: {photo_path}")
        return photo_path
        
    except Exception as e:
        print(f"[CAMERA ERROR] Failed to capture photo: {e}")
        return None

def record_intruder_video(duration=10):
    """
    Record video when intruder detected
    Runs in background thread
    Records for specified duration (default 10 seconds)
    """
    global recording_active
    
    if not camera_available:
        print("[VIDEO] Camera not available")
        return None
    
    if recording_active:
        print("[VIDEO] Already recording")
        return None
    
    recording_active = True
    
    try:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_filename = f"intruder_{timestamp}.h264"
        video_path = os.path.join(VIDEO_DIR, video_filename)
        
        print(f"[VIDEO] Recording started: {video_filename}")
        print(f"[VIDEO] Duration: {duration} seconds")
        
        output = FileOutput(video_path)
        picam2.start_recording(output)
        
        # Record for specified duration
        start_time = time.time()
        while (time.time() - start_time) < duration:
            elapsed = int(time.time() - start_time)
            print(f"[VIDEO] Recording... {elapsed}/{duration}s", end="\r")
            time.sleep(1)
        
        picam2.stop_recording()
        print(f"\n[VIDEO] Recording complete: {video_path}")
        return video_path
        
    except Exception as e:
        print(f"[VIDEO ERROR] {e}")
        return None
    finally:
        recording_active = False

# ===== PIR SENSOR FUNCTIONS =====

def check_pir_sensor():
    """
    Check if PIR sensor detects motion
    Returns True if motion detected
    """
    try:
        if GPIO.input(PIR_PIN) == GPIO.HIGH:
            return True
    except:
        pass
    return False

# ===== INTRUDER ALERT SEQUENCE =====

def intruder_alert():
    """
    Complete intruder alert sequence as per assignment brief:
    1. Turn on spotlight
    2. Activate intruder alert (buzzer + LED blink)
    3. Capture camera footage (photo + video)
    
    Assignment requirement: "Motion detection - Turn on the spotlight, 
    activate the intruder alert. If a camera is available capture any footage"
    """
    global motion_alert_active
    
    if motion_alert_active:
        return
    
    motion_alert_active = True
    
    print("\n" + "!"*80)
    print("!!!                        [INTRUDER ALERT]                           !!!")
    print("!"*80 + "\n")
    
    try:
        # STEP 1: Turn on spotlight
        print("[ALERT] Activating spotlight...")
        spotlight_on()
        time.sleep(0.5)
        
        # STEP 2: Activate intruder alert - Buzzer
        print("[ALERT] Sounding alarm...")
        for i in range(8):
            buzzer_on()
            time.sleep(0.2)
            buzzer_off()
            time.sleep(0.2)
        
        # STEP 3: Blink LEDs for visual alert
        print("[ALERT] LED alert blinking...")
        alert_leds_blink(times=4, duration=0.25)
        
        # STEP 4: Capture photo if camera available
        if camera_available:
            print("[ALERT] Capturing intruder photo...")
            photo_path = capture_intruder_photo()
        else:
            print("[ALERT] Camera not available - skipping photo")
            photo_path = None
        
        # STEP 5: Record video in background if camera available
        if camera_available:
            print("[ALERT] Starting video recording...")
            video_thread = Thread(target=record_intruder_video, args=(15,), daemon=True)
            video_thread.start()
        
        # Keep spotlight ON during alert
        spotlight_on()
        
        # Alert duration: 10 seconds
        print("[ALERT] Alert active - spotlight on, monitoring...")
        time.sleep(10)
        
        # Turn off everything
        print("[ALERT] Alert ending...")
        spotlight_off()
        alert_leds_off()
        buzzer_off()
        
        print("\n" + "!"*80)
        print("!!!                    [ALERT SEQUENCE COMPLETE]                   !!!")
        print("!"*80 + "\n")
        
    except Exception as e:
        print(f"[ALERT ERROR] {e}")
    finally:
        motion_alert_active = False

# ===== MOTION DETECTION MONITOR =====

def motion_detection_monitor():
    """
    Main monitoring loop
    Continuously checks PIR sensor for motion
    Triggers alert when motion detected with cooldown to prevent spam
    """
    global system_running
    
    print("\n" + "="*80)
    print("[MOTION DETECTION SYSTEM - ACTIVE]")
    print("="*80)
    print("[PIR SENSOR]   GPIO25 - Monitoring for motion")
    print("[SPOTLIGHT]    GPIO20 - Green LED ready to activate")
    print("[BUZZER]       GPIO18 - Audio alarm ready")
    print("[LEDs]         GPIO22/27 - Visual alert ready")
    print("[CAMERA]       Ready to capture photos and videos")
    print("[PHOTOS]       Saved to: " + os.path.abspath(PHOTO_DIR))
    print("[VIDEOS]       Saved to: " + os.path.abspath(VIDEO_DIR))
    print("[ASSIGNMENT]   Motion detection with spotlight, alert, and camera footage")
    print("="*80)
    print("\nWaiting for motion... Press Ctrl+C to stop\n")
    
    motion_detected_before = False
    motion_cooldown = 0
    
    try:
        while system_running:
            # Check PIR sensor
            motion_detected = check_pir_sensor()
            
            # Trigger alert if motion just detected (with 5 second cooldown)
            if motion_detected and not motion_detected_before:
                if (time.time() - motion_cooldown) > 5:
                    print("\n[PIR DETECTED] Motion detected! Triggering alert sequence...")
                    intruder_alert()
                    motion_cooldown = time.time()
            
            # Update state
            motion_detected_before = motion_detected
            
            # Display status
            status = "DETECTED" if motion_detected else "clear    "
            recording_status = "RECORDING" if recording_active else "idle     "
            print(f"[STATUS] Motion: {status} | Camera: {recording_status} | Spotlight: {GPIO.input(SPOTLIGHT_PIN)}", end="\r")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n[SYSTEM] Motion detection stopped by user")
    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        # Cleanup
        print("[SYSTEM] Shutting down...")
        alert_leds_off()
        spotlight_off()
        buzzer_off()
        system_running = False

# ===== COMPONENT TEST FUNCTIONS =====

def test_spotlight():
    """Test green spotlight LED (GPIO20)"""
    print("\n[TEST] Green Spotlight LED (GPIO20)")
    print("  Testing ON for 2 seconds...")
    spotlight_on()
    time.sleep(2)
    spotlight_off()
    
    print("  Testing pulse 3 times...")
    spotlight_pulse(times=3, duration=0.5)
    print("  ✓ Spotlight test complete\n")

def test_buzzer():
    """Test buzzer"""
    print("[TEST] Buzzer (GPIO18)")
    print("  Testing beep 5 times...")
    buzzer_beep(beeps=5, duration=0.2)
    print("  ✓ Buzzer test complete\n")

def test_leds():
    """Test indicator LEDs"""
    print("[TEST] Indicator LEDs (GPIO22/27)")
    print("  Testing ON for 2 seconds...")
    alert_leds_on()
    time.sleep(2)
    alert_leds_off()
    
    print("  Testing blink 3 times...")
    alert_leds_blink(times=3, duration=0.5)
    print("  ✓ LED test complete\n")

def test_pir():
    """Test PIR sensor"""
    print("[TEST] PIR Sensor (GPIO25)")
    print("  Testing motion detection for 5 seconds...")
    print("  (Wave your hand in front of the sensor)")
    
    for i in range(5):
        status = "MOTION" if check_pir_sensor() else "clear"
        print(f"  {i+1}: {status}", end="\r")
        time.sleep(1)
    
    print("\n  ✓ PIR test complete\n")

def test_camera():
    """Test camera photo capture"""
    if not camera_available:
        print("\n[TEST] Camera - NOT AVAILABLE\n")
        return
    
    print("[TEST] Camera Module")
    print("  Capturing test photo...")
    photo_path = capture_intruder_photo()
    if photo_path:
        print(f"  ✓ Photo saved: {photo_path}\n")
    else:
        print("  ✗ Failed to capture photo\n")

def run_all_tests():
    """Run all component tests"""
    print("\n" + "="*80)
    print("[RUNNING COMPONENT TESTS]")
    print("="*80)
    
    test_spotlight()
    time.sleep(1)
    
    test_buzzer()
    time.sleep(1)
    
    test_leds()
    time.sleep(1)
    
    test_pir()
    time.sleep(1)
    
    test_camera()
    
    print("="*80)
    print("[ALL TESTS COMPLETE - Ready for motion detection]")
    print("="*80 + "\n")

# ===== MAIN ENTRY POINT =====

if __name__ == "__main__":
    try:
        print("\n[SYSTEM] Motion Detection with Camera - Initializing...\n")
        
        # Run component tests
        print("Running component tests first...\n")
        run_all_tests()
        
        # Start motion monitoring
        motion_detection_monitor()
        
    except KeyboardInterrupt:
        print("\n[SYSTEM] Interrupted by user")
    except Exception as e:
        print(f"[SYSTEM ERROR] {e}")
    finally:
        # Final cleanup
        print("\n[SYSTEM] Performing final cleanup...")
        alert_leds_off()
        spotlight_off()
        buzzer_off()
        
        if camera_available and picam2:
            try:
                picam2.stop()
                print("[CAMERA] Camera stopped")
            except:
                pass
        
        GPIO.cleanup()
        print("[SYSTEM] GPIO cleaned up")
        print("[SYSTEM] Program ended safely\n")
