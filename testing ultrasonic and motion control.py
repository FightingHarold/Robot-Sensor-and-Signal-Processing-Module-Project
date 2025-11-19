from gpiozero import LED, DistanceSensor, Servo
import time

#Led
led_left=LED(17)
led_right=LED(27)

#Ultrasonic sensor 
sensor=DistanceSensor(echo=24, trigger=23)

#Servo motor
servo=Servo(12)

#Variables
obstacle_detected=False
safe_direction=0

#Motion controls
def forward():
    print ("forward movement")
    led_left.on()
    led_right.on()
def backwards():
    print("backward movement")
    led_left.blink(on_time=0.1, off_time=0.1)
    led_right.blink(on_time=0.1, off_time=0.1)
def turn_left():
    print("turning left")
    led_left.blink(on_time=0.02, off_time=0.02)
    led_right.off()
def turn_right():
    print("turning right")
    led_right.blink(on_time=0.02, off_time=0.02)
    led_left.off()
def stop():
    print("Stopping")
    led_left.off()
    led_right.off()

#Servo scanning
def scan_obstacle(): 
    print("\n[Scanning] obstacle detected Searching for safe route")
    distance_readings={} 

    print("Scanning right")
    servo.value=1.0 
    time.sleep(0.5)
    distance_right=sensor.distance*100
    distance_readings['right']=distance_right
    print(f" Right distance: {distance_readings['right']:.1f} cm")
    time.sleep(0.3)

    #Scan center
    print("Scanning center")
    servo.value=0.0
    time.sleep(0.5)
    distance_center=sensor.distance*100
    distance_readings['center']=distance_center
    print(f" Center distance: {distance_center:.1f} cm")
    time.sleep(0.3)

    #Scan left at 90 degree
    print("Scanning left")
    servo.value=-1.0
    time.sleep(0.5)
    distance_left=sensor.distance*100
    distance_readings['left']=distance_left
    print(f" Left distance: {distance_left:.1f} cm")
    time.sleep(0.3)

    #return servo to center
    servo.value=0.0
    time.sleep(0.3)
    
    #distance>15cm
    safe_threshold=15
    
    # Prioritize right, then left, then center
    if distance_readings['right']>safe_threshold:
        print(f"[DECISION] RIGHT is clear ({distance_readings['right']:.1f} cm)")
        return 1
    elif distance_readings['left']>safe_threshold:
        print(f"[DECISION] LEFT is clear ({distance_readings['left']:.1f} cm)")
        return -1
    elif distance_readings['center']>safe_threshold:
        print(f"[DECISION] CENTER is clear ({distance_readings['center']:.1f} cm)")
        return 0
    else:
        print("[DECISION] all directions blocked! Reversing")
        return None

def check_obstacle():
    distance=sensor.distance*100
    if distance<15:
        return True, distance
    return False, distance

def avoid_obstacle():
    print("\n" + "="*50)
    print("{obstacle avoidance sequence initiated}")
    print("="*50)
    
    stop()
    time.sleep(0.5)
    
    safe_dir=scan_obstacle()
    time.sleep(0.5)
    
    print("\n[reversing]")
    backwards()
    time.sleep(1)
    
    if safe_dir==1:
        print("turning right")
        turn_right()
        time.sleep(1)
        
    elif safe_dir==-1:
        print("turning left")
        turn_left() 
        time.sleep(1)
    else:
        print("Resuming foward")
        stop()
        time.sleep(0.3)
    
    print("[Avoidance complete - resuming patrol]\n")


#Main program:
def patrol():
    print("\n" + "="*50)
    print("[starting]")
    print("="*50 + "\n")
    try:
        forward()
        patrol_start=time.time()
        patrol_duration=30
        
        while(time.time() - patrol_start) < patrol_duration:
            obstacle_found, distance=check_obstacle()
            
            print(f"distance: {distance:.1f}cm", end="\r")
            
            if obstacle_found:
                stop()
                time.sleep(0.2)
                avoid_obstacle()
                forward()
            time.sleep(0.1)
            
        stop()
        print("\nPatrol duration complete.") 
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")

if __name__ == "__main__":
    try:
        print("initializing servo to center position")
        servo.value=0.0 
        time.sleep(1)
        
        patrol()
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        led_left.close()
        led_right.close()
        sensor.close()
        servo.close()
        print("GPIO pins cleaned up. Program ended.")