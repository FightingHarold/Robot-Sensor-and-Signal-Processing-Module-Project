from gpiozero import LED
import time

led_left=LED(22)
led_right=LED(27)

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
#Main program:
try:
    print("Starting motion control test\n")
    
    forward()
    time.sleep(3)
    stop()
    time.sleep(1)
    
    backwards()
    time.sleep(3)
    stop()
    time.sleep(1)
    
    turn_left()
    time.sleep(3)
    stop()
    time.sleep(1)
    
    turn_right()
    time.sleep(3)
    stop()
    
    print("\nMotion control test complete!")
except KeyboardInterrupt:
    print("\nProgram interrupted by user")
finally:
    led_left.close()
    led_right.close()
    print("GPIO pins cleaned up. Program ended.")