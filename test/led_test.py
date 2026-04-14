import RPi.GPIO as GPIO
import time
import random

# --- LED Pin Definitions (BCM) ---
# Match these to your host_robot_logic.py
LED_FORWARD = 5
LED_LEFT = 6
LED_RIGHT = 13
LED_STOP = 19

LED_PINS = [LED_FORWARD, LED_LEFT, LED_RIGHT, LED_STOP]
COMMANDS = ["FORWARD", "ROTATE LEFT", "ROTATE RIGHT", "STOP"]

def setup():
    GPIO.setmode(GPIO.BCM)
    for pin in LED_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    print("--- Jim-E LED Hardware Test Initialized ---")
    print("Press Ctrl+C to stop the test.\n")

def update_leds(command_index):
    # Turn off all LEDs first
    for pin in LED_PINS:
        GPIO.output(pin, GPIO.LOW)
    
    # Turn on the specific LED for the chosen command
    chosen_pin = LED_PINS[command_index]
    GPIO.output(chosen_pin, GPIO.HIGH)
    
    print(f"Testing Command: {COMMANDS[command_index]} (GPIO {chosen_pin} ON)")

def loop():
    try:
        while True:
            # Randomly pick an index from 0 to 3
            pick = random.randint(0, 3)
            update_leds(pick)
            
            # Wait 2 seconds before the next random pick
            time.sleep(2.0)
            
    except KeyboardInterrupt:
        print("\nCleaning up GPIO and exiting...")
    finally:
        # Ensure all LEDs are off when we stop
        for pin in LED_PINS:
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()

if __name__ == "__main__":
    setup()
    loop()
