import RPi.GPIO as GPIO
import time

pins = [12, 13, 22, 23, 24, 25, 26, 27]

def writeHigh(pin):
    sleep = 0.001
    GPIO.output(pin, GPIO.HIGH)  # Set pin 12 high
    time.sleep(sleep)
    GPIO.output(pin, GPIO.LOW)  # Set pin 12 high
    time.sleep(sleep)
    GPIO.output(pin, GPIO.HIGH)  # Set pin 12 high
    time.sleep(sleep)
    GPIO.output(pin, GPIO.LOW)  # Set pin 12 high
    time.sleep(sleep)
    GPIO.output(pin, GPIO.HIGH)  # Set pin 12 high
    time.sleep(sleep)
    GPIO.output(pin, GPIO.LOW)  # Set pin 12 high
    time.sleep(sleep)
    GPIO.output(pin, GPIO.HIGH)  # Set pin 12 high

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)  # or GPIO.BCM for BCM numbering
"""
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)

GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

GPIO.output(12, GPIO.LOW)
GPIO.output(13, GPIO.LOW)
GPIO.output(22, GPIO.LOW)
GPIO.output(23, GPIO.LOW)

GPIO.output(24, GPIO.LOW)
GPIO.output(25, GPIO.LOW)
GPIO.output(26, GPIO.LOW)
GPIO.output(27, GPIO.LOW)
"""
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
try:
    while True:
        print("high")
        for pin in pins:
            writeHigh(pin)
        time.sleep(2)  # Wait for 2 seconds
        print("low")
        for pin in pins:
            GPIO.output(pin, GPIO.LOW)   # Set pin 12 low
        time.sleep(2)  # Wait for another 2 seconds

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    GPIO.cleanup()  # Clean up GPIO settings before exit
