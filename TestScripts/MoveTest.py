import RPi.GPIO as GPIO
import time
import threading

pins = [12, 13, 22, 23, 24, 25, 26, 27]
fpwmPins = [26, 27] 
rpwmPins = [24, 25] 
enablePins = [12, 13, 22, 23]

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)  # or GPIO.BCM for BCM numbering

for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

for pin in enablePins:
    GPIO.output(pin, GPIO.HIGH)

class SoftwarePWM:
    def __init__(self, pin, frequency=500):
        """
        Initialize the software PWM.

        :param pin: The GPIO pin object (e.g., from RPi.GPIO or machine.Pin) to control.
        :param frequency: The frequency of the PWM signal in Hz (default: 100 Hz).
        """
        self.pin = pin
        self.frequency = frequency
        self.period = 1.0 / frequency
        self.target_value = 0  # Target duty cycle (0 to 1)
        self.current_value = 0  # Current duty cycle (0 to 1)
        self.running = False
        self.thread = None

    def start(self):
        """Start the PWM thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._pwm_loop)
            self.thread.start()

    def stop(self):
        """Stop the PWM thread."""
        self.running = False
        if self.thread:
            self.thread.join()
        self.pin.off()  # Turn off the pin when stopping

    def set_target_value(self, target_value, ramp_duration=0):
        """
        Set the target duty cycle and optionally ramp to it over a specified duration.

        :param target_value: The target duty cycle (0 to 1).
        :param ramp_duration: The duration (in seconds) to ramp to the target value (default: 0, immediate).
        """
        if target_value < 0 or target_value > 1:
            raise ValueError("Target value must be between 0 and 1.")

        if ramp_duration < 0:
            raise ValueError("Ramp duration must be non-negative.")

        self.target_value = target_value

        if ramp_duration > 0:
            self._ramp_to_target(ramp_duration)
        else:
            self.current_value = target_value

    def _ramp_to_target(self, duration):
        """Ramp the current value to the target value over the specified duration."""
        start_time = time.time()
        end_time = start_time + duration
        start_value = self.current_value

        while time.time() < end_time:
            elapsed_time = time.time() - start_time
            progress = elapsed_time / duration
            self.current_value = start_value + (self.target_value - start_value) * progress
            time.sleep(0.01)  # Small delay to avoid excessive updates

        self.current_value = self.target_value  # Ensure exact target value

    def _pwm_loop(self):
        """Main PWM loop running in a separate thread."""
        GPIO.setmode(GPIO.BCM)
        while self.running:
            if self.current_value > 0:
                on_time = self.current_value * self.period
                off_time = self.period - on_time

                if on_time > 0:
                    GPIO.output(self.pin, GPIO.HIGH)
                    time.sleep(on_time)  # Wait for the on time
                if off_time > 0:
                    GPIO.output(self.pin, GPIO.LOW)
                    time.sleep(off_time)  # Wait for the off time
            else:
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(self.period)  # Wait for a full period

lfPin = SoftwarePWM(26)
lfPin.set_target_value(0)
lfPin.start()


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




try:
    while True:
        print("pulse...")
        lfPin.set_target_value(1, .5)
        # for pin in fpwmPins:
        # # Toggle GPIO pin 12
        #     GPIO.output(pin, GPIO.HIGH)  # Set pin 12 high
        time.sleep(1)  # Wait for 2 seconds
        lfPin.set_target_value(0, .5)
        # for pin in fpwmPins:
        #     GPIO.output(pin, GPIO.LOW)   # Set pin 12 low
        time.sleep(3)  # Wait for another 2 seconds

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    GPIO.cleanup()  # Clean up GPIO settings before exit
