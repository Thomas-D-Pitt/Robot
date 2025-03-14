import time
import RPi.GPIO as GPIO
import threading

class SoftwarePWM:
    def __init__(self, pin, frequency=1000, slew_rate=1):
        """
        Initialize the software PWM.

        :param pin: The GPIO pin object (e.g., from RPi.GPIO or machine.Pin) to control.
        :param frequency: The frequency of the PWM signal in Hz (default: 1000 Hz).
        :param slew_rate: The maximum change in duty cycle per second (default: 0.1).
        """
        self.pin = pin
        self.frequency = frequency
        self.period = 1.0 / frequency
        self.target_value = 0  # Target duty cycle (0 to 1)
        self.current_value = 0  # Current duty cycle (0 to 1)
        self.running = False
        self.thread = None
        self.slew_rate = slew_rate  # Max change per second in duty cycle

        self.set_target_value(0, slew_rate=0)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

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

    def set_target_value(self, target_value, slew_rate=None):
        """
        Set the target duty cycle and optionally ramp to it over a specified slew rate.

        :param target_value: The target duty cycle (0 to 1).
        :param slew_rate: The maximum change per second (duty cycle per second).
        """
        if target_value < 0 or target_value > 1:
            raise ValueError("Target value must be between 0 and 1.")

        if slew_rate is not None:
            self.slew_rate = slew_rate  # Update the slew rate if provided

        self.target_value = target_value

        # Calculate the ramp duration based on the slew rate
        if self.slew_rate > 0:
            delta = abs(self.target_value - self.current_value)
            ramp_duration = delta / self.slew_rate
            self._ramp_to_target(ramp_duration)
        else:
            self.current_value = target_value

    def _ramp_to_target(self, duration):
        """Ramp the current value to the target value over the specified duration."""
        start_time = time.time()
        start_value = self.current_value

        while time.time() - start_time < duration:
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
