import time

# GPIO mock/fallback setup
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    print("[⚠️ GPIO] RPi.GPIO not available or not running on a Pi. Using mock GPIO for development.")
    import types
    GPIO = types.SimpleNamespace()
    GPIO.BCM = GPIO.OUT = GPIO.IN = GPIO.HIGH = GPIO.LOW = 0
    GPIO.setmode = GPIO.setup = GPIO.output = GPIO.cleanup = GPIO.setwarnings = lambda *a, **kw: None


    class MockPWM:
        def __init__(self, pin, freq): pass
        def start(self, duty): pass
        def ChangeDutyCycle(self, duty): pass
        def stop(self): pass
    GPIO.PWM = MockPWM
    GPIO_AVAILABLE = False

# Define pin numbers
ENA = 13
IN1 = 6
IN2 = 5
ENB = 12
IN3 = 16
IN4 = 20
PWM_FREQ = 1000

class MotorController:
    def __init__(self):
        print("[MotorController] Initializing...")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(ENA, GPIO.OUT)

        GPIO.setup(IN3, GPIO.OUT)
        GPIO.setup(IN4, GPIO.OUT)
        GPIO.setup(ENB, GPIO.OUT)

        self.pwm_left = GPIO.PWM(ENA, PWM_FREQ)
        self.pwm_right = GPIO.PWM(ENB, PWM_FREQ)
        self.pwm_left.start(0)
        self.pwm_right.start(0)
        print("[MotorController] Ready.")

    def set_motor_speed(self, motor, speed):
        speed = max(-100, min(100, speed))
        if motor == "left":
            GPIO.output(IN1, GPIO.HIGH if speed > 0 else GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW if speed > 0 else GPIO.HIGH)
            self.pwm_left.ChangeDutyCycle(abs(speed) if speed != 0 else 0)
        elif motor == "right":
            GPIO.output(IN3, GPIO.HIGH if speed > 0 else GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW if speed > 0 else GPIO.HIGH)
            self.pwm_right.ChangeDutyCycle(abs(speed) if speed != 0 else 0)

    def move_forward(self, speed=50):
        self.set_motor_speed("left", speed)
        self.set_motor_speed("right", speed)

    def move_backward(self, speed=50):
        self.set_motor_speed("left", -speed)
        self.set_motor_speed("right", -speed)

    def turn_left(self, speed=50):
        self.set_motor_speed("left", 0)
        self.set_motor_speed("right", speed)

    def turn_right(self, speed=50):
        self.set_motor_speed("left", speed)
        self.set_motor_speed("right", 0)

    def smooth_left(self, speed=50):
        self.set_motor_speed("left", speed * 0.5)
        self.set_motor_speed("right", speed)

    def smooth_right(self, speed=50):
        self.set_motor_speed("left", speed)
        self.set_motor_speed("right", speed * 0.5)

    def spin_clockwise(self, speed=50):
        self.set_motor_speed("left", speed)
        self.set_motor_speed("right", -speed)

    def spin_counterclockwise(self, speed=50):
        self.set_motor_speed("left", -speed)
        self.set_motor_speed("right", speed)

    def stop(self):
        self.set_motor_speed("left", 0)
        self.set_motor_speed("right", 0)

    def cleanup(self):
        self.stop()
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
