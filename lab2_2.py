```python
from utils.brick import ColorSensor, reset_brick, wait_ready_sensors, Motor, time
import math


# ============================================================
# ROBOT CONSTANTS
# ============================================================

r = 2.2          # wheel radius (cm)
b = 9.8          # track width (cm)

# Starting odometer position
x = 0.0
y = 0.0
theta = 0.0      # degrees, 0 = +y direction

# Previous encoder positions
left_ep = 0.0
right_ep = 0.0


# ============================================================
# HARDWARE
# ============================================================

# Left = EV3 color sensor
# Right = NXT color/light sensor
left_light = ColorSensor("1")
right_light = ColorSensor("2")

left_motor = Motor("B")
right_motor = Motor("C")


# ============================================================
# LIGHT SENSOR
# ============================================================

# Change this after testing your sensors on white and black.
# The value should be somewhere between the white and black readings.
BLACK_THRESHOLD = 25


def is_black(sensor):
    """
    Returns True when the sensor sees the black grid line.

    get_reflected() gives reflected-light intensity.
    Black produces a lower reflected-light value than white.
    """
    return sensor.get_reflected() < BLACK_THRESHOLD


# ============================================================
# ODOMETER
# ============================================================

def update_odometer():
    """
    Updates x, y and theta using the motor encoders.

    Robot convention:
        theta = 0    -> +y direction
        theta = 90   -> +x direction

    Moving:
        left forward + right forward  -> +y
        left forward + right backward -> theta increases
    """

    global x, y, theta
    global left_ep, right_ep

    # Current encoder readings
    left_ec = left_motor.get_encoder()
    right_ec = right_motor.get_encoder()

    # Change in encoder position
    delta_left = left_ec - left_ep
    delta_right = right_ec - right_ep

    # Convert encoder degrees to wheel distance
    dl = delta_left * (2 * math.pi * r / 360.0)
    dr = delta_right * (2 * math.pi * r / 360.0)

    # Average distance travelled by the robot
    dc = (dl + dr) / 2.0

    # Change in heading.
    # Left forward/right backward = positive theta.
    dtheta = (dl - dr) / b

    # Use the average heading during this small movement.
    theta_rad = math.radians(theta)
    dtheta_rad = dtheta

    theta_mid = theta_rad + dtheta_rad / 2.0

    # Because theta=0 points along +y:
    dx = dc * math.sin(theta_mid)
    dy = dc * math.cos(theta_mid)

    # Update position
    x += dx
    y += dy

    # Update heading
    theta += math.degrees(dtheta)

    # Keep theta between 0 and 360
    theta %= 360.0

    # Save current encoder values
    left_ep = left_ec
    right_ep = right_ec


# ============================================================
# FLOAT MOTORS
# ============================================================

def float_motors():
    """
    Releases the motors and continuously displays
    the odometer values.

    Required for the Lab 2 float-motor demonstration.
    """

    left_motor.set_power(0)
    right_motor.set_power(0)

    try:
        while True:

            update_odometer()

            print(
                "x: {:.2f} cm | y: {:.2f} cm | theta: {:.2f} deg"
                .format(x, y, theta)
            )

            time.sleep(0.01)

    except KeyboardInterrupt:
        left_motor.set_dps(0)
        right_motor.set_dps(0)
        reset_brick()


# ============================================================
# MOVE FORWARD
# ============================================================

def move_forward(speed=150):
    """
    Drives both wheels forward.

    The robot stops when one of the light sensors
    detects a black grid line.
    """

    left_motor.set_dps(speed)
    right_motor.set_dps(speed)

    while True:

        update_odometer()

        left_black = is_black(left_light)
        right_black = is_black(right_light)

        if left_black or right_black:
            break

        time.sleep(0.01)

    left_motor.set_dps(0)
    right_motor.set_dps(0)


# ============================================================
# LIGHT SENSOR CORRECTION
# ============================================================

def correct_on_line():
    """
    Uses the two light sensors to align the robot with
    the black grid line.

    Cases:

        both black
            -> robot is aligned

        left black only
            -> move the right wheel forward

        right black only
            -> move the left wheel forward
    """

    left_motor.set_dps(0)
    right_motor.set_dps(0)

    time.sleep(0.1)

    while True:

        left_black = is_black(left_light)
        right_black = is_black(right_light)

        # Both sensors are on the line.
        if left_black and right_black:
            left_motor.set_dps(0)
            right_motor.set_dps(0)
            break

        # Left sensor is on the line first.
        elif left_black and not right_black:
            left_motor.set_dps(0)
            right_motor.set_dps(50)

        # Right sensor is on the line first.
        elif not left_black and right_black:
            left_motor.set_dps(50)
            right_motor.set_dps(0)

        time.sleep(0.01)

    # Stop once aligned
    left_motor.set_dps(0)
    right_motor.set_dps(0)

    # Update odometer one final time after correction
    update_odometer()


# ============================================================
# TURN
# ============================================================

def turn(angle):
    """
    Turns the robot by 'angle' degrees.

    Positive angle:
        left wheel forward
        right wheel backward

    Negative angle:
        left wheel backward
        right wheel forward
    """

    global theta

    # Save starting heading
    start_theta = theta

    # Determine desired heading
    target_theta = (start_theta + angle) % 360.0

    # Determine turn direction
    if angle > 0:
        left_speed = 100
        right_speed = -100
    else:
        left_speed = -100
        right_speed = 100

    left_motor.set_dps(left_speed)
    right_motor.set_dps(right_speed)

    while True:

        update_odometer()

        # Calculate signed angular error.
        error = (target_theta - theta + 180) % 360 - 180

        if abs(error) < 2.0:
            break

        time.sleep(0.01)

    left_motor.set_dps(0)
    right_motor.set_dps(0)

    # Small pause before driving again
    time.sleep(0.1)


# ============================================================
# SQUARE DRIVER
# ============================================================

def square_driver():
    """
    Drives the 90 cm square trajectory.

    Each side:
        1. Drive forward
        2. Detect black line
        3. Correct alignment using both sensors
        4. Turn 90 degrees
        5. Repeat
    """

    for side in range(4):

        print("Driving side", side + 1)

        # Drive forward until a grid line is detected
        move_forward()

        print("Line detected")

        # Align both sensors with the line
        correct_on_line()

        print(
            "Corrected position: x={:.2f}, y={:.2f}, theta={:.2f}"
            .format(x, y, theta)
        )

        # Turn 90 degrees
        print("Turning 90 degrees")

        turn(90)

        print(
            "After turn: x={:.2f}, y={:.2f}, theta={:.2f}"
            .format(x, y, theta)
        )

    # Stop
    left_motor.set_dps(0)
    right_motor.set_dps(0)

    print("Square completed!")
    print(
        "Final position: x={:.2f} cm, y={:.2f} cm, theta={:.2f} deg"
        .format(x, y, theta)
    )


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    try:

        print("Starting Lab 2...")

        # Make sure the sensors are ready
        wait_ready_sensors()

        # Start the square trajectory
        square_driver()

    except KeyboardInterrupt:

        print("\nStopping robot...")

        left_motor.set_dps(0)
        right_motor.set_dps(0)

        reset_brick()

        print("Robot stopped safely.")
```
