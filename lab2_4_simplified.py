# Lab 2 - Odometry with light-sensor correction
# Simplified control version:
#   1. Drive forward at ONE constant speed.
#   2. When a sensor sees a black line, STOP THAT WHEEL.
#   3. Keep the other wheel moving until its sensor sees the same line.
#   4. Correct odometry using the known line position.
#   5. After 3 detected lines, drive 10 cm straight.
#   6. Make a fixed 90-degree RIGHT turn.
#
# The odometry functions and required float_motors() function are kept.

from utils.brick import Motor, EV3ColorSensor, NXTColorSensor, reset_brick, wait_ready_sensors
import time
import math

# ---------------- MODE / DEBUG ----------------
MODE = "RUN"                 # "RUN", "TEST", or "FLOAT"
DEBUG = True
PRINT_ODOMETER = True         # Lab requires x/y/theta shown in console

# ---------------- MEASURE / TUNE ----------------
RADIUS = 2.2                  # cm
TRACK_WIDTH = 10.5            # cm
SENSOR_OFFSET = 5.0           # cm, axle -> light sensors
TILE = 30.48                  # cm

LEFT_THRESHOLD = 35           # EV3 red value below this = black
RIGHT_THRESHOLD = 3            # NXT value below this = black

DRIVE_SPEED = -60             # forward
TURN_SPEED = 35               # right turn: left forward, right backward

LINES_PER_SIDE = 3
EXTRA_DISTANCE = 10.0          # cm after the 3rd line before turning

SQUARE_TIMEOUT = 8.0 #gives up if 8 seconds pass after it tries to square up on a line
TURN_TIMEOUT = 5.0 #gives up if 5 seconds pass for turning


leftmotor = Motor("C")
rightmotor = Motor("B")
color_sensor_left = EV3ColorSensor(2)
color_sensor_right = NXTColorSensor(1)
wait_ready_sensors()


x = 0.0
y = 0.0
theta = 0.0

leftmotor.reset_encoder()
rightmotor.reset_encoder()
prev_left_encoder = 0.0
prev_right_encoder = 0.0

# ---------------- SENSOR HELPERS ----------------
def left_value():
    v = color_sensor_left.get_red()
    if isinstance(v, (list, tuple)):
        v = v[0] if len(v) > 0 else None
    return v


def right_value():
    v = color_sensor_right.get_value()
    if isinstance(v, (list, tuple)):
        v = v[0] if len(v) > 0 else None
    return v


def left_is_black(v):
    return v is not None and v < LEFT_THRESHOLD


def right_is_black(v):
    return v is not None and v < RIGHT_THRESHOLD


def left_on_line():
    return left_is_black(left_value())


def right_on_line():
    return right_is_black(right_value())


# ---------------- ODOMETER ----------------
def update_odometer():
    global x, y, theta, prev_left_encoder, prev_right_encoder

    current_left_encoder = -leftmotor.get_encoder()
    current_right_encoder = -rightmotor.get_encoder()

    delta_left = current_left_encoder - prev_left_encoder
    delta_right = current_right_encoder - prev_right_encoder

    delta_distance_left = (delta_left / 360.0) * 2.0 * math.pi * RADIUS
    delta_distance_right = (delta_right / 360.0) * 2.0 * math.pi * RADIUS

    delta_theta = (delta_distance_left - delta_distance_right) / TRACK_WIDTH
    delta_distance = (delta_distance_left + delta_distance_right) / 2.0

    # theta=0 is +y. Positive theta is a right-hand turn
    mid_theta = math.radians(theta) + delta_theta / 2.0
    x += delta_distance * math.sin(mid_theta)
    y += delta_distance * math.cos(mid_theta)
    theta = (theta + math.degrees(delta_theta)) % 360.0

    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder

    if PRINT_ODOMETER:
        print(f"x: {x:6.2f} cm | y: {y:6.2f} cm | theta: {theta:6.1f}")


def float_motors():
    leftmotor.float_motor()
    rightmotor.float_motor()
    while True:
        update_odometer()
        time.sleep(0.01)


# ---------------- BASIC MOTION ----------------
def stop():
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)


def coord(axis):
    return x if axis == 'x' else y


def move_forward_cm(distance_cm):
    """Drive straight for a fixed distance using encoder readings."""
    start_left = -leftmotor.get_encoder()
    start_right = -rightmotor.get_encoder()

    leftmotor.set_dps(DRIVE_SPEED)
    rightmotor.set_dps(DRIVE_SPEED)

    while True:
        update_odometer()

        current_left = -leftmotor.get_encoder()
        current_right = -rightmotor.get_encoder()

        d_left = (current_left - start_left) / 360.0 * 2.0 * math.pi * RADIUS
        d_right = (current_right - start_right) / 360.0 * 2.0 * math.pi * RADIUS
        average_distance = (d_left + d_right) / 2.0

        if average_distance >= distance_cm:
            break

        time.sleep(0.01)

    stop()


def square_up_on_line():
    """
    Drive at ONE constant speed toward the line.
    As soon as one sensor detects black, freeze that wheel.
    Keep the other wheel moving until its sensor also detects black.
    """
    left_done = False
    right_done = False
    t_start = time.time()

    leftmotor.set_dps(DRIVE_SPEED)
    rightmotor.set_dps(DRIVE_SPEED)

    print("[SQUARE] driving toward line...")

    while not (left_done and right_done):
        update_odometer()

        lv = left_value()
        rv = right_value()
        l_black = left_is_black(lv)
        r_black = right_is_black(rv)

        if l_black and not left_done:
            leftmotor.set_dps(0)
            left_done = True
            print(f"[SQUARE] LEFT stopped on line (value={lv})")

        if r_black and not right_done:
            rightmotor.set_dps(0)
            right_done = True
            print(f"[SQUARE] RIGHT stopped on line (value={rv})")

        if time.time() - t_start > SQUARE_TIMEOUT:
            print("[SQUARE] WARNING: timeout")
            break

        time.sleep(0.01)

    stop()
    print("[SQUARE] both sensors on line")


def turn_right_90():
    """
    Fixed-direction 90-degree RIGHT turn.
    Under this robot's lab convention:
      left wheel forward  = negative dps
      right wheel backward = positive dps
      -> theta increases.
    """
    global theta

    start_theta = theta
    leftmotor.set_dps(-TURN_SPEED)
    rightmotor.set_dps(TURN_SPEED)

    print(f"[TURN] RIGHT 90 degrees from theta={start_theta:.1f}")
    t_start = time.time()

    while True:
        update_odometer()
        turned = (theta - start_theta) % 360.0

        if turned >= 88.5:
            break

        if time.time() - t_start > TURN_TIMEOUT:
            print("[TURN] WARNING: timeout")
            break

        time.sleep(0.01)

    stop()
    print(f"[TURN] done, theta={theta:.1f}")


# ---------------- ONE SIDE OF THE 3x3 PATH ----------------
def travel_one_side(axis, direction, heading):
    """Detect 3 lines, square on each, then drive 10 cm beyond the 3rd line."""
    global x, y, theta

    start = coord(axis)

    for i in range(LINES_PER_SIDE):
        # Expected coordinate of the line, following the same geometry
        # used in the original version of the code.
        line = start + direction * (TILE / 2.0 + i * TILE)

        # Drive continuously until the sensors find the line.
        square_up_on_line()

        # The sensors are SENSOR_OFFSET in front of the wheel axle.
        fixed = line - direction * SENSOR_OFFSET
        if axis == 'x':
            x = fixed
        else:
            y = fixed

        # The robot is aligned with the grid line after squaring.
        theta = float(heading) % 360.0

        print(
            f"[LINE {i + 1}/{LINES_PER_SIDE}] "
            f"corrected -> x={x:.2f}, y={y:.2f}, theta={theta:.1f}"
        )

    print(f"[SIDE] 3 lines found -> driving {EXTRA_DISTANCE:.1f} cm")
    move_forward_cm(EXTRA_DISTANCE)


# ---------------- MAIN PATH ----------------
def square_driver():
    """
    find 3 black lines, adjust, then drive forward 10 cm until it turns right 90 degrees after legs 1-3
    """

    # Leg 1: +Y
    travel_one_side('y', +1, 0)
    turn_right_90()

    # Leg 2: +X
    travel_one_side('x', +1, 90)
    turn_right_90()

    # Leg 3: -Y
    travel_one_side('y', -1, 180)
    turn_right_90()

    # Leg 4: -X
    # No fourth turn: finish facing 270 degrees.
    travel_one_side('x', -1, 270)

    print("Full 4-sided path completed")
    print(f"FINAL -> x={x:.2f} cm | y={y:.2f} cm | theta={theta:.1f}")


# ---------------- MAIN ----------------
try:
    if MODE == "TEST":
        while True:
            print(
                f"L: {left_value()} | R: {right_value()} | "
                f"L line: {left_on_line()} | R line: {right_on_line()}"
            )
            time.sleep(0.2)

    elif MODE == "FLOAT":
        float_motors()

    else:
        square_driver()

except KeyboardInterrupt:
    print("Stopped by user (Ctrl+C)")

except Exception as e:
    print(f"Error: {e}")

finally:
    try:
        stop()
    except Exception:
        pass
    reset_brick()
    print("Brick reset")
