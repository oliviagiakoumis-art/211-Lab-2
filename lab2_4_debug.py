# lab 2 - odometry with light-sensor correction (DEBUG version)
from utils.brick import Motor, EV3ColorSensor, NXTColorSensor, reset_brick, wait_ready_sensors
import time
import math

# ---------- MODE: "RUN" (square), "TEST" (print sensors), "FLOAT" (TA float check) ----------
MODE = "FLOAT"

# ---------- debug switches ----------
DEBUG = True              # live sensor / state prints while running
PRINT_ODOMETER = False    # False = hides the fast x/y/theta spam while debugging.
                          # SET TO True FOR THE TA DEMO (lab requires the console display)
SQUARE_TIMEOUT = 10.0     # s: give up squaring up if it takes this long
SECOND_SENSOR_TIMEOUT = 2.0  # s: give up if one sensor saw the line and the other never did
TURN_TIMEOUT = 10.0       # s: give up a turn if it takes this long

# ---------- measure/tune these ----------
RADIUS = 2.2            # cm, wheel radius
TRACK_WIDTH = 10.5      # cm, distance between wheels
SENSOR_OFFSET = 10.0     # cm from wheel axle to light sensors (measure this!)              # CHANGED!
TILE = 30.48            # cm

LEFT_THRESHOLD = 25         # EV3 left sensor (get_red below this = black)
RIGHT_THRESHOLD = 6         # NXT right sensor (get_value below this = black)
# -----------------------------------------------------------

DRIVE_SPEED = -100
CREEP_SPEED = -60
TURN_SPEED = -30
APPROACH_MARGIN = 6.0   # cm short of the line where we switch from fast to creeping

leftmotor = Motor("C")
rightmotor = Motor("B")
color_sensor_left = EV3ColorSensor(2)    # EV3, port 2
color_sensor_right = NXTColorSensor(1)   # NXT, port 1
wait_ready_sensors()

x = 0.0
y = 0.0                                                     # NOTE: y-coord is where the wheel axle (center of wheel) is
theta = 0.0

leftmotor.reset_encoder()
rightmotor.reset_encoder()
prev_right_encoder = 0
prev_left_encoder = 0


# ---------- sensor helpers ----------
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


# ---------- odometer ----------
def update_odometer():
    global x, y, theta, prev_right_encoder, prev_left_encoder

    current_left_encoder = -leftmotor.get_encoder()
    current_right_encoder = -rightmotor.get_encoder()

    delta_left = current_left_encoder - prev_left_encoder
    delta_right = current_right_encoder - prev_right_encoder

    delta_distance_left = (delta_left / 360.0) * 2 * math.pi * RADIUS
    delta_distance_right = (delta_right / 360.0) * 2 * math.pi * RADIUS

    delta_theta = (delta_distance_left - delta_distance_right) / TRACK_WIDTH
    delta_distance = (delta_distance_left + delta_distance_right) / 2.0

    delta_x = delta_distance * math.sin(math.radians(theta) + delta_theta / 2.0)
    delta_y = delta_distance * math.cos(math.radians(theta) + delta_theta / 2.0)

    x += delta_x
    y += delta_y
    theta = (theta + math.degrees(delta_theta)) % 360.0

    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder

    if PRINT_ODOMETER:
        print(f"x: {x:6.2f} cm | y: {y:6.2f} cm | theta: {theta:5.1f}")


def float_motors():
    leftmotor.float_motor()
    rightmotor.float_motor()
    while True:
        update_odometer()
        time.sleep(0.01)


# ---------- motion ----------
def stop():
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)

def coord(axis):
    return x if axis == 'x' else y

def move_to(target, axis, direction):  # line - direction * (SENSOR_OFFSET + APPROACH_MARGIN), axis, direction
    """Drive forward until the axle's coordinate on `axis` reaches target."""
    if DEBUG:
        print(f"[MOVE] driving along {axis}, from {coord(axis):.1f} to {target:.1f}")
    leftmotor.set_dps(DRIVE_SPEED)
    rightmotor.set_dps(DRIVE_SPEED)
    while (coord(axis) - target) * direction < 0:
        update_odometer()
        time.sleep(0.01)
    stop()

def turn_to(target_theta):
    """Turns robot to an angle 0, 90, 180, 270"""
    target_theta = target_theta % 360
    t_start = time.time()
    last_log = 0.0
    if DEBUG:
        print(f"[TURN] start: theta={theta:.1f} -> target {target_theta}")
    while True:
        update_odometer()
        error = (target_theta - theta + 180) % 360 - 180
        if abs(error) < 1.5:
            break
        if error > 0:
            leftmotor.set_dps(TURN_SPEED)
            rightmotor.set_dps(-TURN_SPEED)
        else:
            leftmotor.set_dps(-TURN_SPEED)
            rightmotor.set_dps(TURN_SPEED)

        now = time.time()
        if DEBUG and now - last_log > 0.2:
            print(f"[TURN] theta={theta:6.1f} | target={target_theta} | error={error:6.1f}")
            last_log = now
        if now - t_start > TURN_TIMEOUT:
            print("[TURN] WARNING: turn timed out. If theta moves AWAY from the target, "
                  "the motor/encoder direction is reversed.")
            break
        time.sleep(0.01)
    stop()
    if DEBUG:
        print(f"[TURN] done: theta={theta:.1f}")

def square_up_on_line():
    """Creep until BOTH sensors have seen the line; each wheel stops on its own."""
    leftmotor.set_dps(CREEP_SPEED)
    rightmotor.set_dps(CREEP_SPEED)

    left_done = False
    right_done = False
    t_start = time.time()
    first_detect_time = None
    last_log = 0.0

    if DEBUG:
        print("[SQUARE] creeping toward line...")

    while not (left_done and right_done):
        update_odometer()

        lv = left_value()
        rv = right_value()
        l_black = left_is_black(lv)
        r_black = right_is_black(rv)

        if l_black and not left_done:
            leftmotor.set_dps(0)
            left_done = True
            print(f"[SQUARE] LEFT detected line   (value={lv}, threshold={LEFT_THRESHOLD})")

        if r_black and not right_done:
            rightmotor.set_dps(0)
            right_done = True
            print(f"[SQUARE] RIGHT detected line  (value={rv}, threshold={RIGHT_THRESHOLD})")

        now = time.time()
        if DEBUG and now - last_log > 0.1:
            print(f"[SQUARE] L={lv} ({'BLACK' if l_black else 'white'}, "
                  f"{'stopped' if left_done else 'moving'}) | "
                  f"R={rv} ({'BLACK' if r_black else 'white'}, "
                  f"{'stopped' if right_done else 'moving'})")
            last_log = now

        # safety: one sensor saw the line, the other never does
        if (left_done or right_done) and first_detect_time is None:
            first_detect_time = now
        if first_detect_time is not None and now - first_detect_time > SECOND_SENSOR_TIMEOUT:
            missing = "RIGHT" if left_done else "LEFT"
            print(f"[SQUARE] WARNING: {missing} sensor never saw the line. "
                  f"Check its threshold, height, or which side it is on.")
            break

        # safety: nothing detected at all
        if now - t_start > SQUARE_TIMEOUT:
            print("[SQUARE] WARNING: timed out, no line found.")
            break

        time.sleep(0.01)

    stop()
    print("[SQUARE] finished squaring up")

def travel_squares(heading, axis, direction, num_squares=3):
    """Travel across squares, squaring up on each line and correcting the odometer."""
    global x, y, theta
    start = coord(axis)

    for i in range(num_squares):
        # where this line really is, along the axis
        line = start + direction * (TILE / 2 + i * TILE)

        # fast phase: stop APPROACH_MARGIN short of where the sensors should hit the line
        move_to(line - direction * (SENSOR_OFFSET + APPROACH_MARGIN), axis, direction)
        # creep phase
        square_up_on_line()

        # ---- odometer correction ----
        theta = float(heading)                          # robot is square to the line
        fixed = line - direction * SENSOR_OFFSET        # axle sits behind the line
        if axis == 'x':
            x = fixed
        else:
            y = fixed
        print(f"Squared up on line {i + 1} | corrected to x={x:.2f} y={y:.2f} theta={theta:.1f}")

    # drive into the center of the target tile before turning
    move_to(start + direction * num_squares * TILE, axis, direction)

def square_driver():
    turn_to(0)
    travel_squares(0, 'y', 1)       # Leg 1: +Y (north)

    turn_to(90)
    travel_squares(90, 'x', 1)      # Leg 2: +X (east)

    turn_to(180)
    travel_squares(180, 'y', -1)    # Leg 3: -Y (south)

    turn_to(270)
    travel_squares(270, 'x', -1)    # Leg 4: -X (west)

    print("Full 4-sided path completed")


# ---------- main ----------
try:
    if MODE == "TEST":
        while True:
            print(f"L: {left_value()} | R: {right_value()} "
                  f"| L line: {left_on_line()} | R line: {right_on_line()}")
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