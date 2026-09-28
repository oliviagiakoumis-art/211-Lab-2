# lab 2
from utils.brick import Motor, EV3ColorSensor
import time
import math

RADIUS = 2.2
color_sensor_right = EV3ColorSensor(1)
color_sensor_left = EV3ColorSensor(2)
TRACK_WIDTH = 10.0

BLACK_THRESHOLD = 30

DRIVE_SPEED = 100
CREEP_SPEED = 30
TURN_SPEED = 30

APPROACH_MARGIN = 6.0   # cm short of the line where we switch from DRIVE_SPEED to creeping
SENSOR_OFFSET = 5.0     # cm from the wheel axle to the light sensors (measure this!)

leftmotor = Motor("C")
rightmotor = Motor("B")

x = 0.0
y = 0.0
theta = 0.0

# Motor encoder reset to 0.
leftmotor.reset_encoder()
rightmotor.reset_encoder()
prev_right_encoder = 0
prev_left_encoder = 0

def update_odometer():
    global x, y, theta, prev_right_encoder, prev_left_encoder

    current_left_encoder = leftmotor.get_encoder()
    current_right_encoder = rightmotor.get_encoder()

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
    theta = theta + math.degrees(delta_theta)
    theta = theta % 360.0

    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder

    print(f"x: {x:6.2f} cm | y: {y:6.2f} cm | theta: {theta:5.1f}°")

def move_fwd(d, axis='x'):
    """
    d > 0: coordinate on this axis increases, d < 0: it decreases.
    The robot always drives forward (heading decides coordinate change direction).
    """
    initial = x if axis == 'x' else y 
    target_distance = initial + d

    leftmotor.set_dps(DRIVE_SPEED)
    rightmotor.set_dps(DRIVE_SPEED)

    if d > 0:
        while (x if axis == 'x' else y) < target_distance:
            update_odometer()
            time.sleep(0.01)
    else:
        while (x if axis == 'x' else y) > target_distance:
            update_odometer()
            time.sleep(0.01)
    
    leftmotor.set_dps(0) 
    rightmotor.set_dps(0)

def square_up_on_line():
    
    leftmotor.set_dps(CREEP_SPEED)
    rightmotor.set_dps(CREEP_SPEED)

    left_line_detected = False
    right_line_detected = False

    while True: 
        update_odometer()
        
        left_color = color_sensor_left.get_red()
        right_color = color_sensor_right.get_red()

        if left_line_detected and right_line_detected:
            break 

        if left_color < BLACK_THRESHOLD and not left_line_detected:
            leftmotor.set_dps(0) 
            left_line_detected = True   
            print("Left sensor detected the line")

        if right_color < BLACK_THRESHOLD and not right_line_detected:
            rightmotor.set_dps(0)
            right_line_detected = True  
            print("Right sensor detected the line")

        if left_line_detected and not right_line_detected:
            rightmotor.set_dps(CREEP_SPEED)
        elif right_line_detected and not left_line_detected:
            leftmotor.set_dps(CREEP_SPEED)

        time.sleep(0.01)

    leftmotor.set_dps(0)
    rightmotor.set_dps(0)
    print("Both sides are now parallel to line")

def turn_to(target_theta):
    """Turns robot to an angle 0, 90, 180, 270"""
    global theta
    target_theta = target_theta % 360
    
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
            
        time.sleep(0.01)
        
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)

def travel_squares(num_squares=3, distance_per_square=30.48, axis='x', direction=1):
    """Travels across multiple squares, squaring up on each line."""
    for i in range(num_squares):
        if i == 0:
            fast_distance = distance_per_square / 2 - SENSOR_OFFSET - APPROACH_MARGIN
        else:
            fast_distance = distance_per_square - APPROACH_MARGIN

        move_fwd(fast_distance * direction, axis=axis)   # Fast phase
        square_up_on_line()                              # Creep phase
        print(f"Squared up on line {i+1}")

    # Drive past the 3rd line into the center of the target square before turning
    move_fwd((distance_per_square / 2 + SENSOR_OFFSET) * direction, axis=axis)

# main execution: 4 legs
try:
    # Leg 1: +Y direction (North, 0 degrees)
    turn_to(0)
    travel_squares(num_squares=3, distance_per_square=30.48, axis='y', direction=1)
    
    # Leg 2: +X direction (East, 90 degrees)
    turn_to(90)
    travel_squares(num_squares=3, distance_per_square=30.48, axis='x', direction=1)
    
    # Leg 3: -Y direction (South, 180 degrees)
    turn_to(180)
    travel_squares(num_squares=3, distance_per_square=30.48, axis='y', direction=-1)
    
    # Leg 4: -X direction (West, 270 degrees)
    turn_to(270)
    travel_squares(num_squares=3, distance_per_square=30.48, axis='x', direction=-1)
    
    print("Full 4-sided path completed")

except KeyboardInterrupt:
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)
    print("Emergency stop")