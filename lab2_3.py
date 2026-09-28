#lab 2
from utils.brick import Motor, EV3ColorSensor
import time
import math

RADIUS = 2.2
color_sensor_right = EV3ColorSensor(1)
color_sensor_left = EV3ColorSensor(2)
TRACK_WIDTH = 10.0

DRIVE_SPEED = 200
CREEP_SPEED = 50
TURN_SPEED = 50

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

    # FIXED: Removed math.arcsin to prevent domain crashes during wheel slip
    delta_theta = (delta_distance_left - delta_distance_right) / TRACK_WIDTH
    delta_distance = (delta_distance_left + delta_distance_right) / 2.0

    # FIXED: Included global 'theta' so coordinates update correctly after turns
    delta_x = delta_distance * math.sin(theta + delta_theta / 2.0)
    delta_y = delta_distance * math.cos(theta + delta_theta / 2.0)

    x += delta_x
    y += delta_y
    theta = theta + math.degrees(delta_theta)
    theta = theta % 360.0

    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder

def move_fwd(d, axis='x'):
    """
    Teacher's style movement function. 
    Handles both positive (forward) and negative (backward) travel distances.
    """
    initial = x if axis == 'x' else y 
    target_distance = initial + d

    speed = DRIVE_SPEED if d > 0 else -DRIVE_SPEED
    leftmotor.set_dps(speed)
    rightmotor.set_dps(speed)

    # FIXED: Handles both increasing and decreasing coordinates properly
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
    BLACK_THRESHOLD = 30 

    leftmotor.set_dps(CREEP_SPEED)
    rightmotor.set_dps(CREEP_SPEED)

    left_line_detected = False
    right_line_detected = False

    while True: 
        update_odometer()
        
        left_color = color_sensor_left.get_red()
        right_color = color_sensor_right.get_red()

        if left_line_detected == True and right_line_detected == True:
            break 

        if left_color < BLACK_THRESHOLD and left_line_detected == False:
            leftmotor.set_dps(0) 
            left_line_detected = True   
            print("Left sensor detected the line")

        if right_color < BLACK_THRESHOLD and right_line_detected == False:
            rightmotor.set_dps(0)
            right_line_detected = True  
            print("Right sensor detected the line")

        if left_line_detected == True and right_line_detected == False:
            rightmotor.set_dps(CREEP_SPEED)
        elif right_line_detected == True and left_line_detected == False:
            leftmotor.set_dps(CREEP_SPEED)

        # FIXED: Moved time.sleep inside the loop to prevent CPU max-out/overheating
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

def travel_squares(num_squares=3, distance_per_square=25.0, axis='x', direction=1):
    """Travels across multiple squares, squaring up on each line."""
    total_distance = distance_per_square * direction
    for i in range(num_squares):
        move_fwd(total_distance, axis=axis)
        square_up_on_line()
        print(f"Squared up on line {i+1}")

# --- MAIN EXECUTION (All 4 Legs) ---
try:
    # Leg 1: +Y direction (North, 0 degrees)
    turn_to(0)
    travel_squares(num_squares=3, distance_per_square=25.0, axis='y', direction=1)
    
    # Leg 2: +X direction (East, 90 degrees)
    turn_to(90)
    travel_squares(num_squares=3, distance_per_square=25.0, axis='x', direction=1)
    
    # Leg 3: -Y direction (South, 180 degrees)
    turn_to(180)
    travel_squares(num_squares=3, distance_per_square=25.0, axis='y', direction=-1)
    
    # Leg 4: -X direction (West, 270 degrees)
    turn_to(270)
    travel_squares(num_squares=3, distance_per_square=25.0, axis='x', direction=-1)
    
    print("Full 4-sided path completed")

except KeyboardInterrupt:
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)
    print("Emergency stop")