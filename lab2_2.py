#lab 2
from utils.brick import Motor, time, EV3ColorSensor
import time, math

RADIUS = 2.2
color_sensor_right = EV3ColorSensor(1)
color_sensor_left = EV3ColorSensor(2)
TRACK_WIDTH = 10 #for now

DRIVE_SPEED = 200
CREEP_SPEED = 50
TURN_SPEED = 50

leftmotor = Motor("C")
rightmotor = Motor("B")

x = 0.0
y = 0.0
theta = 0.0

# motor encoder reset to 0.
left_encoder_starter = leftmotor.reset_encoder()
right_encoder_starter = rightmotor.reset_encoder()
prev_right_encoder = 0
prev_left_encoder = 0

def update_odometer():
    global x, y, theta, prev_right_encoder, prev_left_encoder

    # get current cumulative encoder
    current_left_encoder = leftmotor.get_encoder()
    current_right_encoder = rightmotor.get_encoder()

    # change in encoder. how much did the wheel turn at this instant
    delta_left = current_left_encoder - prev_left_encoder
    delta_right = current_right_encoder - prev_right_encoder

    # convert to distance in cm
    delta_distance_left = (delta_left/ 360) * 2 * math.pi * RADIUS
    delta_distance_right = (delta_right / 360) * 2 * math.pi * RADIUS

    # theta changes
    delta_theta = math.arcsin((delta_distance_left - delta_distance_right)/ TRACK_WIDTH)

    delta_distance = (delta_distance_left + delta_distance_right) / 2

    delta_x = delta_distance * math.sin(delta_theta)
    delta_y = delta_distance * math.cos(delta_theta)

    x = x + delta_x
    y = y + delta_y

    theta = theta + math.degrees(delta_theta)

    # keep theta between 0 and 360
    theta = theta % 360.0

    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder

def move_fwd(d, axis='x'):
    """
    1. call move_fwd when we start the robot until hit a horizontal line.
    2. if one of the sensors hits the line first, immediately freeze that wheel and let the other motor run until the sensor senses the line. robot is now parallel.
    """

    initial = x if axis == 'x' else y #grabs the starting coordinate based on the chosen axis, x or y

    leftmotor.set_dps(DRIVE_SPEED)
    rightmotor.set_dps(DRIVE_SPEED)

    target_distance = initial + d

    # moves forward until it reaches the distance that we want (parameter d)
    while(x if axis == 'x' else y) < target_distance:
        update_odometer()
        time.sleep(0.01)
    
    leftmotor.set_dps(0) #for when you've reached necessary destination
    rightmotor.set_dps(0)

def square_up_on_line():
    BLACK_THRESHOLD = 30 ################fix

    leftmotor.set_dps(CREEP_SPEED)
    rightmotor.set_dps(CREEP_SPEED)

    left_line_detected = False
    right_line_detected = False

    # this loop runs until both wheels have found the black horizontal line and are locked into place
    while True: 
        update_odometer()
        
        left_color = color_sensor_left.get_red()
        right_color = color_sensor_right.get_red()

        # if both sensors hit the black line 
        if left_line_detected == True and right_line_detected == True:
            break 

        # LEFT SENSOR CHECK
        # if left sensor has reached the line (it sees black) and detection state hasnt been updated yet
        if left_color < BLACK_THRESHOLD and left_line_detected == False:
            leftmotor.set_dps(0) 
            left_line_detected = True   # mark that the left side found the line
            print("Left sensor detected the line")

        # RIGHT SENSOR CHECK
        # if left sensor has reached the line (it sees black) and detection state hasnt been updated yet
        if right_color < BLACK_THRESHOLD and right_line_detected == False:
            rightmotor.set_dps(0)
            right_line_detected = True  # mark that the right side found the line
            print("Right sensor detected the line")


        # MOVE THE SLOWER SIDE SO THAT IT ALIGNS WITH LINE
        if left_line_detected == True and right_line_detected == False:
            rightmotor.set_dps(CREEP_SPEED)
        elif right_line_detected == True and left_line_detected == False:
            leftmotor.set_dps(CREEP_SPEED)

    time.sleep(0.01)
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)
    print("both sides are now parallel to line")


def turn_to(target_theta):
    """Turns robot to an angle 0, 90, 180, 270 """
    global theta
    target_theta = target_theta % 360
    
    while True:
        update_odometer()

        # Find shortest angular error
        error = (target_theta - theta + 180) % 360 - 180
        
        if abs(error) < 1.5:  # 1.5 degree tolerance
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
    
    # Leg 1: +Y direction (North, 0 degrees) -> Coordinates increase
    turn_to(0)
    travel_squares(num_squares=3, distance_per_square=25.0, axis='y', direction=1)
    
    # Leg 2: +X direction (East, 90 degrees) -> Coordinates increase
    turn_to(90)
    travel_squares(num_squares=3, distance_per_square=25.0, axis='x', direction=1)
    
    # Leg 3: -Y direction (South, 180 degrees) -> Coordinates decrease (direction = -1)
    turn_to(180)
    travel_squares(num_squares=3, distance_per_square=25.0, axis='y', direction=-1)
    
    # Leg 4: -X direction (West, 270 degrees) -> Coordinates decrease (direction = -1)
    turn_to(270)
    travel_squares(num_squares=3, distance_per_square=25.0, axis='x', direction=-1)
    
    print("Full 4-sided path completed")

except KeyboardInterrupt:
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)
    print("Emergency stop")