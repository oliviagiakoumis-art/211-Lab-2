# lab 2 - odometry with light-sensor correction (DEBUG version)
from utils.brick import Motor, EV3ColorSensor, NXTColorSensor, reset_brick, wait_ready_sensors
import time
import math

MODE = "" #"FLOAT" 
# ---------- measure/tune these ----------
RADIUS = 2.2            # cm, wheel radius
TRACK_WIDTH = 10.0      # cm, distance between wheels
SENSOR_OFFSET = 5.0     # cm from wheel axle to light sensors (measure this!)
TILE = 30.48            # cm

LEFT_THRESHOLD = 35         # EV3 left sensor (get_red below this = black)
RIGHT_THRESHOLD = 3         # NXT right sensor (get_value below this = black)
# -----------------------------------------------------------
START = True
DRIVE_SPEED = -100
TURN_SPEED = -30
count = 0
leftmotor = Motor("C")
rightmotor = Motor("B")
color_sensor_left = EV3ColorSensor(2)    # EV3, port 2
color_sensor_right = NXTColorSensor(1)   # NXT, port 1
wait_ready_sensors()

x = 0.0
y = 0.0
theta = 0.0

leftmotor.reset_encoder()
rightmotor.reset_encoder()
prev_right_encoder = 0
prev_left_encoder = 0

def update_odometer():
    global x, y, theta, prev_right_encoder, prev_left_encoder

    # 1. Get current encoder values
    current_left_encoder = leftmotor.get_encoder()
    current_right_encoder = rightmotor.get_encoder()

    # 2. Compute change in encoder values
    delta_left = current_left_encoder - prev_left_encoder
    delta_right = current_right_encoder - prev_right_encoder

    # 3. Convert encoder changes to wheel distances (cm)
    delta_distance_left = (delta_left * 2 * math.pi * RADIUS) / 360.0
    delta_distance_right = (delta_right * 2 * math.pi * RADIUS) / 360.0

    # 4. Compute change in theta
    # asin gives radians, so convert to degrees
    delta_theta = (180.0 / math.pi) * math.asin(
        (delta_distance_left - delta_distance_right) / TRACK_WIDTH
    )

    # 5. Compute average distance traveled
    delta_distance = (delta_distance_left + delta_distance_right) / 2.0

    # 6. Compute change in x and y
    # Use theta + half of the change in theta
    theta_mid = math.radians(theta + delta_theta / 2.0)

    delta_x = delta_distance * math.sin(theta_mid)
    delta_y = delta_distance * math.cos(theta_mid)

    # 7. Update global position
    x = x + delta_x
    y = y + delta_y
    theta = theta + delta_theta

    # 8. Keep theta between 0 and 360 degrees
    theta = theta % 360.0

    # 9. Update previous encoder values
    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder

def float_motors():
    leftmotor.float_motor()
    rightmotor.float_motor()
    while True:
            update_odometer()
            time.sleep(0.01)

def stop_robot():
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)

def move_fwdy(d):
    initial = y
    leftmotor.set_dps(DRIVE_SPEED)
    rightmotor.set_dps(DRIVE_SPEED)
    while(y < initial + d):
        update_odometer()
        time.sleep(0.01)
             
    leftmotor.set_dps(0) 
    rightmotor.set_dps(0)
def move_fwdx(d):
    
    initial = x
    leftmotor.set_dps(DRIVE_SPEED)
    rightmotor.set_dps(DRIVE_SPEED)
    while(x < initial + d):
            update_odometer()
            time.sleep(0.01)
        
    leftmotor.set_dps(0) 
    rightmotor.set_dps(0)

def turn(turn_angle):
    initial = theta

    leftmotor.set_dps(-30)
    rightmotor.set_dps(-30)

    while theta < initial + turn_angle:
        update_odometer()
        time.sleep(0.01)

    leftmotor.set_dps(0)
    rightmotor.set_dps(0)

if __name__ == "__main__":
    try:
        while (True):
            if START :
                move_fwdy(15)
            else:
                 move_fwdx(15)

            if (color_sensor_left.get_rgb()[0] < 100 and color_sensor_right.get_rgb() [0] < 600):
                count += 1

            if count >= 25:
                if START:
                    move_fwdy(15)
                else:
                     move_fwdx(15)
              
                turn(90)
                count = 0
                START = not START    
    except BaseException:
        reset_brick()
        exit()

    reset_brick()
