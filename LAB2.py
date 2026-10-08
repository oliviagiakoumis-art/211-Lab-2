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

    current_left_encoder = -leftmotor.get_encoder()
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
    theta = (theta + math.degrees(delta_theta)) % 360.0

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

def move_fwd(d):
    
    initial = x
    leftmotor.set_dps(DRIVE_SPEED)
    rightmotor.set_dps(DRIVE_SPEED)
    while(x < initial + d):
            update_odometer()
            time.sleep(0.01)
        
    leftmotor.set_dps(0) 
    rightmotor.set_dps(0)

def turn():

    
    leftmotor.set_dps(TURN_SPEED)
    rightmotor.set_dps(-TURN_SPEED)
    


if __name__ == "__main__":
    try:
        while (True):
            print(f"X: {x}, Y: {y}, O: {theta}")
            move_fwd(15)

            if (color_sensor_left.get_rgb()[0] < 100 and color_sensor_right.get_rgb() [0] < 600):
                count += 1

            if count >= 25:
                move_fwd(15)
                initial = theta
                while(theta< initial + 90):
                    turn()
                count = 0
            update_odometer()    
    except BaseException:
        reset_brick()
        exit()

    reset_brick()
