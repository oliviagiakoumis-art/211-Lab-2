#lab 2
# initialize brick pi...?
from utils.brick import configure_ports, reset_brick, wait_ready_sensors, Motor, time, EV3ColorSensor
import time, math

RADIUS = 2.2
color_sensor_right = EV3ColorSensor(1)
color_sensor_left = EV3ColorSensor(2)
TRACK_WIDTH = 10 #for now

leftmotor = Motor("C")
rightmotor = Motor("B")

x = 0.0
y = 0.0
theta = 0.0

# motor encoder already has an initial encoder tick counts, reset to 0.
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

def stop_robot():
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)

def move_fwd(d):

    initial = x #if in x-dir,,if in y, change this logic to y
    leftmotor.set_dps(30)
    rightmotor.set_dps(30)

    while(x < initial + d):
        update_odometer()
        time.sleep(0.01)
    
    leftmotor.set_dps(0) #for when you've reached necessary destination
    rightmotor.set_dps(0)