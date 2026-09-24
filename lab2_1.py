from utils.brick import ColorSensor, reset_brick, wait_ready_sensors, Motor, time, EV3ultrasonicsensor
import math


r= 2.2 #cm
x = 0
y = 0
theta =0
b=  9.8 #cm  #track width,wodth of robot

left_light = ColorSensor("1")
right_light = ColorSensor("2")

left_motor= Motor("B")
right_motor = Motor("C")

left_ep = 0 #initialize
right_ep = 0 #iniitalize


distance=30.48*3

def update_odometer():
    
    #ghet current encoders
    left_ec = left_motor.get_encoder()
    right_ec = right_motor.get_encoder()
    
    #get differnece4 between prev and current encoders
    Del = left_ec-left_ep
    Der = right_ec-right_ep
        
        
    
    #2*(math.pi *r )/360 --> distance wheel travels for every degree of rot
    dl = ((Del) *2*(math.pi *r )/360)
    dr = ((Der) * (2*math.pi*r)/360)
    dd= (dl +dr)/2
    
    dtheta_rad = math.asin((dl-dr)/b)
    dtheta= (180/(math.pi)) * dtheta_rad
    dx= dd* math.sin(dtheta_rad)
    dy= dd* math.cos (dtheta_rad)
    
    
    #update x, y and theta
    global x
    global y
    global theta
    global left_ep
    global right_ep
    
    x += dx
    y += dy
    theta += dtheta
    
    # keep theta (0-360)
    theta= theta % 360
    
    #if theta becomes negative:
    if theta < 0:
        theta += 360.0
    
    #update ep with ec:
    left_ep = left_ec
    right_ep = right_ec




def float_motors():
    #release motors break-- allow it to spin
    left_motor.set_power(0)
    right_motor.set_power(0)
    
    #keep it going until back to initial positions
    #with this, we assume x,y,theta= 0,0,0 is at OG position
    try: 
        while True:
            
            #also need to keep track of previous encoder value(ep)
            update_odometer()
            
            print("x:", x)
            print("y:", y)
            print("theta:", theta)
            
            time.sleep(0.01)#mts use inside a continuou while loop
            
            
    except BaseException:
        reset_brick()
        exit()
        


def move_fwd(target_distance):
    """
    Drives forward until it reaches the target_distance (in cm) 
    or detects a black grid line using the light sensors.
    """
    # 1. Record starting position using current global odometer coordinates
    start_x = x
    start_y = y
    
    distance_traveled = 0.0
    black_threshold = 500  # Adjust this based on your light sensor calibration
    
    # 2. Start moving forward at a steady speed
    forward_speed = 150
    left_motor.set_dps(forward_speed)
    right_motor.set_dps(forward_speed)
    
    # 3. Loop continuously until target distance is met or a line stops us
    while distance_traveled < target_distance:
        # Check light sensors dynamically inside the loop
        left_val = left_light.get_light_level()
        right_val = right_light.get_light_level()
        
        if left_val < black_threshold or right_val < black_threshold:
            print("Hit a black line mid-move! Stopping.")
            break  # Exit movement early so light correction can happen
            
        # Update odometry to get fresh x and y coordinates from encoders
        update_odometer()
        
        # Continuously calculate real-time distance traveled from the start point
        distance_traveled = math.sqrt((x - start_x)**2 + (y - start_y)**2)
        
        # Small delay to keep the loop running smoothly
        time.sleep(0.01)
        
    # 4. Stop the motors when finished or when a line is hit
    left_motor.set_dps(0)
    right_motor.set_dps(0)
    
    
# def move_fwd(d):
# 
# 
# lines_passed = 0
# travelled=0
# 
# 
#     
# while(travelled <= d):
# 
# #if we keep this in while loop, it will constantly check the 15
#     left_is_black = left_light.get_light_level() 
#     right_is_black = right_light.get_light_level() 
#     
# 
# #if on first line:
#     if -45 < theta < 45 or 135< theta < 225:
#         initial = y
#         
#     if 45 < theta < 135 or 245 < theta < 325 :
#         initial = x
#         
#     
#         left_motor.set_dps(30)
#         right_motor.set_dps(30)
#     
#         while (y< initial + d):
#             update_odometer()
#             time.sleep(0.01)
#     
# #second line
#     if 45 < theta < 135 or 245 < theta < 325 :
#         initial = x
#         left_motor.set_dps(30)
#         right_motor.set_dps(30)
#     
#         while (x< initial + d):
#             update_odometer()
#             time.sleep(0.01)

    
'''
initial = x #if in x-dir,,if in y, change this logic to y
motor.set.dps(30)

while(x<initial +d):
    update_odometer()
    time.sleep(0.01)
    
motor.set_dps(0)#for when you've reached necessary destination


'''
        
    
def turn(turn_angle):
    #need to fix turn_angle
    
    initial = theta
    #turn by 90 degrees--> we could also use encoder if that easier?:
    left_motor.set_dps(30)
    right_motor.set_dps(-30)
    
    while(theta< initial + turn_angle):
        update_odometer()
        time.sleep(0.01)
    
    
    
        
        
#def square_driver():
    
#     lines_passed = 0
#     
#     left_is_black = left_light.get_light_level() < 500  
#     right_is_black = right_light.get_light_level() < 500
#     
#     
#     while not(left_is_black or right_is_black):
#         
#         
#         if lines_passed in range [0,2]:
#          move_fwd(30)
#          
#         if lines_passed == 3:
#             turn(90)
#             lines_passed = 0
#             
#     #not in while loop--> implies one of t=sensors picked up on black:
#     while color_sensor_left or color_sensor_right < 500:
#         if color_sensor_left == black and color_sensor_right != black:
#             #implies: left is behind right,,therefore need to shoft forward:
#             left_motor.set_dps(0)
#             right_motor.set_dps(30)
#         
#         if color_sensor_left != black and color_sensor_right == black:
#             #implies: left is behind right,,therefore need to shoft forward:
#             left_motor.set_dps(30)
#             right_motor.set_dps(0)
#         
#         
#         if color_sensor_left == black and color_sensor_right == black:
#             lines_passed +=1
#             move_fwd(2)#move forwrad ca bit so blakc no longer detected and we enter our while loop
#         
#    
#     update_odometer()
#          
        
def square_driver():
    sides_completed = 0
    black_threshold = 500  # Adjust based on your calibration
    
    # A square has 4 sides
    while sides_completed < 4:
        
        # --- PART 1: Drive along the side until hitting a grid line ---
        print(f"Driving along side {sides_completed + 1}")
        
        # Start moving forward (using your odometry/distance tracker or constant drive)
        left_motor.set_dps(150)
        right_motor.set_dps(150)
        
        while True:
            # Continuously update odometry in the background
            update_odometer()
            
            # Read light sensors live
            left_val = left_light.get_light_level()
            right_val = right_light.get_light_level()
            
            # Check if either sensor hits a black line
            if left_val < black_threshold or right_val < black_threshold:
                break # Exit this inner loop to handle line correction
                
            time.sleep(0.01)

        # --- PART 2: Light Correction Routine (Aligning on the line) ---
        print("Hit line! Starting correction alignment...")
        
        # Stop motors briefly to assess alignment
        left_motor.set_dps(0)
        right_motor.set_dps(0)
        time.sleep(0.2)
        
        # Adjust until both sensors see the black line squarely
        while True:
            l_hit = left_light.get_light_level() < black_threshold
            r_hit = right_light.get_light_level() < black_threshold
            
            if l_hit and r_hit:
                # Both are aligned on the line! Move slightly past it to clear it
                left_motor.set_dps(100)
                right_motor.set_dps(100)
                time.sleep(0.5) # Adjust time/distance to clear the line
                
                left_motor.set_dps(0)
                right_motor.set_dps(0)
                break
                
            elif l_hit and not r_hit:
                # Left hit first, nudge right wheel forward to straighten out
                left_motor.set_dps(0)
                right_motor.set_dps(50)
            elif not l_hit and r_hit:
                # Right hit first, nudge left wheel forward
                left_motor.set_dps(50)
                right_motor.set_dps(0)

        # --- PART 3: Turn 90 degrees for the next side ---
        print("Executing 90-degree turn...")
        turn()  # Call your turn function here
        
        sides_completed += 1
    
    if sides_completed == 4:
    # Stop all motors
        left_motor.set_dps(0)
        right_motor.set_dps(0)
        print("Reached starting zone! Stopping robot.")
        print("Square completed successfully!")

         
    
         
         
         
         
         
    



