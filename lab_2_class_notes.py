#lab 2
import math



'''
rules about global variables:
1. you can use global variables as function argyments:
When you pass a global variable into a function as an argument, you are passing the value stored inside that variable.



2. can you update global varaible sinside a fcn?:

Rule A: Reading them is free. You can read a global variable inside any function without any special keywords.

Rule B: Reassigning them requires the global keyword. If you want to change what a global variable points to (like my_variable = new_value),
Python will assume it's a local variable unless you explicitly tell it otherwise using the global keyword.


'''



'''
use global variables x, y, theta

**for theta:
theta= 

r=radius of wheel -->measure
ec=current encoder(angle)
ep=previous encoder(angle)
b=track width


math context:

dL = ((ecl-epl) *2*math.pi *r )/360)
dR = ((ecr-ecr) * 2*math.pi*r)/360)
dtheta= (180/2*math.pi) * math.asin((dL-dR)/b)....in degrees
dd= (dl +dr)/2

de= ec-ep

dx= dd* sin(dtheta)
dy= dd * cos (dtheta)


'''

def update_odometer():
    '''
1. take ec(left and right wheels)

2. compute delta X,Y,Theta

3. take new measurements( for x, y and theta)
x= x+dx
y=y+dy
theta=theta+dtheta

find de= ec-ep (for both wheels)
**need some other global variables to keep track of ep

4. update ep:
ep=ec
'''

def move_fwd(d):

'''
initial = x #if in x-dir,,if in y, change this logic to y
motor.set.dps(30)

while(x<initial +d):
    update_odometer()
    time.sleep(0.01)
    
motor.set_dps(0)#for when you've reached necessary destination


'''


def turn(turn_angle):
    
    '''
    initial= theta
    #turn by 90 dgerees--> we could aso use encoder if that easier?:
    leftmotor.set_dps(30)
    rightmtoor.set_dps(-30)
    
    while(theta< initial + turn angle):
        update_odometer()
        time.sleep(0.01)
    
    
    '''
    
    
    
