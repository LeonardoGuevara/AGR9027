# This script uses the camera mounted on the trilobot to detect, locate and determine the color of circles/balls in the image. 
# The color of the circle/ball defines what action the robot must do including 1) moving forwards for 3 seconds 2) moving while creating a 
# square shape path 3) moving while creating a circular shape path 4) adjusting the robot orientation to keep the circle/ball always detected 
# in the center of the image.
# When the robot detects a circle/ball with a specific color, the LEDs keep activated in that color while the robot performs the specific action
# otherwise they are turned off.
# After detecting the circle/ball, the robot performs the corresponding action only once, in order to repeat the action, the circle/ball must 
# be removed from the camera view to reset the robot's operation.

import picamera
import cv2
import numpy
import time
from trilobot import *

# create the robot object
tbot = Trilobot()

# RGB Colors
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)

action_completed=False # initial condition
robot_action= "NO ACTION" # initial condition

def capture_image():
    """Captures a single still image from the Trilobot's camera and returns it as
    a BGR numpy array, together with the image width (used later to work out how
    far a circle is from the centre of the image)."""
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)
        image = numpy.empty((240 * 320 * 3,), dtype=numpy.uint8)
        camera.capture(image, 'bgr')
        image = image.reshape((240, 320, 3))
        #image = cv2.imdecode(numpy.fromfile('/home/pi/trilobot-examples/images/red_circle.PNG', dtype=numpy.uint8), cv2.IMREAD_UNCHANGED)
        
        h, w, d = image.shape
        
    return image,w
  
def circle_detection(image):
    """Finds circular shapes (balls) in the image using a Hough circle transform.
    Returns the number of circles found and their x, y positions and radius (in
    pixels), as three separate lists."""
    # Convert to grayscale.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Blur using 3 * 3 kernel.
    gray_blurred = cv2.blur(gray, (3, 3))
    # Apply Hough transform on the blurred image.
    detected_circles = cv2.HoughCircles(gray_blurred, 
                       cv2.HOUGH_GRADIENT, 1, 20, param1 = 30,
                   param2 = 80, minRadius = 0, maxRadius = 0)
    # Count and locate the circles that are detected.
    x=[] # list with x component of the circles detected
    y=[] # list with y component of the circles detected
    r=[] # list with the radius of the circles detected
    if detected_circles is not None:
        num_circles=len(detected_circles[0,:,0])  
        # Convert the circle parameters x, y and r to integers.
        detected_circles = numpy.uint16(numpy.around(detected_circles))
        for pt in detected_circles[0,:]:
            x.append(pt[0])
            y.append(pt[1])
            r.append(pt[2])
    else:
        num_circles=0
    return num_circles,x,y,r

def check_color(mask,h,w,x,y,r):
    """Checks whether a given colour mask has any pixels switched on within the
    square region around a detected circle (x, y, r). Everything outside that
    region is zeroed out first. Returns whether the colour was found, plus the
    image moments of the masked region (used to measure how much of that colour
    is present)."""
    
    search_top = max(int(y - r),0)
    search_bot = min(int(y + r),h)
    search_left = max(int(x - r),0)
    search_right = min(int(x + r),w)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    mask[0:h, 0:search_left] = 0
    mask[0:h, search_right:w] = 0
    
    M = cv2.moments(mask)
    if M['m00'] > 0:
        color_det=True
    else:
        color_det=False
    return color_det,[M['m00'],M['m10'],M['m01']]

def color_detection(image,x,y,r):
    """Works out the colour (RED/YELLOW/GREEN/BLUE) of the biggest ball detected
    in the image, by masking each colour in HSV space and checking which one is
    present at the ball's position. Returns the detected colour (as text) and the
    x position of that ball in the image."""
            
    ## convert to hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    ## mask of red 
    mask_r_lower = cv2.inRange(hsv, (0,0,0), (10, 255, 255))
    mask_r_upper = cv2.inRange(hsv, (170,0,0), (180, 255, 255))
    mask_r=cv2.bitwise_or(mask_r_lower, mask_r_upper)
    ## mask of yellow 
    mask_y = cv2.inRange(hsv, (15,0,0), (36, 255, 255))
    ## mask of green 
    mask_g = cv2.inRange(hsv, (36, 0, 0), (70, 255,255))
    ## mask of blue 
    mask_b = cv2.inRange(hsv, (100,0,0), (135, 255, 255))

    h, w, d = image.shape
    
    ## Selecting the biggest ball detected
    circle_index=0
    for i in range(len(x)):
        if r[i]>=r[circle_index]:
            circle_index=i
    
    ## Masking
    [color_det_r,M_r]=check_color(mask_r,h,w,x[circle_index],y[circle_index],r[circle_index])
    [color_det_y,M_y]=check_color(mask_y,h,w,x[circle_index],y[circle_index],r[circle_index])
    [color_det_g,M_g]=check_color(mask_g,h,w,x[circle_index],y[circle_index],r[circle_index])
    [color_det_b,M_b]=check_color(mask_b,h,w,x[circle_index],y[circle_index],r[circle_index])       
    
    ## Detecting the color (R,Y,G,B) of the ball             
    color_det=[color_det_r,color_det_y,color_det_g,color_det_b]
    M=[M_r[0],M_y[0],M_g[0],M_b[0]]
    unknown_color=True
    for j in range(4):
        # Is the ball of a known color?
        if color_det[j]==True:
            if unknown_color==True:
                color_index=j
                unknown_color=False
            # To prioritize the color detected with bigger M0
            if M[j]>M[color_index]:
                color_index=j
    if unknown_color==False:
        if color_index==0:
            object_color="RED"
        elif color_index==1:
            object_color="YELLOW"
        elif color_index==2:
            object_color="GREEN"
        elif color_index==3:
            object_color="BLUE"
    else:
        object_color="UNKNOWN"
    return object_color,x[circle_index]

def action_planner(color,x_pos,width):
    """Decides what the robot should do based on the colour detected: GREEN makes
    it drive forward, YELLOW makes it follow a square path, BLUE makes it follow a
    circular path, and RED makes it rotate to keep tracking the ball. Returns a
    short text description of the action performed."""
    activate_leds(color)
    if color=="GREEN":
        tbot.forward()
        time.sleep(3)
        tbot.stop()
        robot_action="MOVING FORWARD FOR 3 SECONDS"      
    elif color=="YELLOW":
        tbot.forward()
        time.sleep(1)
        tbot.turn_right()
        time.sleep(0.3)
        tbot.forward()
        time.sleep(1)
        tbot.turn_right()
        time.sleep(0.3)
        tbot.forward()
        time.sleep(1)
        tbot.turn_right()
        time.sleep(0.3)
        tbot.forward()
        time.sleep(1)
        tbot.stop()
        robot_action="FOLLOWING A SQUARE PATH"
    elif color=="BLUE":
        vel_l=0.8
        vel_r=0.2
        tbot.set_motor_speeds(vel_l, vel_r)
        time.sleep(5)
        tbot.stop()
        robot_action="FOLLOWING A CIRCULAR PATH"
    elif color=="RED":
        w=width
        x=x_pos
        err_x = x - w/2
        vel = 0.5*(-float(err_x) / 100)
        print("VELOCITY",vel)
        if abs(vel)<0.15:
            tbot.disable_motors()
        else:
            tbot.set_motor_speeds(-vel, vel)
        robot_action="TRACKING THE BALL"
        
    return robot_action

def activate_leds(color):
    """Sets the underlighting to match the given colour name."""
    if color=="RED":
        tbot.fill_underlighting(RED)
    elif color=="YELLOW":
        tbot.fill_underlighting(YELLOW)
    elif color=="GREEN":
        tbot.fill_underlighting(GREEN)
    elif color=="BLUE":
        tbot.fill_underlighting(BLUE)   

while True or KeyboardInterrupt:
    [image,width]=capture_image()
    [num_balls,x_pos,y_pos,radius]=circle_detection(image)
    if num_balls>0:
        [ball_color,ball_pos_x]=color_detection(image,x_pos,y_pos,radius)
        if ball_color!="UNKNOWN":
            if action_completed==False or robot_action=="TRACKING THE BALL":
                if action_completed==False:
                    print("ACTION STARTED") 
                robot_action=action_planner(ball_color,ball_pos_x,width)            
                if robot_action!="TRACKING THE BALL":
                    action_completed=True
                    print("ACTION COMPLETED")                         
        else:
            print("UNKNOWN COLOR") 
            tbot.fill_underlighting(BLACK)  
            tbot.disable_motors()
    else:
        print("NO BALLS DETECTED")
        if robot_action!="TRACKING THE BALL" and action_completed==True:
            action_completed=False # To reset the robot operation
            print("ROBOT OPERATION RESETED") 
        tbot.fill_underlighting(BLACK)  
        tbot.disable_motors()
