# This script uses the camera mounted on the trilobot to detect the color of an object in the center of the image. 
# The object has to be detected within an specific distance in order to make the robot start running the color detection.
# If the object in front of the robot has a known color, then the LEDs are turned on using the same color, otherwise they are turned off.

import picamera
import cv2
import numpy
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

def distance_detection():
    """Reads the ultrasound distance sensor a few times in a row and returns the
    last distance measured, in centimetres."""
    # Take 3 measurements rapidly
    for i in range(3):
        clock_check = time.perf_counter()
        distance = tbot.read_distance(timeout=25, samples=3)
        #print("Rapid:  Distance is {:.1f} cm (took {:.4f} sec)".format(distance, (time.perf_counter() - clock_check)))
        time.sleep(0.01)
   
    return distance

def capture_image():
    """Captures a single still image from the Trilobot's camera and returns it as
    a BGR numpy array (the format OpenCV expects)."""
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)
        image = numpy.empty((240 * 320 * 3,), dtype=numpy.uint8)
        camera.capture(image, 'bgr')
        image = image.reshape((240, 320, 3))
        
    return image
  
def check_color(mask,h,w):
    """Checks whether a given colour mask has any pixels switched on within a
    central window of the image (ignoring the edges). Returns whether the colour
    was found, plus the image moments of the masked region."""
    
    search_top = int(h/2 - h/5)
    search_bot = int(h/2 + h/5)
    search_left = int(w/2 - w/5)
    search_right = int(w/2 + w/5)
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

def locate_color(M,w,h):
    """Converts image moments into a pixel position (cx, cy) and how far that
    position is from the centre of the image (err_x, err_y)."""
    cx = int(M[1]/M[2])
    cy = int(M[2]/M[0])
    err_x = cx - w/2
    err_y = cy - h/2
    return [cx,cy,err_x,err_y]

def color_detection(image):
    """Detects the colour (RED/YELLOW/GREEN/BLUE) of whichever known-coloured
    object is closest to the centre of the image, turns the underlighting to
    match, and returns a short text description of the result."""
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
    ## Masking
    h, w, d = image.shape
    [color_det_r,M_r]=check_color(mask_r,h,w)
    [color_det_y,M_y]=check_color(mask_y,h,w)
    [color_det_g,M_g]=check_color(mask_g,h,w)
    [color_det_b,M_b]=check_color(mask_b,h,w)
    ## Detecting the color (R,Y,G,B) of an object in front of the robot 
    color_det=[color_det_r,color_det_y,color_det_g,color_det_b]
    M=[M_r,M_y,M_g,M_b]
    unknown_color=True
    for i in range(4):
        # Is the object of a known color?
        if color_det[i]==True:
            if unknown_color==True:
                index=i
                unknown_color=False
                object_pos_index=locate_color(M[index],w,h)
            object_pos=locate_color(M[i],w,h)
            # To prioritize the object located at the center of the image
            if (abs(object_pos[2])+abs(object_pos[3])) < (abs(object_pos_index[2])+abs(object_pos_index[3])):
                index=i
                object_pos_index=locate_color(M[index],w,h)
    if unknown_color==False:
        if index==0:
            object_color="RED OBJECT"
            tbot.fill_underlighting(RED)
        elif index==1:
            object_color="YELLOW OBJECT"
            tbot.fill_underlighting(YELLOW)
        elif index==2:
            object_color="GREEN OBJECT"
            tbot.fill_underlighting(GREEN)
        elif index==3:
            object_color="BLUE OBJECT"
            tbot.fill_underlighting(BLUE)
    else:
        object_color="UNKNOWN COLOR"
        tbot.fill_underlighting(BLACK)    
        
    return object_color 
        
while True or KeyboardInterrupt:
    distance=distance_detection()
    if distance<30: #30cm threshold
        image=capture_image()
        object_color=color_detection(image)
        print(object_color)
    else:
        print("NO OBJECT DETECTED")
        tbot.fill_underlighting(BLACK)   
