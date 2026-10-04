# This script uses the camera mounted on the trilobot to detect and count the number of balls in the image. 
# The ball detection is activated when an object is detected within a specific distance.
# When the robot is detecting balls, the LEDs are activated in red color, otherwise they are turned off.

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
    last distance measured, in centimetres. Used to only run the (slower) camera
    processing when something is close enough to be worth checking."""
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
        tbot.fill_underlighting(RED)
        num_circles=len(detected_circles[0,:,0])  
        # Convert the circle parameters x, y and r to integers.
        detected_circles = numpy.uint16(numpy.around(detected_circles))
        for pt in detected_circles[0,:]:
            x.append(pt[0])
            y.append(pt[1])
            r.append(pt[2])
    else:
        num_circles=0
        tbot.fill_underlighting(BLACK)
    return num_circles,x,y,r
        
while True or KeyboardInterrupt:
    distance=distance_detection()
    if distance<50: #50cm threshold
        image=capture_image()
        num_balls=circle_detection(image)
        print("NUMBER OF BALLS:",num_balls[0])
    else:
        print("NO BALLS DETECTED")
        tbot.fill_underlighting(BLACK) 
    
