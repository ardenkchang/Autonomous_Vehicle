# Robust Linear Regression Example
#
# This example shows off how to use the get_regression() method on your OpenMV Cam
# to get the linear regression of a ROI. Using this method you can easily build
# a robot which can track lines which all point in the same general direction
# but are not actually connected. Use find_blobs() on lines that are nicely
# connected for better filtering options and control.
#
# We're using the robust=True argument for get_regression() in this script which
# computes the linear regression using a much more robust algorithm... but potentially
# much slower. The robust algorithm runs in O(N^2) time on the image. So, YOU NEED
# TO LIMIT THE NUMBER OF PIXELS the robust algorithm works on or it can actually
# take seconds for the algorithm to give you a result... THRESHOLD VERY CAREFULLY!

THRESHOLD = (128, 255) # Grayscale threshold for dark things...
BINARY_VISIBLE = True # Does binary first so you can see what the linear regression
                      # is being run on... might lower FPS though.

import sensor, image, time, pyb
from pyb import Pin, Timer, LED,


def DeinitTimer(TimeVar,CLKNum, frequ):
    time.sleep_ms(50)
    Timer.deinit(TimeVar)
    tim4 = Timer(CLKNum, freq= frequ)

def SetPWmotor(PWpercent):
    #DeinitTimer(tim2, 2, 100)
    ch1 = tim2.channel(3, Timer.PWM, pin=Pin("P4"), pulse_width=PWpercent)

def SetPWservo(PWidth):
    DeinitTimer(tim2, 2, 100)
    ch2 = tim2.channel(4, Timer.PWM, pin=Pin("P5"), pulse_width=PWidth)

tim2 = Timer(2, freq=100)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.
sensor.set_brightness(-3)

red_led   = LED(1)

green_led = LED(2)

blue_led = LED(3)

def redled_control(x):
    if   (x&1)==0: red_led.off()
    elif (x&1)==1: red_led.on()


def blueled_control(x):
    if   (x&1)==0: blue_led.off()
    elif (x&1)==1: blue_led.on()

def greenled_control(x):
    if   (x&1)==0: green_led.off()
    elif (x&1)==1: green_led.on()

prev3_deltax1 = 0
prev2_deltax1 = 0
prev1_deltax1 = 0

kp = 1
kd = 0

#olderror_x1 = 0
olderror_coord = 0
set_coord = 40
derror = 0
error_coord = 0
output = 0

p_term = 0
d_term = 0

old_time = pyb.millis()


def constrain (value, minimum, maximum):
    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    else:
        return value

def update_pid(input_value):
    global olderror_coord
    offset = 15
    input_value = constrain(x1, offset, 80-offset)

    new_time = pyb.millis()
    dt = new_time - old_time
    error_coord = set_coord - input_value
    derror = error_coord - olderror_coord

    p_term = kp * error_coord
    d_term = kd * (derror/dt)

    olderror_coord = error_coord
    output = p_term + d_term

    max_output = kp * 25
    #SetPWservo(240000 + 120000 + 120000 * output/max_output)

    output = output/max_output

    return output

flag = 1
servoVal= 0
prev_x1 = 40
desired_x1 = 40
deltax1 = 0
delta_thresh = 10 #change in coordinates
p_thresh = 25   #number of pixels
sum_delta_thresh = 25
last_deflection = 0

while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()

    # Returns a line object similar to line objects returned by find_lines() and
    # find_line_segments(). You have x1(), y1(), x2(), y2(), length(),
    # theta() (rotation in degrees), rho(), and magnitude().
    #
    # magnitude() represents how well the linear regression worked. It means something
    # different for the robust linear regression. In general, the larger the value the
    # better...
    line = img.get_regression([(255,255) if BINARY_VISIBLE else THRESHOLD], robust = True)

    if (line): img.draw_line(line.line(), color = 127)
    #print("FPS %f, mag = %s" % (clock.fps(), str(line.magnitude()) if (line) else "N/A"))
    #print("Theta = %s, rho = %s" % (str(line.theta()) if (line) else "N/A", str(line.rho()) if (line) else "N/A"))
    #print(line)
    rho = line.rho() if (line) else 361
    theta = line.theta() if (line) else 361
    x1 = line.x1() if (line) else -1
    x2 = line.x2() if (line) else -1

    new_time = pyb.millis()

    #error_x1 = desired_x1 - x1 #current error value

    #deltaerror_x1 = kd * abs( error_x1- olderror_x1 )/dt # derror/dtime

    #olderror_x1 = error_x1 #previous error value

    #print(rho)
    if (theta>=0 and theta<90):
        deflection_angle = -theta
    elif (theta>90 and theta<180):
        deflection_angle = (180-theta)
    else:
        deflection_angle = last_deflection  #"N/A"

    last_deflection = deflection_angle

    location = "N/A"
    print("Deflection Angle = ", deflection_angle)

    if (flag == 1):
        SetPWservo(240000+120000)
        SetPWmotor(240000+120000)
        time.sleep_ms(1000)
        flag =  0

    SetPWservo( int(240000 + 120000 + 120000 * update_pid(x1)) )

    old_time = new_time
    ##prev4_deltax1 = prev3_deltax1
    #prev3_deltax1 = prev2_deltax1
    #prev2_deltax1 = prev1_deltax1
    #prev1_deltax1 = deltax1

    #sum_deltax1 = prev3_deltax1 + prev2_deltax1 + prev1_deltax1 + deltax1


    prev_x1 = x1
    prev_x2 = x2
    #print ("old time = ",old_time)
    #old_time = new_time

    ##print(line)
    #print ("new time = ",new_time)
    ##print ("dt = ",dt)
    #print("delta x1 = ", deltax1)
    ##print("Location = ", location)
    #print("x1= ", x1)
    #print("x2= ", x2)
    print("output = ", update_pid(x1))



# About negative rho values:
#
# A [theta+0:-rho] tuple is the same as [theta+180:+rho].++
