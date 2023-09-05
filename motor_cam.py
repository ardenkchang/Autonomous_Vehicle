##############################################################
#                           IMPORT
##############################################################
import sensor, image, time, math
from ulab import numpy as np
from pyb import Pin, Timer, LED
import pyb

##############################################################
#                           DEFINE
##############################################################

def DeinitTimer(TimeVar,CLKNum, frequ):
    time.sleep_ms(50)
    Timer.deinit(TimeVar)
    tim4 = Timer(CLKNum, freq= frequ)

def SetPWmotor(PWpercent):
    DeinitTimer(tim2, 2, 100)
    ch1 = tim2.channel(3, Timer.PWM, pin=Pin("P4"), pulse_width=PWpercent)

##############################################################
#               GLOBAL VARIABLES/INITIALIZATION
##############################################################

# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
# The below grayscale threshold is set to only find extremely bright white areas.
thresholds = (245, 255)
tim2 = Timer(2, freq=100)

#red_led   = LED(1)

#def led_control(x):
#    if   (x&1)==0: red_led.off()
#    elif (x&1)==1: red_led.on()
#time.sleep_ms(50)

p = pyb.Pin("P1", pyb.Pin.IN)
stop = p.value()

sensor.reset()                          # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)     # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)         # Wait for settings take effect.
sensor.set_auto_gain(False)             # must be turned off for color tracking
sensor.set_auto_whitebal(False)         # must be turned off for color tracking

clock = time.clock()                    # Create a clock object to track the FPS.
flag = 1


while(True):

        p = pyb.Pin("P1", pyb.Pin.IN)
        stop = p.value()

        print("stop value ", stop)
        time.sleep(0.5)

        if (stop == 1):
            while(True):
                SetPWmotor(240000+120000)


        clock.tick()                    # Update the FPS clock.
        img = sensor.snapshot()         # Take a picture and return the image.

        if (flag == 1):
        #SetPWservo(240000+120000)
            SetPWmotor(240000+120000)
            time.sleep_ms(1000)
            flag =  0

        top_blob = img.find_blobs([thresholds], roi = (0, 0, 80, 10), pixels_threshold=1,
                                   area_threshold=1, merge=True)
        down_blob = img.find_blobs([thresholds], roi = (0, 50, 80, 60), pixels_threshold=1,
                                   area_threshold=1, merge=True)

        x1 = 0
        x2 = 0

        for blob1 in top_blob:
            # These values depend on the blob not being circular - otherwise they will be shaky.

            # These values are stable all the time.
            img.draw_rectangle(blob1.rect(), color=127)
            img.draw_cross(blob1.cx(), blob1.cy(), color=127)
            x1 = blob1.cx()
            # Note - the blob rotation is unique to 0-180 only.


        for blob2 in down_blob:
            img.draw_rectangle(blob2.rect(), color=127)
            img.draw_cross(blob2.cx(), blob2.cy(), color=127)

            x2 = blob2.cx()

        xavg = (x1+x2)/2

        if (0 <= x1 < 25 and 0 <= x2 < 25):

            location = "LEFT"                   # LEFT
            SetPWmotor(240000+120000+32000)     # MOTOR SPEED

        elif (0 <= x1 < 55 and 55 <= x2 < 80):      # When the line is diagonal

            location = "LEFT"                   # LEFT
            SetPWmotor(240000+120000+30000)     # MOTOR SPEED

        elif (25 <= x1 < 80 and 0 <= x2 < 25):

            location = "RIGHT"                  # RIGHT
            SetPWmotor(240000+120000+30000)     # MOTOR SPEED

        elif (55 <= x1 < 80 and 55 <= x2 < 80):     # When the line is diagonal

            location = "RIGHT"                  # RIGHT
            SetPWmotor(240000+120000+32000)     # MOTOR SPEED

        elif (25 <= x1 < 55 and 25 <= x2 < 55):
            location = "CENTER"                 # CENTER
            SetPWmotor(240000+120000+38000)     # MOTOR SPEED

        else:
            location = "NONE"

        print(clock.fps())              # Note: OpenMV Cam runs about half as fast when connected
                                        # to the IDE. The FPS should increase once disconnected.
