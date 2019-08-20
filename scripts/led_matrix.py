#! /usr/bin/env python

import rospy
from sense_hat import SenseHat

rainbow_pixels = [
    [255, 0, 0], [255, 0, 0], [255, 87, 0], [255, 196, 0], [205, 255, 0], [95, 255, 0], [0, 255, 13], [0, 255, 122],
    [255, 0, 0], [255, 96, 0], [255, 205, 0], [196, 255, 0], [87, 255, 0], [0, 255, 22], [0, 255, 131], [0, 255, 240],
    [255, 105, 0], [255, 214, 0], [187, 255, 0], [78, 255, 0], [0, 255, 30], [0, 255, 140], [0, 255, 248], [0, 152, 255],
    [255, 223, 0], [178, 255, 0], [70, 255, 0], [0, 255, 40], [0, 255, 148], [0, 253, 255], [0, 144, 255], [0, 34, 255],
    [170, 255, 0], [61, 255, 0], [0, 255, 48], [0, 255, 157], [0, 243, 255], [0, 134, 255], [0, 26, 255], [83, 0, 255],
    [52, 255, 0], [0, 255, 57], [0, 255, 166], [0, 235, 255], [0, 126, 255], [0, 17, 255], [92, 0, 255], [201, 0, 255],
    [0, 255, 66], [0, 255, 174], [0, 226, 255], [0, 117, 255], [0, 8, 255], [100, 0, 255], [210, 0, 255], [255, 0, 192],
    [0, 255, 183], [0, 217, 255], [0, 109, 255], [0, 0, 255], [110, 0, 255], [218, 0, 255], [255, 0, 183], [255, 0, 74]
]

colorshift_pixel = [255,0,0]

def rainbow():

    for pix in rainbow_pixels:
        r = pix[0]
        g = pix[1]
        b = pix[2]

        if (r == 255 and g < 255 and b == 0):
            g += 1

        if (g == 255 and r > 0 and b == 0):
            r -= 1

        if (g == 255 and b < 255 and r == 0):
            b += 1

        if (b == 255 and g > 0 and r == 0):
            g -= 1

        if (b == 255 and r < 255 and g == 0):
            r += 1

        if (r == 255 and b > 0 and g == 0):
            b -= 1

        pix[0] = r
        pix[1] = g
        pix[2] = b

def colorshift():

    r = colorshift_pixel[0]
    g = colorshift_pixel[1]
    b = colorshift_pixel[2]

    if (r == 255 and g < 255 and b == 0):
        g += 1

    if (g == 255 and r > 0 and b == 0):
        r -= 1

    if (g == 255 and b < 255 and r == 0):
        b += 1

    if (b == 255 and g > 0 and r == 0):
        g -= 1

    if (b == 255 and r < 255 and g == 0):
        r += 1

    if (r == 255 and b > 0 and g == 0):
        b -= 1


    colorshift_pixel[0] = r
    colorshift_pixel[1] = g
    colorshift_pixel[2] = b
   


if __name__ == '__main__':
    
    sense = SenseHat()


    freq = 100
    rospy.init_node("sense_hat_ledmatrix", anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + " Initializing node with frequency %s Hz", freq)
    rate = rospy.Rate(freq)

    try:

        rospy.loginfo(rospy.get_caller_id() + " Starting...")
        pattern = 2
        last_time = rospy.Time.now()
        interval = rospy.Duration(10)
        while not rospy.is_shutdown():
            time_now = rospy.Time.now()
            delta_time = time_now - last_time


            if(delta_time >= interval):
                if(pattern == 1):
                    pattern = 2
                    rospy.loginfo(rospy.get_caller_id() + " Running Rainbow Pattern...")
                else:
                    pattern = 1

                    rospy.loginfo(rospy.get_caller_id() + " Running Color Shift Pattern...")
                last_time = time_now

            if(pattern == 1):
                colorshift()
                sense.clear(colorshift_pixel)
            elif(pattern == 2):
                rainbow()
                sense.set_pixels(rainbow_pixels)
            else:
                pattern = 1


        rospy.loginfo(rospy.get_caller_id() + " Quit...")

    except rospy.ROSInterruptException:
        pass

