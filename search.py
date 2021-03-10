#!/usr/bin/env python3

'''COM2009 ev3dev Assignment2'''



import os

import sys

import time

import ev3dev.ev3 as ev3

from random import *



'''Provided Functions'''



def debug_print(*args, **kwargs):

    '''Print debug messages to stderr.



    This shows up in the output panel in VS Code.

    '''

    print(*args, **kwargs, file=sys.stderr)





def reset_console():

    '''Resets the console to the default state'''

    print('\x1Bc', end='')





def set_cursor(state):

    '''Turn the cursor on or off'''

    if state:

        print('\x1B[?25h', end='')

    else:

        print('\x1B[?25l', end='')





def set_font(name):

    '''Sets the console font



    A full list of fonts can be found with `ls /usr/share/consolefonts`

    '''

    os.system('setfont ' + name)



''' End provided functions '''





def debug_printer(var_name, var_val):

    debug_print(var_name)

    debug_print(var_val)







def actuate(m_left, m_right, v, dv, leftFlip, rightFlip):

    ''' Function to actuate motors 



    Takes 2 motors as inputs, v for base speed, dv for diff 

    returns nil

    '''



    left = (v+dv)*leftFlip

    right = (v-dv)*rightFlip



    '''debug_printer("Fed into motor left:", (left))

    debug_printer("Fed into right motor:", (right))'''



    m_left.run_direct(duty_cycle_sp=left) 

    m_right.run_direct(duty_cycle_sp=right)



def get_response(kp, ki, kd, dt, cur_error, prev_error, integral):

    '''Get PID response'''

    integral = 0

    integral+= cur_error * dt

    derivative = (cur_error - prev_error) / dt



    return (kp * cur_error) + (ki * integral) + (kd * derivative)





''' hk error metric - difference of sensors '''

def hk_metric(l_dist, r_dist):

    return (l_dist-r_dist)





def theo_metric(l_dist, r_dist):

    return ((l_dist+r_dist)/2)



def theo_jose_hybrid_metric(l_dist, r_dist):

    if l_dist > 2500:

        return (r_dist / 2)

    elif r_dist > 2500:

        return (l_dist / 2)

    else:

        return theo_metric(l_dist, r_dist)





# Motor vars

m_left = ev3.LargeMotor('outB')

m_right = ev3.LargeMotor('outC')



# Sensors

us_left = ev3.UltrasonicSensor('in3')

us_right = ev3.UltrasonicSensor('in2')



cs = ev3.ColorSensor('in1')

cs.mode = 'COL-AMBIENT'

gy = ev3.GyroSensor('in4')

gy.mode = 'GYRO-ANG'





global runWalk

global runSearch







def walk():

    # Sampling rate

    sampling_rate = 0.0001


    # motor speed range

    max_speed = 70

    base_speed = 40



    # Set distance

    min_dist = 300



    # PID - Initialise

    integral = 0

    last_error = 0

    ku = 8

    tu = 0.1



    # Zieger Nichols Calculated value

    kp = 1

    ki = 0.8

    kd = 0.168



    ''' Actual dt does not equal our sampling rate. Code may take longer to run.

        So we time everytime the code runs, this is inaccurate at first but should be better 

        in the long run '''

    time_then = time.time()

    time.sleep(sampling_rate)

    error = hk_metric(us_left.value(), us_right.value())

    #error_rate = theo_metric(us.left_value(), us.right_value())

    #error_rate = jose_metric(us.left_value(), us.right_value())



    time_now = time.time()

    dt = time_now - time_then

    time_then = time_now



    #debug_printer("Error from Metric used:", error)



    dv = get_response(kp, ki, kd, dt, error, last_error, integral)



    # Overspeed stopper



    if dv + base_speed > max_speed:

        dv = max_speed - base_speed

    elif dv - base_speed < -max_speed:

        dv = base_speed - max_speed



    actuate(m_left, m_right, base_speed, dv, -1, -1)



    # Copy previous error rate

    last_error = error



def setNewDirection():

    randomWalkAngle = randint(0, 270)
    startingAngle = gy.value()
    currentAngle = gy.value()

    while currentAngle < (startingAngle + randomWalkAngle):
        actuate(m_left, m_right, 20, 20, -1, 1)
        currentAngle = gy.value()





def search():

    global runWalk

    global runSearch

    units = gy.units

    angle = gy.value()

    startingAngle = angle

    currentAngle = angle

    currentCSValue = cs.value()

    startCSValue = cs.value()

    #debug_print(str(angle) + " " + units)

    #debug_print("CS VALUE:" + str(cs.value()))

    
    DL = [1, -1]
    direction = sample(DL, 1)

    while runSearch == True:
        currentAngle = gy.value()
        if gy.value() > currentAngle:

            #if the colour sensor value is triggered then stop searching and start walking
            if cs.value() > (startCSValue + 3):
                debug_print("Sensor stopped search")
                runSearch = False

                runWalk = True

        else:

            #update CS value correctly, fix the robot, find the light, win the competition

            currentCSValue = cs.value()

            actuate(m_left, m_right, 20, 20, direction[0], direction[0])

        if currentAngle > (startingAngle + (270 * direction[0] * -1)):
            debug_print("Rotation stopped search")
            setNewDirection()
            runSearch = False
            runWalk = True








def main():

    '''Main calling Function'''

    debug_print("Running")

    global runWalk

    global runSearch

    runWalk = False

    runSearch = True

    lightFound = False


    while lightFound == False:

        if cs.value() > 30:

            runWalk = False

            runSearch = False

            lightFound = True

            debug_print("LIGHT FOUND!!!")

        elif runWalk == True:

            debug_print("Walk")
            
            startTime = time.time()

            timePassed = 0
            # Random walk time
            randomWalkTime = randint(0, 7)
            debug_print(randomWalkTime)

            while cs.value() < 20 and timePassed < randomWalkTime:

                currentTime = time.time()

                walk()

                timePassed = currentTime - startTime

            while cs.value() > 20 and timePassed < 0.5:
                
                currentTime = time.time()
                walk()
                timePassed = currentTime - startTime

            runWalk = False
            runSearch = True

        

        elif cs.value() > 20 or runSearch == True:

            debug_print("Search")

            search()










if __name__ == '__main__':

    main()
