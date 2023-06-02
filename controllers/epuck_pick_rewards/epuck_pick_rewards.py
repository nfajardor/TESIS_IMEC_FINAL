#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import String
from controller import Robot
import math

def inRange(value):
    global SIDEFRONT_ANGLE
    global TURNING_OFFSET
    x = math.cos(SIDEFRONT_ANGLE)
    if (value > (x*(1.0 - TURNING_OFFSET))) and (value < (x*(1.0 + TURNING_OFFSET))):
        return True
    return False
def measure_sensors():
    global fr 
    global fl 
    global frr 
    global fll 
    global r 
    global l 
    global rw
    global lw
    fr = sfr.getValue() 
    fl = sfl.getValue() 
    frr = sfrr.getValue() 
    fll = sfll.getValue() 
    r = sr.getValue() 
    l = sl.getValue() 
    rw = r_sensor.getValue()
    lw = l_sensor.getValue()
    s = ''
    s += '---------------------------\n'
    s += '{}:'.format(name)
    #s += 'front:\n({},{})\n'.format(fl,fr)
    #s += 'front2:\n({},{})\n'.format(fll,frr)
    #s += 'sides:\n({},{})\n'.format(l,r)
    #s += 'wheels:\n({},{})\n'.format(lw,rw)
    #print(s)
    

def adjust_speed():
    global r
    global l
    measure_sensors()
    if r < 600 and l < 600:
        d = r - l
        d_max = 100
        d_abs = abs(d)
        mapped_offset = d_abs*FORWARD_OFFSET/d_max
        if d_abs > 50:
            if d > 0:
                lMotor.setVelocity(L_SPEED*(1+mapped_offset))
                rMotor.setVelocity(R_SPEED*(1-mapped_offset))
            else:
                lMotor.setVelocity(L_SPEED*(1-mapped_offset))
                rMotor.setVelocity(R_SPEED*(1+mapped_offset))
    else:
        if l < 160:
            lMotor.setVelocity(L_SPEED*(1+FORWARD_OFFSET))
            rMotor.setVelocity(R_SPEED*(1-FORWARD_OFFSET))
        elif r < 160:
            lMotor.setVelocity(L_SPEED*(1-FORWARD_OFFSET))
            rMotor.setVelocity(R_SPEED*(1+FORWARD_OFFSET))

def epuck_callback(data):
    global route
    info = data.data.split('||')
    msg_id = int(info[0])
    if msg_id == 0 and len(route) == 0:
        print("{} ROUTE is: {}".format(name,info[1]))
        route = eval(info[1])
        msg = "0||{}".format(name)
        pub.publish(msg)
    

if __name__ == '__main__':
    # create the Robot instance.
    robot = Robot()
    name = robot.getName()
    print("I am {}".format(name))
    #SET GLOBAL CONSTANTS
    MAX_SPEED = 1
    TIME_STEP = int(robot.getBasicTimeStep())
    R_SPEED = MAX_SPEED
    L_SPEED = MAX_SPEED
    DIAMETER = 71
    SSENSOR_OFFSET = 4.5
    WHEEL_RADIUS = 20.5
    WHEEL_DISTANCE = 53
    FORWARD_OFFSET = 0.04
    SIDEFRONT_ANGLE = 0.77
    FRONTSIDE_DISTANCE = 33.3
    TURNING_OFFSET = 0.04
    route = []
    route_length = len(route)
    step = 0
    state = 3 #0: rotate in place. 1: turn right. 2: turn left. 3: awaiting instructions. 4: go forward
    #Important variables
    obj_r = 0
    obj_l = 0
    unit_rot = math.pi/2
    cur_blocks = 0
    obj_blocks = 0
    inBlock = True
    turning = False
    last_reading = 0
    ended = False
    just_rotated = False
    
    #INITIALIZE THE SENSORS
    print("Initializing sensors")
    sfr = robot.getDevice('gs0')
    sfl = robot.getDevice('gs7')
    sfrr = robot.getDevice('gs1')
    sfll = robot.getDevice('gs6')
    sr = robot.getDevice('gs2')
    sl = robot.getDevice('gs5')
    l_sensor = robot.getDevice('left wheel sensor')
    r_sensor = robot.getDevice('right wheel sensor')
    sfr.enable(TIME_STEP)
    sfl.enable(TIME_STEP)
    sfrr.enable(TIME_STEP)
    sfll.enable(TIME_STEP)
    sr.enable(TIME_STEP)
    sl.enable(TIME_STEP)
    l_sensor.enable(TIME_STEP)
    r_sensor.enable(TIME_STEP)
    
    #Sensor values
    fr = sfr.getValue() 
    fl = sfl.getValue() 
    frr = sfrr.getValue() 
    fll = sfll.getValue() 
    r = sr.getValue() 
    l = sl.getValue() 
    rw = r_sensor.getValue()
    lw = l_sensor.getValue()
    
    #SETUP THE WHEEL MOTORS
    lMotor = robot.getDevice('left wheel motor')
    rMotor = robot.getDevice('right wheel motor')
    lMotor.setPosition(float('inf'))
    rMotor.setPosition(float('inf'))
    lMotor.setVelocity(-L_SPEED)
    rMotor.setVelocity(R_SPEED)
    
    #SETUP ROS
    rospy.init_node(name)
    name2 = "{}ToMaster".format(name)
    pub = rospy.Publisher(name2,String,queue_size = 1)
    name2 = "MasterTo{}".format(name)
    sub = rospy.Subscriber(name2,String, callback = epuck_callback)
    
    while route == []:
        pass
    print("{} ROUTE OK".format(name))
    
    

    while robot.step(TIME_STEP) != -1:
        measure_sensors()
        # Read the sensors:
        #s = ''
        #s += '---------------------------\n'
        #s += 'front:\n({},{})\n'.format(fl,fr)
        #s += 'front2:\n({},{})\n'.format(fll,frr)
        #s += 'sides:\n({},{})\n'.format(l,r)
        #print(s)
        if step < len(route):
            cur_step = route[step]
            inst = cur_step[0]
            amm = cur_step[1]
            #print("Checking instruction {}".format(cur_step))
            if inst != state:
                measure_sensors()
                state = inst
                if state == 0:
                    #Set up the vars to rotate in place
                    print("gotta rotate in place: {},{}".format(lw,rw))
                    delta = amm * unit_rot
                    obj_r = rw + delta*1.3
                    obj_l = lw - delta*1.3
                    lMotor.setVelocity(-L_SPEED)
                    rMotor.setVelocity(R_SPEED)
                    
                elif state == 1:
                    #Set up the vars to turn right
                    print("Gotta turn right")
                    obj_blocks = amm
                    cur_blocks = 0
                    inBlock = True
                    turning = False
                    last_reading = 0
                    lMotor.setVelocity(L_SPEED)
                    rMotor.setVelocity(R_SPEED)
                    if just_rotated:
                        if r > 800:
                            obj_blocks += 1
                    just_rotated = False
                elif state == 2:
                    #Set up the vars to turn left
                    print("Gotta turn left")
                    obj_blocks = amm
                    cur_blocks = 0
                    inBlock = True
                    turning = False
                    last_reading = 0
                    lMotor.setVelocity(L_SPEED)
                    rMotor.setVelocity(R_SPEED)
                    if just_rotated:
                        if l > 800:
                            obj_blocks += 1
                    just_rotated = False
                    pass
                elif state == 4:
                    #Set up the vars to go forward
                    print("Gotta go forward")
                    delta = amm * 2*math.pi*(100/(2*WHEEL_RADIUS*math.pi))
                    obj_r = rw + delta*(1-FORWARD_OFFSET)
                    obj_l = lw + delta*(1-FORWARD_OFFSET)
                    lMotor.setVelocity(L_SPEED)
                    rMotor.setVelocity(R_SPEED)
                    just_rotated = False
            else:
                measure_sensors()
                if state == 0:
                    if rw > obj_r or lw < obj_l:
                        print("rot ended")
                        just_rotated = True
                        lMotor.setVelocity(0)
                        rMotor.setVelocity(0)
                        step += 1
                        state = 3
                elif state == 1:
                    
                    #print("we gonna turn right")
                    if not turning:
                        adjust_speed()
                        if inBlock:
                            if r > 350:
                                inBlock = False
                                cur_blocks += 1
                                if cur_blocks == obj_blocks:
                                    turning = True
                                else:
                                    inBlock = False
                            else:
                                last_reading = r
                        else:
                            if r < 250:
                                inBlock = True
                    else:
                        to_end = r/frr
                        if inRange(to_end) and r < 900 and frr < 900:
                            print("turning ended: {}/{}".format(r,frr))
                            lMotor.setVelocity(0)
                            rMotor.setVelocity(0)
                            step += 1
                            state = 3
                        else:
                            lMotor.setVelocity(L_SPEED*(last_reading/10+54.5)/(last_reading/10+31))
                            rMotor.setVelocity(R_SPEED*(last_reading/10+4.5)/(last_reading/10+31))
                elif state == 2:
                    #print("we gonna turn right")
                    if not turning:
                        adjust_speed()
                        if inBlock:
                            if l > 350:
                                inBlock = False
                                cur_blocks += 1
                                if cur_blocks == obj_blocks:
                                    turning = True
                                else:
                                    inBlock = False
                            else:
                                last_reading = l
                        else:
                            if l < 250:
                                inBlock = True
                    else:
                        to_end = l/fll
                        if inRange(to_end) and l < 900 and fll < 900:
                            print("turning ended: {}/{}".format(l,fll))
                            lMotor.setVelocity(0)
                            rMotor.setVelocity(0)
                            step += 1
                            state = 3
                        else:
                            rMotor.setVelocity(R_SPEED*(last_reading/10+54.5)/(last_reading/10+31))
                            lMotor.setVelocity(L_SPEED*(last_reading/10+4.5)/(last_reading/10+31))
                elif state == 4:
                    adjust_speed()
                    if rw > obj_r or lw > obj_l:
                        print("forward ended")
                        lMotor.setVelocity(0)
                        rMotor.setVelocity(0)
                        step += 1
                        state = 3
        else:
            if not ended:
                ended = True
                print("{} - ALL DONE".format(name))
    


