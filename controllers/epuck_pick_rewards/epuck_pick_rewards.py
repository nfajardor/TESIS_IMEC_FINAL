#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import String
from controller import Robot

def epuck_callback(data):
    global route
    info = data.data.split('||')
    msg_id = int(info[0])
    if msg_id == 0:
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
    FORWARD_OFFSET = 0.00
    SIDEFRONT_ANGLE = 0.77
    FRONTSIDE_DISTANCE = 33.3
    TURNING_OFFSET = 0.03
    route = []
    
    #INITIALIZE THE SENSORS
    sfr = robot.getDevice('gs0')
    sfl = robot.getDevice('gs7')
    sfrr = robot.getDevice('gs1')
    sfll = robot.getDevice('gs6')
    sr = robot.getDevice('gs2')
    sl = robot.getDevice('gs5')
    sfr.enable(TIME_STEP)
    sfl.enable(TIME_STEP)
    sfrr.enable(TIME_STEP)
    sfll.enable(TIME_STEP)
    sr.enable(TIME_STEP)
    sl.enable(TIME_STEP)
    
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
    # Read the sensors:
    fr = sfr.getValue() 
    fl = sfl.getValue() 
    frr = sfrr.getValue() 
    fll = sfll.getValue() 
    r = sr.getValue() 
    l = sl.getValue() 
    s = ''
    s += '---------------------------\n'
    s += 'front:\n({},{})\n'.format(fl,fr)
    s += 'front2:\n({},{})\n'.format(fll,frr)
    s += 'sides:\n({},{})\n'.format(l,r)
    #print(s)
    

    pass

