import rospy
from std_msgs.msg import String
from controller import Robot
import math
import time

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
    route = [[0,1],[1,1],[1,2],[4,1]]
    route_length = len(route)
    step = 0
    state = 3 #0: rotate in place. 1: turn right. 2: turn left. 3: awaiting instructions. 4: go forward
    
    #Important variables
    obj_r = 0
    obj_l = 0
    unit_rot = math.pi/2
    blocks = 0
    inBlock = True
    turning = False
    
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
    print("Setting up the motors")
    lMotor = robot.getDevice('left wheel motor')
    rMotor = robot.getDevice('right wheel motor')
    lMotor.setPosition(float('inf'))
    rMotor.setPosition(float('inf'))
    lMotor.setVelocity(L_SPEED)
    rMotor.setVelocity(R_SPEED)
    
    

    print("About to enter the while")
    while robot.step(TIME_STEP) != -1:
        measure_sensors()
        #s = ''
        #s += '---------------------------\n'
        #s += 'front:\n({},{})\n'.format(fl,fr)
        #s += 'front2:\n({},{})\n'.format(fll,frr)
        #s += 'sides:\n({},{})\n'.format(l,r)
        #s += 'wheels:\n({},{})\n'.format(lw,rw)
        #print(s)
        #print("Route: {}\nLength: {}\nStep: {}".format(route,route_length,step))
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
                    blocks = amm
                    inBlock = True
                    turning = False
                    lMotor.setVelocity(L_SPEED)
                    rMotor.setVelocity(R_SPEED)
                elif state == 2:
                    #Set up the vars to turn left
                    pass
                elif state == 4:
                    #Set up the vars to go forward
                    pass
            else:
                measure_sensors()
                if state == 0:
                    if rw > obj_r or lw < obj_l:
                        print("rot ended")
                        lMotor.setVelocity(0)
                        rMotor.setVelocity(0)
                        step += 1
                        state = 3
                elif state == 1:
                    print("we gonna turn right")
                elif state == 2:
                    pass
                elif state == 4:
                    pass
                    
                