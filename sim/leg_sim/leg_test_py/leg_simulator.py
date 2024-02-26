def leg_ik(l1,l2,end_x,end_y):
    ang_t1 = math.atan(-1*end_y / abs(end_x+0.00001))
    if(end_x < 0):
        ang_t1 += math.pi/2
    #print('ang_t1 ', ang_t1)
    l3 = math.sqrt(end_x**2 + end_y**2)

    tmp_1 = l1**2 + l2**2 - l3**2
    tmp_2 = 2*l1*l2
    ang_t2 = math.acos(tmp_1 / tmp_2)

    tmp_3 = l1**2 + l3**2 - l2**2
    tmp_4 = 2*l1*l3
    ang_t3 = math.acos(tmp_3 / tmp_4)

    ang_1 = math.pi - ang_t1 - ang_t3
    ang_2 = ang_t2

    print('test ang1', ang_1)
    print('test ang2', ang_2)
    return ang_1,ang_2

try:
    import sim
except:
    print ('Can not import the simluator')

import odrive
from odrive.enums import *

import time
import math

joint_a_init_pos = -2.25
joint_b_init_pos = 0

print("finding an odrive...")
my_drive = odrive.find_any()

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#my_drive = odrive.find_any("serial:/dev/ttyUSB0")

# Calibrate motor and wait for it to finish
print("starting calibration...")
my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while my_drive.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

my_drive.axis0.motor.config.current_lim = 10
time.sleep(1)
my_drive.axis1.motor.config.current_lim = 10
time.sleep(1)

my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

my_drive.axis0.controller.input_pos = joint_a_init_pos
time.sleep(2)

print ('Program started connected')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(1) #wait for startup

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()
    sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming

    err_code, joint_1 = sim.simxGetObjectHandle(clientID, 'joint_1', sim.simx_opmode_blocking)
    #print('handle1 state:', err_code)
    err_code, joint_2 = sim.simxGetObjectHandle(clientID, 'joint_2', sim.simx_opmode_blocking)
    err_code, base_liner_joint = sim.simxGetObjectHandle(clientID, 'base_liner_joint', sim.simx_opmode_blocking)
    print('joint handle init')

    sim_time = 60 # go for 60 seconds
    time_interval = 0.05
    interval_t = 0

    while time.time()-startTime > 0:
        returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data

    #when face to the leg, clockwise is positive, two joint are linked the same
    #pass the target in rad!

        if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            turn_a_sent = 0
            turn_b_sent = 0

            joint_a_ratio = 9
            joint_b_ratio = 13.5

            x_tmp = 0
            y_tmp = -0.35 + 0.1*math.sin(interval_t)
            interval_t += 0.1
            
            ang_1_o,ang_2_o = leg_ik(0.31,0.35,x_tmp,y_tmp)
            err_code = sim.simxSetJointTargetPosition(clientID, joint_1, -1 * ang_1_o, sim.simx_opmode_oneshot)
            err_code = sim.simxSetJointTargetPosition(clientID, joint_2, ang_2_o, sim.simx_opmode_oneshot)

            #set odrive
            turn_a_sent = ang_1_o/math.pi/2*joint_a_ratio + joint_a_init_pos
            my_drive.axis0.controller.input_pos = turn_a_sent

            turn_b_sent = ang_2_o/math.pi/2*joint_b_ratio + joint_b_init_pos
            my_drive.axis1.controller.input_pos = turn_b_sent
            
            #print('handle force state:', err_code)
            #print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window

        time.sleep(time_interval) #fixed time interval

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'2dof leg sim',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
