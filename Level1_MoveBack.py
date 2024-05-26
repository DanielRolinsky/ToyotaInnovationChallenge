#Start with imports, ie: import the wrapper
#import other libraries as needed
import TMMC_Wrapper
import rclpy
import numpy as np
import math

#Start ros with initializing the rclpy object
if not rclpy.ok():
    rclpy.init()

TMMC_Wrapper.is_SIM = False
if not TMMC_Wrapper.is_SIM:
    #Specify hardware api
    TMMC_Wrapper.use_hardware()
    
if not "robot" in globals():
    robot = TMMC_Wrapper.Robot()

#Debug messaging 
print("running main")

#start processes
#add starter functions here
robot.start_keyboard_control()

CONST_reverseVelocity = -0.25
CONST_boundary = 0.3

#rclpy,spin_once is a function that updates the ros topics once
rclpy.spin_once(robot, timeout_sec=0.1)

#run control functions on loop
try:
    print("Entering the robot loop which cycles until the srcipt is stopped")
    while True:

        #rclpy,spin_once is a function that updates the ros topics once
        rclpy.spin_once(robot, timeout_sec=0.1)

        #Add looping functionality here
        if robot.last_scan_msg:
            minDistance, minAngle = robot.detect_obstacle(robot.last_scan_msg.ranges)
            print(">>>minDis & minAngle is: " + str(minDistance) + ", " +str(minAngle))
            if minDistance != -1 and minDistance <= CONST_boundary:
                robot.stop_keyboard_control()
                robot.set_cmd_vel(CONST_reverseVelocity, 0, 0.5)
                robot.set_cmd_vel(0, 0, 1)
                minDistance, minAngle = robot.detect_obstacle(robot.last_scan_msg.ranges)
                disToTravel = CONST_boundary + 0.05 - minDistance
                robot.set_cmd_vel(CONST_reverseVelocity, 0, disToTravel/CONST_reverseVelocity + 0.5) #Forward is backwards b/c lidar is on the back
                robot.start_keyboard_control()


        
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    #add functionality to ending processes here
    robot.destroy_node()
    rclpy.shutdown()
