#Start with imports, ie: import the wrapper
#import other libraries as needed
import TMMC_Wrapper
import rclpy
import numpy as np
import math
from ultralytics import YOLO
import time

#Start ros with initializing the rclpy object
if not rclpy.ok():
    rclpy.init()

TMMC_Wrapper.is_SIM = True
if not TMMC_Wrapper.is_SIM:
    #Specify hardware api
    TMMC_Wrapper.use_hardware()
    
if not "robot" in globals():
    robot = TMMC_Wrapper.Robot()

#Debug messaging 
print("running main")
#Testing of the test
#start processes
#add starter functions here
robot.start_keyboard_control()   #this one is just pure keyboard control
#rclpy,spin_once is a function that updates the ros topics once
rclpy.spin_once(robot, timeout_sec=0.1)

model = YOLO('yolov8n.pt')

#run control functions on loop
try:
    print("Entering the robot loop which cycles until the srcipt is stopped")
    while True:
        print("TEST1")
        # robot.reset_odometry() # make the robot think it is at position (0,0,0)
        # # Display raw data: Odometry based position
        # robot.odom_future = rclpy.Future()
        # pose1 = robot.spin_until_future_completed(robot.odom_future).pose.pose
        # print(pose1.position)
        # print(pose1.orientation)
 

        img = robot.rosImg_to_cv2()


        stop_sign_detected, x1, y1, x2, y2 = robot.ML_predict_stop_sign(model, img)
        print("stop sign detected: ", stop_sign_detected)
        distance_to_stop = -1
        if(stop_sign_detected):
            distance_to_stop = ((2*147.6375*640)/((y2-y1)*31.75))
            print("Distance to Stop: "+ distance_to_stop)
        if(distance_to_stop<10):
            robot.stop_keyboard_control()
            robot.set_cmd_vel(0, 0, 1)
            time.sleep(200)
            robot.start_keyboard_control()


        #rclpy,spin_once is a function that updates the ros topics once
        rclpy.spin_once(robot, timeout_sec=0.1)
        #Add looping functionality here
        
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    #add functionality to ending processes here
    robot.destroy_node()
    rclpy.shutdown()
