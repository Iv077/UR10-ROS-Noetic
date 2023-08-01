#!/usr/bin/env python3
#---------------Import Libraries and msgs---------------:

from platform import win32_edition
from defer import return_value
import rospy
import time
import math
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
import tf2_msgs.msg
from tf.transformations import euler_from_quaternion
from collections import Counter
from control_lib import UR_Controller

#----------------------Initialise-------------------------:

print("Please Wait While System Starts Up...")
rospy.init_node("example_assignment", anonymous=False)
ur_script = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)
ur_con = UR_Controller()
time.sleep(2)
print("System Started")

#------------------FUNTIONS/DEFINITIONS -------------------:

    #   First function made to go true the axis of X and Z and stopping ones 
    #   it reaches the minimal cartesian error from both axis rotations:


def cart():                                                                                         # Function in which we add code that we want, making us not to write the full code again, enouth with writing: cart()

    my_pos = ur_con.get_pose()                                                                      # Detection of the actual position of the robot
    old_errors = ur_con.check_errors(my_pos)                                                        # Shows the error between the actual position and the desired one
    print("Errors:", (old_errors.ee_trans_error),"m")                                               # Prints directional error in meters


    cartesian = True                                                                                # Loop bolean condition True
    while cartesian:                                                                                # Infinity loop until the directional error beacomes smaller than 0.05 - then changed to False or brake
        if old_errors.ee_trans_error > 0.05:                                                        # Statement/Condition making run the code until the position error beacomes smaller than 0.05
            the_best_y = True                                                                       # Loop bolean condition True - makes the loop work
            while the_best_y:                                                                       # Sub infinity loop, stops when finds the smalles cartesian error by moving in y axis
                my_pos.position.y += 0.01                                                           # Movement of the robot postion in Y axis by 0.01
                ur_script.publish(ur_con.generate_move_l(my_pos))                                   # Sends the comand to the ur10: linear movement by 0.01
                time.sleep(2)                                                                       # Delay to allow the rotation to be completed fully and a more accurate error
                errors_y = ur_con.check_errors(my_pos)                                              # Shows the error between the actual position and the desired one
                print("Errors in Y - 0:", (old_errors.ee_trans_error),"m")                          # Prints directional error in meters

                if errors_y.ee_trans_error < old_errors.ee_trans_error:                             # Statement/Condition that compares 2 errors by which is the smaller
                    old_errors = errors_y                                                           # Applying values from one error to other  - Unprades the error 
                    my_pos.position.y += 0.01                                                       # Movement of the robot postion in Y axis by 0.01
                    ur_script.publish(ur_con.generate_move_l(my_pos))                               # Sends the comand to the ur10: linear movement by 0.01
                    time.sleep(2)                                                                   # Delay to allow the rotation to be completed fully and a more accurate error
                    print("Errors in Y - 1:", (old_errors.ee_trans_error),"m")                      # Prints directional error in meters
                    

                elif errors_y.ee_trans_error > old_errors.ee_trans_error:                           # Statement/Condition that compares 2 directional errors by which is the smaller
                    my_pos.position.y -= 0.01                                                       # Movement of the robot postion in Y axis by -0.01
                    ur_script.publish(ur_con.generate_move_l(my_pos))                               # Sends the comand to the ur10: linear movement by -0.01
                    the_best_y = False                                                              # Loop bolean condition False - makes the loop stop

            the_best_x = True                                                                       # Loop bolean condition True - makes the loop work         
            while the_best_x:                                                                       # Sub infinity loop, stops when finds the smalles cartesian error by moving in y axis
                my_pos.position.x += 0.01                                                           # Movement of the robot postion in Y axis by 0.01
                ur_script.publish(ur_con.generate_move_l(my_pos))                                   # Sends the comand to the ur10: linear movement by 0.01
                time.sleep(2)                                                                       # Delay to allow the rotation to be completed fully and a more accurate error
                errors_x = ur_con.check_errors(my_pos)                                              # Shows the error between the actual position and the desired one
                print("Errors in X - 0:", (old_errors.ee_trans_error),"m")                          # Prints the error

                if errors_x.ee_trans_error < old_errors.ee_trans_error:                             # Statement/Condition that compares 2 directional errors by which is the smaller
                    old_errors = errors_x                                                           # Applying values from one error to other  - Unprades the error 
                    my_pos.position.x += 0.01                                                       # Movement of the robot postion in X axis by 0.01
                    ur_script.publish(ur_con.generate_move_l(my_pos))                               # Sends the comand to the ur10: linear movement by 0.01
                    time.sleep(2)                                                                   # Delay to allow the rotation to be completed fully and a more accurate error
                    print("Errors in X - 1:", (old_errors.ee_trans_error),"m")                      # Prints the error
                    

                elif errors_x.ee_trans_error > old_errors.ee_trans_error:                           # Statement/Condition that compares 2 directional errors by which is the smaller
                    my_pos.position.x -= 0.01                                                       # Movement of the robot postion in X axis by -0.01
                    ur_script.publish(ur_con.generate_move_l(my_pos))                               # Sends the comand to the ur10: linear movement by -0.01
                    the_best_x = False                                                              # Loop bolean condition False - makes the loop stop


        else:                                                                                       # Statement/Condition that happens if the first one didnt happend
            cartesian = False                                                                       # Loop bolean condition False - makes the loop stop


    my_pos = ur_con.get_pose()                                                                      # Detection of the actual position of the robot
    old_errors = ur_con.check_errors(my_pos)                                                        # Shows the error between the actual position and the desired one
    print("Errors:", (old_errors.ee_trans_error),"m")                                               # Prints directional error in meters

#   Point of double check in the loop, if by any chance the error moved by itself 
#   to check again the rotational error and to re-run the loop for finding the best X and Y:

    if old_errors.ee_trans_error > 0.05:                                                            # Statement/Condition that check if the error is bigger than 0.05
        cart()                                                                                      # Calling the code that is contained in car() withour writing everything again


#   Second function made to go true the axis of RX, RY and RZ and stopping ones 
#   it reaches the minimal orientational error from both axis rotations:

def rotate_end_effector(mode,rad):                                                                  # Function used to rotate the tool in either in relation to rx, ry or rz depending on input to the function
    if mode == "rz":                                                                                # When input to the function is "rx" rotates the tool in relation to rx
        command = ur_con.rotate_tool(0, 0, rad)                                                     # Sends the comand to the ur10: linear movement by rotating in relation to rx 
        ur_script.publish(command)                                                                  # # Sends the comand to the ur10
    if mode == "ry":                                                                                # When input to the function is "ry" rotate the tool in relation to ry
        command = ur_con.rotate_tool(0, rad, 0)                                                     #  Sends the comand to the ur10: linear movement by rotating in relation to ry
        ur_script.publish(command)                                                                  # # Sends the comand to the ur10
    if mode == "rx":                                                                                # When input to the function is "rz" rotate the tool in relation to rz
        command = ur_con.rotate_tool(rad, 0, 0)                                                     #  Sends the comand to the ur10: linear movement by rotating in relation to rz
        ur_script.publish(command)                                                                  # # Sends the comand to the ur10
    my_pos = ur_con.get_pose()                                                                      # Get the current position of the robot 
    rot_error = ur_con.check_errors(my_pos).ee_rot_error                                            # Checks the current rotational error of the end effector in relation of the desired position
    time.sleep(2)                                                                                   # Delay to allow the rotation to be completed fully and a more accurate error is returned
    return rot_error                                                                                # Return rotational error of the end effector



#---------------------------MAIN CODE----------------------------:

#Home robot
home_waypoint = [-1.5708, -1.5708, -1.5708, -0.802851, 1.58825, -0.03106686]                        # Initial position of eacj joint set by a list with angles in rads
command = ur_con.generate_move_j(home_waypoint)                                                     # Generation an order for the controler to move to the positon in parentesis/initial position
ur_script.publish(command)                                                                          # Sends the comand to the ur10
time.sleep(10) # TO DO replace this with check                                                      # Break before next action

# # Some code that checks error and moves accordingly
my_pos = ur_con.get_pose()                                                                          # Detection of the actual position of the robot
old_errors = ur_con.check_errors(my_pos)                                                            # Shows the error between the actual position and the desired one
print("Errors:", (old_errors.ee_trans_error),"m")                                                   # Prints directional error in meters

#---------------POSITIONAL ERROR SOLUTION -----------------------:
cart()                                                                                              # Calling the code that is contained in car() withour writing everything again

my_pos = ur_con.get_pose()                                                                          # Detection of the actual position of the robot
errors = ur_con.check_errors(my_pos)                                                                # Shows the error between the actual position and the desired one
print("Cartesian error:", (errors.ee_trans_error),"m")                                              # Prints directional error in meters
print("Orientational error:", errors.ee_rot_error, "degrees")                                       # Prints rotational error in degrees

#---------------ROTATIONAL ERROR SOLUTION -----------------------:

end_effector_mode = [["rx", 0.1],["ry", 0.1],["rz",0.1]]                                              # Row, Pich and Jow with movements by 0.1
for j, mode in enumerate(end_effector_mode):                                                        # Loop tha goes true through every list or element in the list "end_effector_mode"                                                                                      
    my_pos = ur_con.get_pose()                                                                      # Get the current position of the robot 
    rot_error = ur_con.check_errors(my_pos).ee_rot_error                                            # Checks the current rotational error of the end effector in relation of the desired position
    while rot_error >= 5:                                                                           # Code will continue in a loop if rotational error of the end effector is more than or equal to 5 degrees                                                                         
        new_rot_error = rotate_end_effector(mode[0], mode[1])                                       # Obtains the current rotational error from the end effector with the function rotate_end_effector with input of the first and second element of the list in the list                                                                                                                            
        if new_rot_error > rot_error:                                                               # Statement/Condition that compares 2 rotational errors by which is the bigger
            new_rot_error = rotate_end_effector(mode[0], mode[1]*(-2))                              # Returns negative and double the seond element of the list in the list(the radian)               
            if new_rot_error < rot_error:                                                           # Statement/Condition that compares 2 rotational errors by which is the smaller
                rot_error = new_rot_error                                                           # Applying values from one error to other  - Unprades the error  
        else:                                                                                       
            rot_error = new_rot_error                                                               # Applying values from one error to other  - Unprades the error                                                                       
            print("Orientational error: ", rot_error, "degrees")                                    # Prints rotational error in degrees              




 