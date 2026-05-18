#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time

def draw_I_fixed_Z():
    # 1. Define Publishers for the joint command topics
    # Note: Joint_r is now using the /joint_r_position_controller topic
    pub_x = rospy.Publisher('/joint_x_position_controller/command', Float64, queue_size=10)
    pub_y = rospy.Publisher('/joint_y_position_controller/command', Float64, queue_size=10)
    pub_z = rospy.Publisher('/joint_z_position_controller/command', Float64, queue_size=10)
    pub_r = rospy.Publisher('/joint_r_position_controller/command', Float64, queue_size=10) 

    rospy.init_node('draw_letter_node', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz

    # Set initial default positions and control constants
    HOME_X = 0.05
    HOME_Y = 0.00
    HOME_Z_FIXED = 0.00 # Z is fixed at the surface level (0.00 for the 'pen')
    HOME_R = 0.0 # R is fixed at its initial position
    MOVE_SPEED = 2.0 # Time (seconds) to wait for joint to reach target

    # --- Sequence of Waypoints to Draw a capital 'I' (Fixed Z/R) ---
    # Waypoints are defined as (X, Y) positions. The float MOVE_SPEED signals a pause.
    
    # [Point X, Point Y]
    waypoints = [
        # 1. Initial Position (Go to the top-left of the letter I)
        (0.25, -0.15), 
        MOVE_SPEED, # PAUSE: Wait for robot to get to the starting X/Y

        # 2. Draw Top Bar (Horizontal movement of 'I', Joint_y moves)
        (0.25, -0.05),
        MOVE_SPEED, # PAUSE

        # 3. Move to start of vertical stem (Move X up/down, Y left/right)
        (0.25, -0.10),
        MOVE_SPEED, # PAUSE

        # 4. Draw the Vertical Stem (X-axis moves down/up)
        (0.05, -0.10),
        MOVE_SPEED, # PAUSE
        
        # 5. Move to the start of the bottom bar (Y-axis moves left/right)
        (0.05, -0.15),
        MOVE_SPEED, # PAUSE

        # 6. Draw Bottom Bar (Horizontal movement of 'I', Joint_y moves)
        (0.05, -0.05),
        MOVE_SPEED, # PAUSE
        
        # 7. Return home
        (HOME_X, HOME_Y),
    ]

    rospy.loginfo("Starting robot drawing sequence (Fixed Z/R)...")

    # Set initial command values and wait for robot to move to the safe start position
    pub_x.publish(HOME_X)
    pub_y.publish(HOME_Y)
    pub_z.publish(HOME_Z_FIXED) # Set Z to the fixed drawing height
    pub_r.publish(HOME_R)       # Set R to the fixed zero position
    time.sleep(3) # Initial wait for robot to go to start position

    for waypoint in waypoints:
        # Check if the waypoint is a float (the pause signal)
        if isinstance(waypoint, float): 
            rospy.loginfo(f"Waiting for {waypoint} seconds for movement to complete.")
            time.sleep(waypoint) # Execute sleep inside the loop
            continue
        
        # If it's not a float, it MUST be the coordinate tuple (X, Y)
        try:
            target_x, target_y = waypoint
        except ValueError:
            rospy.logerr(f"Skipping malformed waypoint: {waypoint}")
            continue

        # Publish the new joint positions (Z and R are held constant)
        pub_x.publish(target_x)
        pub_y.publish(target_y)
        
        rospy.loginfo(f"Moving to: X={target_x:.2f}, Y={target_y:.2f} (Z/R fixed)")
        rate.sleep()

    rospy.loginfo("Drawing sequence complete. Returning to final home position.")
    pub_x.publish(HOME_X)
    pub_y.publish(HOME_Y)
    pub_z.publish(HOME_Z_FIXED)
    pub_r.publish(HOME_R)


if __name__ == '__main__':
    try:
        draw_I_fixed_Z()
    except rospy.ROSInterruptException:
        pass