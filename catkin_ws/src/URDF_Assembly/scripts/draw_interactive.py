#!/usr/bin/env python

import rospy
import math
import time
import sys
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# =========================================================
# CONFIG & CONSTANTS
# =========================================================

# **TUNE THESE DELAYS**
LIFT_TIME = 1.0     
JUMP_SETTLE_TIME = 0.5

DRAWING_Z = 0.00    
LIFT_Z = 0.05       

X_OFFSET = 0.10     
Y_OFFSET = 0.00     

DRAW_SIZE = 0.05        # Height of all capital letters
LETTER_SPACING = 0.07

FRAME_ID = "world"
DRAW_RATE = 30      

# =========================================================
# GLOBALS & MARKER COUNTER (FOR RVIZ STROKE SEPARATION)
# =========================================================

pub_x = pub_y = pub_z = pub_r = None
marker_pub = None
rate = None
pen_is_down = False
# Global counter for robust Rviz stroke separation (CRITICAL FIX)
marker_id_counter = 0 
current_marker = None

# =========================================================
# RVIZ MARKER (Robust Separate Stroke Logic)
# =========================================================

def init_marker_system():
    pass 

def start_new_stroke():
    """Initializes a new Marker object with a unique ID for a single stroke."""
    global marker_id_counter, current_marker
    
    marker_id_counter += 1
    
    current_marker = Marker()
    current_marker.header.frame_id = FRAME_ID
    current_marker.ns = "drawing"
    current_marker.id = marker_id_counter 
    current_marker.type = Marker.LINE_STRIP 
    current_marker.action = Marker.ADD
    current_marker.scale.x = 0.003
    current_marker.color.r = 1.0
    current_marker.color.a = 1.0
    current_marker.pose.orientation.w = 1.0

def add_point(x, y):
    """Adds a point to the current marker and publishes."""
    if not pen_is_down or current_marker is None:
        return
    
    p = Point()
    p.x = x
    p.y = y
    p.z = DRAWING_Z
    current_marker.points.append(p)
    
    current_marker.header.stamp = rospy.Time.now()
    marker_pub.publish(current_marker)

# =========================================================
# MOTION
#=========================================================

def pen_up():
    global pen_is_down
    pub_z.publish(LIFT_Z)
    pen_is_down = False
    time.sleep(LIFT_TIME)

def pen_down():
    global pen_is_down
    pub_z.publish(DRAWING_Z)
    pen_is_down = True
    time.sleep(LIFT_TIME)

def move_to(x_draw, y_draw, draw=False):
    xj = x_draw + X_OFFSET
    yj = y_draw + Y_OFFSET

    pub_x.publish(xj)
    pub_y.publish(yj)
    pub_r.publish(0.0)

    if draw:
        add_point(xj, yj)
        rate.sleep() 
    else:
        time.sleep(JUMP_SETTLE_TIME)
        
# =========================================================
# BEZIER CURVE PRIMITIVE
# =========================================================

def get_bezier_point(p0, p1, p2, t):
    """Calculates a point on a Quadratic Bezier Curve. """
    x = (1 - t)**2 * p0[0] + 2 * (1 - t) * t * p1[0] + t**2 * p2[0]
    y = (1 - t)**2 * p0[1] + 2 * (1 - t) * t * p1[1] + t**2 * p2[1]
    return x, y

def draw_bezier(p0, p1, p2, steps=30):
    """Draws a curve defined by 3 points, treating it as one discrete stroke."""
    
    start_new_stroke() 

    # 1. Jump to start position (p0)
    pen_up() 
    move_to(p0[0], p0[1], draw=False) 
    
    # 2. Start drawing
    pen_down()

    # 3. Interpolate and draw steps
    for i in range(steps + 1):
        t = float(i) / steps
        x, y = get_bezier_point(p0, p1, p2, t)
        move_to(x, y, draw=True)

    # 4. Finish stroke
    pen_up()

# =========================================================
# CAPITAL LETTER DEFINITIONS (NEW D and A added)
# =========================================================

def D(x, y):
    # D: Two strokes - Stem, Arc/Bowl
    
    # 1. Stem (Top to Bottom)
    p_top = (x, y)
    p_bottom = (x, y - DRAW_SIZE)
    p_control_stem = (x, y - DRAW_SIZE/2)
    draw_bezier(p_top, p_control_stem, p_bottom, steps=10) 
    
    # 2. Arc/Bowl (Bottom to Top)
    p_arc_start = p_bottom
    p_arc_end = p_top 
    
    # Control point for a smooth, wide arc (semicircle approximation)
    p_arc_control = (x + DRAW_SIZE * 1.5, y - DRAW_SIZE/2)
    
    draw_bezier(p_arc_start, p_arc_control, p_arc_end, steps=30)
    
    
def R(x, y):
    # R: Three strokes - Stem, Arc/Bowl, Diagonal Leg
    
    # --- STROKE 1: Stem (Top to Bottom) ---
    p_top = (x, y)
    p_bottom = (x, y - DRAW_SIZE)
    p_control_stem = (x, y - DRAW_SIZE/2)
    draw_bezier(p_top, p_control_stem, p_bottom, steps=10)
    
    # --- STROKE 2: Arc/Bowl (Top to Mid-right) ---
    p_arc_start = (x, y)
    p_arc_end = (x + DRAW_SIZE * 0.1, y - DRAW_SIZE/2)
    p_arc_control = (x + DRAW_SIZE * 0.75, y + DRAW_SIZE * 0.1)
    draw_bezier(p_arc_start, p_arc_control, p_arc_end, steps=50)
    
    # --- STROKE 3: Diagonal Leg (Mid-right to Bottom-right) ---
    p_leg_start = p_arc_end
    p_leg_end = (x + DRAW_SIZE, y - DRAW_SIZE)
    
    # FIX: Move control point further right and slightly down
    p_leg_control = (x + DRAW_SIZE * 1.1, y - DRAW_SIZE * 0.6)
    
    draw_bezier(p_leg_start, p_leg_control, p_leg_end, steps=15)



def A(x, y):
    # A: Three strokes - Left Leg, Right Leg, Crossbar
    
    # Define Points (A is typically wider than DRAW_SIZE high)
    A_WIDTH = DRAW_SIZE * 0.8
    p_top = (x + A_WIDTH/2, y)
    p_left_bottom = (x, y - DRAW_SIZE)
    p_right_bottom = (x + A_WIDTH, y - DRAW_SIZE)
    p_mid_y = y - DRAW_SIZE * 0.4
    
    # 1. Left Leg (Top to Bottom-Left)
    p_control_left = (x + A_WIDTH/4, y - DRAW_SIZE/2)
    draw_bezier(p_top, p_control_left, p_left_bottom, steps=20) 
    
    # 2. Right Leg (Bottom-Left to Top, then down to Bottom-Right)
    # The right leg often starts from the top and goes down, but drawing from L-bottom to R-bottom is awkward.
    # We will draw a second stroke from the top down to the right bottom.
    
    p_control_right = (x + A_WIDTH * 0.75, y - DRAW_SIZE/2)
    draw_bezier(p_top, p_control_right, p_right_bottom, steps=20) 
    
    # 3. Crossbar (Left-Mid to Right-Mid)
    p_cross_start = (x + A_WIDTH * 0.1, p_mid_y)
    p_cross_end = (x + A_WIDTH * 0.9, p_mid_y)
    p_cross_control = (x + A_WIDTH/2, p_mid_y + DRAW_SIZE/10) # Slight curve up
    draw_bezier(p_cross_start, p_cross_control, p_cross_end, steps=10)


def M(x, y):
    # M: Four strokes - First Stem, Down-stroke, Up-stroke, Final Stem
    
    # 1. First Stem (Bottom to Top)
    p_s1_start = (x, y - DRAW_SIZE)
    p_s1_end = (x, y)
    p_s1_control = (x, y - DRAW_SIZE/2) 
    draw_bezier(p_s1_start, p_s1_control, p_s1_end, steps=15)

    # 2. Down-stroke to V-Dip (Top-L to Bottom-Center)
    p_v_dip = (x + DRAW_SIZE/2, y - DRAW_SIZE)
    p_v_control = (x + DRAW_SIZE/4, y - DRAW_SIZE * 0.5) 
    draw_bezier(p_s1_end, p_v_control, p_v_dip, steps=20) 
    
    # 3. Up-stroke to Top-Right 
    p_s2_end = (x + DRAW_SIZE, y)
    p_s2_control = (x + DRAW_SIZE * 0.75, y - DRAW_SIZE * 0.5) 
    draw_bezier(p_v_dip, p_s2_control, p_s2_end, steps=20) 

    # 4. Final Stem (Top-Right to Bottom-Right)
    p_final_stem_start = p_s2_end
    p_final_stem_end = (x + DRAW_SIZE, y - DRAW_SIZE)
    p_final_stem_control = (x + DRAW_SIZE, y - DRAW_SIZE/2) 
    draw_bezier(p_final_stem_start, p_final_stem_control, p_final_stem_end, steps=15)


# =========================================================
# HEART SHAPE (Unchanged)
# =========================================================

def draw_heart(x_center, y_center, size=0.04):
    """Draws a heart using two Quadratic Bezier curves, ensuring closure."""

    x = x_center
    y = y_center
    s = size

    # Define key points as tuples
    p_tip = (x, y - s * 1.5) 
    p_mid = (x, y + s * 0.5) 
    
    # Control points
    p_c1 = (x - s * 1.8, y + s * 1.5) 
    p_c2 = (x + s * 1.8, y + s * 1.5) 
    
    # --- Stroke 1: Left Arc (Tip up to Midpoint) ---
    draw_bezier(p_tip, p_c1, p_mid, steps=40) 

    # --- Stroke 2: Right Arc (Midpoint down to Tip) ---
    draw_bezier(p_mid, p_c2, p_tip, steps=40)

# =========================================================
# TEXT & MAIN EXECUTION
# =========================================================

def draw_text_and_heart(text, sx, sy, heart_size=0.035):
    x = sx
    
    # 1. Draw the word
    for ch in text:
        if ch == 'D':
            D(x, sy)
        elif ch == 'R':
            R(x, sy)
        elif ch == 'A':
            A(x, sy)
        elif ch == 'M':
            M(x, sy)
        elif ch == ' ':
            x += LETTER_SPACING * 1.5 # Extra space for word break
            continue
            
        x += LETTER_SPACING

    # 2. Draw the heart after the word (removed since the prompt only requested "DR AMMAR")
    # heart_center_x = x + LETTER_SPACING * 1.5 
    # heart_center_y = sy - DRAW_SIZE / 2 
    # draw_heart(heart_center_x, heart_center_y, size=heart_size)


def main():
    global pub_x, pub_y, pub_z, pub_r, marker_pub, rate

    rospy.init_node("bezier_dr_ammar")
    rate = rospy.Rate(DRAW_RATE)

    # Setup Publishers
    pub_x = rospy.Publisher("/joint_x_position_controller/command", Float64, queue_size=10)
    pub_y = rospy.Publisher("/joint_y_position_controller/command", Float64, queue_size=10)
    pub_z = rospy.Publisher("/joint_z_position_controller/command", Float64, queue_size=10)
    pub_r = rospy.Publisher("/joint_r_position_controller/command", Float64, queue_size=10)
    marker_pub = rospy.Publisher("/draw_path_marker", Marker, queue_size=10)

    rospy.loginfo("Waiting for publishers to connect...")
    rospy.sleep(2) 

    init_marker_system()
    pen_up()
    
    rospy.loginfo("Starting Bezier Handwriting of 'DR AMMAR'...")
    
    # *** MAIN EXECUTION CHANGE: "DR AMMAR" ***
    # Note: I am passing the heart_size argument even though the heart drawing is commented out, 
    # as the function signature requires it.
    draw_text_and_heart("DR AMMAR", 0.0, 0.0, heart_size=0.035) 

    rospy.loginfo("Drawing complete. Node spinning...")
    
    rospy.spin() 


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user.")
        sys.exit(0)