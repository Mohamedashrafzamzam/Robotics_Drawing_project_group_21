#!/usr/bin/env python

import rospy
import time
import re
import sys
import math
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# =========================================================
# CONFIG & CONSTANTS
# =========================================================

GCODE_FILE_PATH = "drawing.gcode"  # <--- PLACE YOUR G-CODE FILE HERE
MOVE_RATE = 50                     # ROS loop rate for publishing
PEN_SPEED_FACTOR = 0.005           # Scale factor for calculating step time (tune this!)

# Robot Coordinates (in METERS)
DRAWING_Z = 0.00                   # Robot Z position for Pen DOWN
LIFT_Z = 0.05                      # Robot Z position for Pen UP (safe height)

# G-Code Coordinates (in MILLIMETERS, as specified by G21)
# We use these to interpret the Z position from the file:
GCODE_DRAWING_Z_MM = 0.0           # G-code Z value considered "on the surface"
GCODE_LIFT_Z_MM = 5.0              # G-code Z value considered "safe height"

X_OFFSET = 0.10                    # X offset for robot base to drawing surface
Y_OFFSET = 0.00                    # Y offset for robot base to drawing surface

FRAME_ID = "world"

# =========================================================
# GLOBALS & MARKER COUNTER
# =========================================================

pub_x = pub_y = pub_z = pub_r = None
marker_pub = None
rate = None

# Current robot position (in drawing coordinates - METERS)
current_x = 0.0
current_y = 0.0
current_z = LIFT_Z
current_pen_is_down = False
last_z_was_down = False # For Rviz stroke separation

# Marker state for Rviz visualization
marker_id_counter = 0
current_marker = None

# =========================================================
# RVIZ MARKER (Handles drawing path visualization)
# =========================================================

def start_new_stroke():
    """Initializes a new Marker object with a unique ID for a single stroke."""
    global marker_id_counter, current_marker
    
    marker_id_counter += 1
    
    current_marker = Marker()
    current_marker.header.frame_id = FRAME_ID
    current_marker.ns = "gcode_drawing"
    current_marker.id = marker_id_counter 
    current_marker.type = Marker.LINE_STRIP 
    current_marker.action = Marker.ADD
    current_marker.scale.x = 0.003
    current_marker.color.r = 0.0
    current_marker.color.g = 0.0
    current_marker.color.b = 1.0 # Blue color for drawing
    current_marker.color.a = 1.0
    current_marker.pose.orientation.w = 1.0

def add_point_to_marker(x_draw, y_draw):
    """Adds a point to the current marker and publishes."""
    if not current_pen_is_down or current_marker is None:
        return
    
    # Convert drawing coordinates (x_draw, y_draw) to Rviz/Gazebo world coordinates
    p = Point()
    p.x = x_draw + X_OFFSET 
    p.y = y_draw + Y_OFFSET
    p.z = DRAWING_Z
    
    current_marker.points.append(p)
    
    current_marker.header.stamp = rospy.Time.now()
    marker_pub.publish(current_marker)

# =========================================================
# MOTION CONTROL
# =========================================================

def set_z_position(z_target_meters):
    """Publishes the Z position and updates state, handling Rviz stroke start."""
    global current_z, current_pen_is_down, last_z_was_down
    
    pub_z.publish(z_target_meters)
    current_z = z_target_meters
    
    # Check if the target is close to the drawing plane (0.00m)
    is_down_now = abs(z_target_meters - DRAWING_Z) < 0.001
    
    if is_down_now and not last_z_was_down:
        # Pen just went down: Start a new stroke in Rviz
        current_pen_is_down = True
        start_new_stroke() 
        add_point_to_marker(current_x, current_y)
    elif not is_down_now:
        # Pen is up
        current_pen_is_down = False
        
    last_z_was_down = is_down_now
    
    # Wait for the Z-axis to settle
    time.sleep(0.5) 
    
def move_linear(x_target_m, y_target_m, feed_rate):
    """Interpolates and publishes path points from current position to target."""
    global current_x, current_y
    
    dx = x_target_m - current_x
    dy = y_target_m - current_y
    distance = math.sqrt(dx**2 + dy**2)
    
    # Calculate time needed for motion (using the Feed Rate and a tuning factor)
    if feed_rate > 0 and distance > 0:
        segment_time = distance / PEN_SPEED_FACTOR # Simple, tuned speed control
    else:
        segment_time = 0.01 
        
    num_steps = max(int(segment_time * MOVE_RATE), 5) 
    
    # Linear interpolation loop
    for i in range(1, num_steps + 1):
        t = float(i) / num_steps
        
        # Calculate intermediate position (in METERS)
        x_interp = current_x + dx * t
        y_interp = current_y + dy * t
        
        # Publish to joints (add the offsets)
        pub_x.publish(x_interp + X_OFFSET)
        pub_y.publish(y_interp + Y_OFFSET)
        pub_r.publish(0.0) 

        # Add point to Rviz marker if drawing
        if current_pen_is_down:
            add_point_to_marker(x_interp, y_interp)

        rate.sleep() 

    # Update final current position
    current_x = x_target_m
    current_y = y_target_m

# =========================================================
# G-CODE PARSING
# =========================================================

def parse_gcode_file(filename):
    """Reads and parses a simplified 2D G-code file."""
    rospy.loginfo(f"Attempting to read G-code from: {filename}")
    
    gcode_commands = []
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                # Clean up line: remove comments and leading/trailing whitespace
                line = line.split(';')[0].strip()
                if not line:
                    continue
                
                # Use regex to find commands and parameters (X, Y, Z, F, G, P, M)
                command = {}
                # Match major codes (G, M) and parameters (X, Y, Z, F, P)
                parts = re.findall(r'([A-Z])([+\-]?[\d\.]+)', line)
                
                for key, value in parts:
                    try:
                        command[key] = float(value)
                    except ValueError:
                        pass
                
                if 'G' in command or 'M' in command:
                    gcode_commands.append(command)
                    
    except FileNotFoundError:
        rospy.logerr(f"G-code file not found at: {filename}")
        sys.exit(1)
        
    rospy.loginfo(f"Successfully loaded {len(gcode_commands)} G-code commands.")
    return gcode_commands

def execute_gcode(commands):
    """Executes the parsed G-code commands with units conversion and G4 support."""
    global current_x, current_y
    
    current_feed_rate = 3000.0
    
    for cmd in commands:
        g_code = int(cmd.get('G', -1))
        
        if rospy.is_shutdown():
            break

        # --- G4: Dwell / Pause ---
        if g_code == 4:
            dwell_time_ms = cmd.get('P', 0)
            if dwell_time_ms > 0:
                rospy.loginfo(f"G4: Dwell for {dwell_time_ms}ms")
                time.sleep(dwell_time_ms / 1000.0) # Convert P (ms) to seconds
            continue
            
        # --- M-Codes ---
        m_code = int(cmd.get('M', -1))
        if m_code == 2 or m_code == 30:
             rospy.loginfo("M2/M30: Program End Detected. Stopping execution.")
             set_z_position(LIFT_Z)
             break
            
        # --- F: Feed Rate Update ---
        if 'F' in cmd:
            current_feed_rate = cmd['F']
            
        # --- G00/G01: Motion Commands ---
        if g_code == 0 or g_code == 1:
            
            # 1. Units Conversion (from MM to Meters)
            # Default target is the current position (in meters, converted back to mm)
            target_x_mm = cmd.get('X', current_x * 1000.0)
            target_y_mm = cmd.get('Y', current_y * 1000.0)
            target_z_mm = cmd.get('Z', current_z * 1000.0) 

            # Convert targets to meters
            target_x_m = target_x_mm / 1000.0
            target_y_m = target_y_mm / 1000.0
            # target_z_m is determined by the logic below, not a direct conversion.
            
            # 2. Handle Z moves (Pen Up/Down)
            if 'Z' in cmd:
                # If the G-code Z is at or below the drawing plane (e.g., Z=0 or Z=-1.00)
                if target_z_mm <= GCODE_DRAWING_Z_MM:
                    set_z_position(DRAWING_Z) # Robot Z=0.00m (Down)
                else:
                    # If Z is above the drawing plane (e.g., Z=5.00)
                    set_z_position(LIFT_Z) # Robot Z=0.05m (Up)
            
            # 3. Execute the X-Y movement
            move_linear(target_x_m, target_y_m, current_feed_rate)
            
        # --- G28: Homing ---
        elif g_code == 28:
            rospy.loginfo("Executing G28: Homing...")
            set_z_position(LIFT_Z) 
            move_linear(0.0, 0.0, 5000) 
            current_x = 0.0
            current_y = 0.0
            
        else:
            rospy.logwarn(f"Unsupported G-code or parameter ignored: {cmd}")


# =========================================================
# MAIN EXECUTION
# =========================================================

def main():
    global pub_x, pub_y, pub_z, pub_r, marker_pub, rate

    rospy.init_node("gcode_drawing_interpreter")
    rate = rospy.Rate(MOVE_RATE)

    # Setup Publishers
    pub_x = rospy.Publisher("/joint_x_position_controller/command", Float64, queue_size=1)
    pub_y = rospy.Publisher("/joint_y_position_controller/command", Float64, queue_size=1)
    pub_z = rospy.Publisher("/joint_z_position_controller/command", Float64, queue_size=1)
    pub_r = rospy.Publisher("/joint_r_position_controller/command", Float64, queue_size=1)
    marker_pub = rospy.Publisher("/draw_path_marker", Marker, queue_size=1)

    rospy.loginfo("Waiting for publishers to connect...")
    rospy.sleep(2) 
    
    # 1. Start with pen up and robot at origin
    set_z_position(LIFT_Z)
    pub_r.publish(0.0)
    pub_x.publish(current_x + X_OFFSET)
    pub_y.publish(current_y + Y_OFFSET)
    time.sleep(1.0) # Settle at start

    # 2. Parse and Execute G-code
    gcode_commands = parse_gcode_file(GCODE_FILE_PATH)
    
    if gcode_commands:
        rospy.loginfo("Starting G-code execution...")
        execute_gcode(gcode_commands)
        rospy.loginfo("G-code execution complete.")

    # 3. Finish
    set_z_position(LIFT_Z) # Ensure pen is up at the end
    rospy.loginfo("Drawing process finished. Node spinning...")
    
    rospy.spin() 


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user.")
        sys.exit(0)