#!/usr/bin/env python

import rospy
import time
import sys
import os
import rospkg

from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# =========================================================
# ------------------- CONSTANTS ----------------------------
# =========================================================

HOME_X = 0.05
HOME_Y = 0.00
HOME_R = 0.0

DRAWING_Z = 0.001     # Pen touching surface (simulation safe)
LIFT_Z = 0.06         # Pen lifted

MOVE_PAUSE = 0.5
DRAW_PAUSE = 0.2

DRAWING_PLANE_OFFSET_Y = -0.15

# =========================================================
# ---------------- GLOBAL PUBLISHERS -----------------------
# =========================================================

pub_x = None
pub_y = None
pub_z = None
pub_r = None
marker_pub = None

# =========================================================
# ---------------- HELPER FUNCTIONS ------------------------
# =========================================================

def publish_point(x, y, z, r=HOME_R):
    pub_x.publish(x)
    pub_y.publish(y)
    pub_z.publish(z)
    pub_r.publish(r)

def wait(duration):
    if duration > 0.0 and not rospy.is_shutdown():
        time.sleep(duration)

def go_home():
    publish_point(HOME_X, HOME_Y, LIFT_Z)
    wait(MOVE_PAUSE)

# =========================================================
# --------------- PATH GENERATION --------------------------
# =========================================================

def process_and_plan_image(image_path):
    rospy.loginfo(f"SIMULATED: Processing image at {image_path}")

    path = [
        # ---- Square ----
        (0.10, DRAWING_PLANE_OFFSET_Y, LIFT_Z, MOVE_PAUSE),
        (0.10, DRAWING_PLANE_OFFSET_Y, DRAWING_Z, MOVE_PAUSE),

        (0.10, DRAWING_PLANE_OFFSET_Y + 0.05, DRAWING_Z, DRAW_PAUSE),
        (0.15, DRAWING_PLANE_OFFSET_Y + 0.05, DRAWING_Z, DRAW_PAUSE),
        (0.15, DRAWING_PLANE_OFFSET_Y, DRAWING_Z, DRAW_PAUSE),
        (0.10, DRAWING_PLANE_OFFSET_Y, DRAWING_Z, DRAW_PAUSE),

        (0.10, DRAWING_PLANE_OFFSET_Y, LIFT_Z, MOVE_PAUSE),

        # ---- Dot ----
        (0.15, DRAWING_PLANE_OFFSET_Y + 0.08, LIFT_Z, MOVE_PAUSE),
        (0.15, DRAWING_PLANE_OFFSET_Y + 0.08, DRAWING_Z, MOVE_PAUSE),
        (0.15, DRAWING_PLANE_OFFSET_Y + 0.08, DRAWING_Z, DRAW_PAUSE),
        (0.15, DRAWING_PLANE_OFFSET_Y + 0.08, LIFT_Z, MOVE_PAUSE),
    ]

    return path

# =========================================================
# ---------------- RVIZ VISUALIZATION ----------------------
# =========================================================

def visualize_path(planned_path):
    rospy.loginfo("Publishing drawing preview to RViz...")

    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()

    marker.ns = "drawing_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.scale.x = 0.002

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose.orientation.w = 1.0

    for x, y, z, _ in planned_path:
        if z <= DRAWING_Z:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

    marker_pub.publish(marker)
    rospy.sleep(1.0)

# =========================================================
# ---------------- PATH EXECUTION --------------------------
# =========================================================

def execute_planned_path(planned_path):
    rospy.loginfo("Executing planned path...")

    rospy.loginfo("----- PLANNED WAYPOINTS -----")
    for i, (x, y, z, p) in enumerate(planned_path):
        rospy.loginfo(
            f"WP {i:02}: X={x:.3f}, Y={y:.3f}, "
            f"Z={'LIFT' if z > DRAWING_Z else 'DRAW'}, Pause={p:.2f}"
        )
    rospy.loginfo("-----------------------------")

    for x, y, z, pause in planned_path:
        if rospy.is_shutdown():
            break
        publish_point(x, y, z)
        wait(pause)

    rospy.loginfo("Path execution complete.")

# =========================================================
# -------------------- MAIN LOGIC --------------------------
# =========================================================

def draw_from_photo_sequence():
    global pub_x, pub_y, pub_z, pub_r, marker_pub

    rospy.init_node('draw_from_photo_node', anonymous=True)

    pub_x = rospy.Publisher('/joint_x_position_controller/command', Float64, queue_size=10)
    pub_y = rospy.Publisher('/joint_y_position_controller/command', Float64, queue_size=10)
    pub_z = rospy.Publisher('/joint_z_position_controller/command', Float64, queue_size=10)
    pub_r = rospy.Publisher('/joint_r_position_controller/command', Float64, queue_size=10)

    marker_pub = rospy.Publisher('/draw_path_marker', Marker, queue_size=1)

    rospy.loginfo("Initializing Robot...")
    go_home()
    wait(2.0)

    try:
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('URDF_Assembly')
        image_file = os.path.join(pkg_path, 'images', 'images.jpg')
    except rospkg.ResourceNotFound:
        rospy.logerr("Package 'URDF_Assembly' not found!")
        sys.exit(1)

    planned_path = process_and_plan_image(image_file)

    visualize_path(planned_path)

    rospy.loginfo("Check RViz. Executing in 3 seconds...")
    rospy.sleep(3.0)

    execute_planned_path(planned_path)

    go_home()
    rospy.loginfo("Drawing session ended.")

# =========================================================
# -------------------- ENTRY POINT -------------------------
# =========================================================

if __name__ == '__main__':
    try:
        draw_from_photo_sequence()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Program interrupted by user.")
        sys.exit(0)
