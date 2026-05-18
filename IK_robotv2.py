import serial
import time
import math
import sys
import msvcrt

# --- CONFIGURATION ---
arduino_port = 'COM8'   
baud_rate = 115200

# Physical Limits (in cm)
RAIL_MAX_X = 26 
ARM_MIN_Y  = 3.8    # Physical minimum length
ARM_MAX_Y  = 33.8   # Physical max reach
Y_SHIFT    = ARM_MIN_Y 

# --- INVERSE KINEMATICS ---
def calculate_smart_move(target_x, user_y):
    real_target_y = abs(user_y) + Y_SHIFT 
    base_x = max(0, min(target_x, RAIL_MAX_X))
    
    dx = target_x - base_x
    dy = user_y 
    angle_rad = math.atan2(dy, dx) 
    angle_deg = math.degrees(angle_rad)

    # Smart Rotation
    if angle_deg > 180: angle_deg -= 360
    if angle_deg <= -180: angle_deg += 360

    dist_needed = math.sqrt(dx**2 + user_y**2) + Y_SHIFT
    motor_y = dist_needed - ARM_MIN_Y

    if dist_needed < (ARM_MIN_Y - 0.1) or dist_needed > ARM_MAX_Y:
        return None

    return base_x, motor_y, angle_deg

# --- HOMING ---
def home_robot(ser):
    print("\n--- HOMING ---")
    # Send all to 0 (including Angle 0)
    cmd = "X0.00 Y0.00 Z0.00 A0.00\n"
    ser.write(cmd.encode('utf-8'))
    
    # Wait for completion (Homing MUST block to be safe)
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if line == "DONE" or line == "STOPPED":
                print("-> HOMED.")
                break
    return 0.0, 0.0, 0.0, 0.0

# --- CONTINUOUS JOG (R/T Keys) ---

def run_joint_jog(ser, cur_x, cur_ext, cur_z, cur_a):
    print("\n--- SMOOTH JOG MODE ---")
    print(" [W / S]     Extend/Retract Arm (Y)")
    print(" [A / D]     Slide Base         (X)")
    print(" [R / T]     Pen UP / DOWN      (Z)")
    print(" [Q / E]     Rotate Base        (Angle)")
    print(" [ESC]       Exit")
    
    # Steps in CM
    step_xy = 0.5   
    step_a  = 1.0   
    step_z  = 0.5   
    
    mx, my, mz, ma = cur_x, cur_ext, cur_z, cur_a

    # z limit
    Z_MAX_LIMIT = 27.3 

    while True:
        if msvcrt.kbhit():
            key = msvcrt.getch()
            
            if key == b'\x1b': return mx, my, mz, ma # ESC

            # --- MOVEMENT KEYS ---
            if key.lower() == b'a': mx -= step_xy
            if key.lower() == b'd': mx += step_xy

            if key.lower() == b'w': my += step_xy
            if key.lower() == b's': my -= step_xy

            if key.lower() == b'r': mz += step_z # UP
            if key.lower() == b't': mz -= step_z # DOWN

            if key.lower() == b'q': ma -= step_a
            if key.lower() == b'e': ma += step_a
            
           #limits
            # X Limits
            if mx < 0: mx = 0
            if mx > RAIL_MAX_X: mx = RAIL_MAX_X
            
            # Y Limits
            if my < 0: my = 0
            if my > (ARM_MAX_Y - ARM_MIN_Y): my = (ARM_MAX_Y - ARM_MIN_Y)

            # Z Limits 
            if mz < 0: mz = 0
            if mz > Z_MAX_LIMIT: mz = Z_MAX_LIMIT
            
           
           
            cmd = f"X{mx*10:.2f} Y{my*10:.2f} Z{mz*10:.2f} A{ma:.2f}\n"
            ser.write(cmd.encode('utf-8'))
            
            time.sleep(0.02) 
            
            print(f"\r Jog: X{mx:.1f}cm Y{my:.1f}cm Z{mz:.1f}cm A{ma:.1f}°   ", end="")
            
# --- MAIN SCRIPT ---
ser = None
try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)
    print("CONNECTED.")
    
    mot_x, mot_y, mot_z, mot_a = 0.0, 0.0, 0.0, 0.0

    while True:
        print(f"\n\n[State: X{mot_x:.1f} Y{mot_y:.1f} Z{mot_z:.1f} A{mot_a:.1f}]")
        choice = input("Enter X (or 'h' Home, 'm' Jog): ")

        if choice.lower() == 'q': break
        
        # HOMING (Special Sequence)
        if choice.lower() == 'h':
            mot_x, mot_y, mot_z, mot_a = home_robot(ser)
            continue

        # JOG MODE
        if choice.lower() == 'm':
            mot_x, mot_y, mot_z, mot_a = run_joint_jog(ser, mot_x, mot_y, mot_z, mot_a)
            continue

        # COORDINATE MOVE
        try:
            # 1. Get X
            t_x = float(choice)
            
            # 2. Get Y
            t_y = float(input("Target Y: "))
            
            # 3. Get Z (NOW it waits for this!)
            z_in = input(f"Target Z [{mot_z}]: ")
            if z_in != "": 
                mot_z = float(z_in)

            # 4. Calculate & Move
            result = calculate_smart_move(t_x, t_y)
            if result:
                mx, my, ma = result
                print(f"   Moving to ({t_x}, {t_y}, {mot_z})...")
                
                cmd = f"X{mx*10:.2f} Y{my*10:.2f} Z{mot_z*10:.2f} A{ma:.2f}\n"
                ser.write(cmd.encode('utf-8'))
                
                # Update State
                mot_x, mot_y, mot_a = mx, my, ma

                # Wait for move to finish
                while True:
                    if ser.in_waiting:
                        line = ser.readline().decode('utf-8').strip()
                        
                        if line == "DONE": 
                            break
                        
                        if line == "STOPPED": 
                            print("\n[!] Emergency Stop Triggered!")
                            break

                        # --- THE FIX ---
                        if line == "AUTO_RESET":
                            print("\n[!] Button Released. Aborting Move.")
                            break
        except ValueError:
            print("Invalid input.")

except Exception as e:
    print(f"Error: {e}")
finally:
    if ser: ser.close()