import serial
import time
import sys
import re
import msvcrt 
import math

# --- CONFIGURATION ---
ARDUINO_PORT = 'COM8' 
BAUD_RATE    = 115200

# --- ROBOT LIMITS (CM) ---
X_RAIL_MAX = 24.0  
Y_ARM_MAX  = 33.8   
Z_MAX      = 27.3   

# --- PAPER SETUP ---
Y_OFFSET   = 13.0   

# --- PEN HEIGHTS ---
# 0 = Base/Ground.
PEN_UP_Z   = 18.8  # Travel Height
PEN_DOWN_Z = 17   # Drawing Height (Touching Paper)

# --- INVERSE KINEMATICS CONFIG ---
ARM_MIN_Y  = 3.8 
Y_SHIFT    = ARM_MIN_Y 

class GCodeRobot:
    def __init__(self):
        self.ser = None
        self.connect()
        
        self.curr_x = 0
        self.curr_y = 0 
        self.curr_z = 0
        self.curr_a = 0       
        self.last_raw_x = 0
        self.last_raw_y = 0

        self.startup_sequence()

    def connect(self):
        try:
            print(f"Connecting to {ARDUINO_PORT}...")
            self.ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) 
            self.ser.reset_input_buffer()
            print("Connected.")
        except serial.SerialException:
            print("ERROR: Could not open port.")
            sys.exit()

    def calculate_ik(self, target_x, user_y):
        real_user_y = user_y + Y_OFFSET
        base_x = max(0, min(target_x, X_RAIL_MAX))
        dx = target_x - base_x
        dy = real_user_y 
        angle_rad = math.atan2(dy, dx) 
        angle_deg = math.degrees(angle_rad)
        if angle_deg > 180: angle_deg -= 360
        if angle_deg <= -180: angle_deg += 360
        dist_needed = math.sqrt(dx**2 + real_user_y**2) + Y_SHIFT
        motor_y = dist_needed - ARM_MIN_Y
        return base_x, motor_y, angle_deg

    def send_command(self, x, y, z, a, wait=False):
        wait_flag = " W1" if wait else ""
        # Create the command string once so we can reuse it if we need to resend
        cmd = f"X{x*10:.2f} Y{y*10:.2f} Z{z*10:.2f} A{a:.2f}{wait_flag}\n"
        self.ser.write(cmd.encode('utf-8'))
        
        while True:
            # --- 1. ESC KEY = QUIT & HOME ---
            if msvcrt.kbhit():
                if msvcrt.getch() == b'\x1b': # ESC key
                    print("\n[!] ESC Detected. Quitting...")
                    return "STOP" # This triggers shutdown_sequence() in the main loop

            # --- 2. ARDUINO RESPONSE CHECK ---
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Normal completion
                if line == "DONE": 
                    break 
                
                # --- 3. PHYSICAL E-STOP = AUTO RESUME ---
                if line == "STOPPED":
                    print("\n[!!!] PHYSICAL HOLD DETECTED [!!!]")
                    print("    Holding position... Release button to resume.")
                    
                    # Wait here until the button is released
                    while True:
                        if self.ser.in_waiting:
                            msg = self.ser.readline().decode('utf-8').strip()
                            if msg == "AUTO_RESET":
                                print("    -> Button Released. Resuming move...")
                                # CRITICAL: We must RE-SEND the command because the 
                                # Arduino forgot it when it stopped.
                                self.ser.write(cmd.encode('utf-8'))
                                break # Break the inner wait loop, go back to waiting for "DONE"

        return "OK"

    def move_to_coordinate(self, target_x, target_y):
        result = self.calculate_ik(target_x, target_y)
        if not result: return "OK"
        motor_x, motor_y, motor_a = result
        self.curr_x, self.curr_y, self.curr_a = motor_x, target_y, motor_a
        return self.send_command(motor_x, motor_y, self.curr_z, motor_a, wait=True)

    def startup_sequence(self):
        print("\n--- STARTUP SEQUENCE ---")
        print("1. Lifting Pen...")
        self.curr_z = PEN_UP_Z
        self.send_command(self.curr_x, 0, self.curr_z, self.curr_a, wait=True)
        print("2. Centering...")
        self.send_command(0, 0, self.curr_z, 90, wait=True)
        self.curr_x = 0
        self.curr_a = 90
        print(">>> ROBOT READY <<<")

    def scan_file_and_get_scale(self, filename):
        print("Scanning file...")
        max_x = 0; max_y = 0
        try:
            with open(filename, 'r') as f: lines = f.readlines()
        except: return 0.25 
        for line in lines:
            x_match = re.search(r'X\s*([0-9\.-]+)', line, re.IGNORECASE)
            y_match = re.search(r'Y\s*([0-9\.-]+)', line, re.IGNORECASE)
            if x_match: max_x = max(max_x, float(x_match.group(1)))
            if y_match: max_y = max(max_y, float(y_match.group(1)))
        max_x_cm = max_x / 10.0; max_y_cm = max_y / 10.0
        print(f"   Size: {max_x_cm:.1f}cm x {max_y_cm:.1f}cm")
        scale_x = (X_RAIL_MAX - 1.0) / max(max_x_cm, 1)
        scale_y = (Y_ARM_MAX - Y_OFFSET - 1.0) / max(max_y_cm, 1)
        final_scale = min(scale_x, scale_y)
        if final_scale > 1.0: final_scale = 1.0 
        print(f"   Scale: {final_scale:.2f}")
        return final_scale

    # --- CUSTOM SHUTDOWN (Retract Y First) ---
    def shutdown_sequence(self):
        print("\n--- FINISHED ---")
        
        # 1. Lift Safe
        print("   Lifting...")
        self.curr_z = PEN_UP_Z
        self.send_command(self.curr_x, 0, PEN_UP_Z, self.curr_a, wait=True)
        
        # 2. Retract Arm (Y) ONLY
        print("   Retracting Arm...")
        # Calculate IK for current X, but Y=0
        mx, my, ma = self.calculate_ik(self.curr_x, 0)
        self.send_command(mx, my, PEN_UP_Z, ma, wait=True)
        
        # 3. Go Full Home
        print("   Homing...")
        self.send_command(0, 0, PEN_UP_Z, 0, wait=True)
        
        # 4. Park Z
        print("   Parking Z...")
        self.send_command(0, 0, 0, 0, wait=True)
        print("Done.")

    def run_gcode(self, filename):
        SCALE = self.scan_file_and_get_scale(filename)
        print(f"Drawing... (Press ESC to Stop)")
        
        # Safe Start
        self.curr_z = PEN_UP_Z
        if self.send_command(0, 0, PEN_UP_Z, 90, wait=True) == "STOP": 
            self.shutdown_sequence(); return

        is_first_point_reached = False 
        
        try:
            with open(filename, 'r') as f: lines = f.readlines()
        except: return

        for line in lines:
            line = line.strip().upper()
            if not line or line.startswith(';') or line.startswith('(') or line.startswith('%'): continue

            x_match = re.search(r'X\s*([0-9\.-]+)', line)
            y_match = re.search(r'Y\s*([0-9\.-]+)', line)
            z_match = re.search(r'Z\s*([0-9\.-]+)', line)

            # --- 1. HANDLE FIRST MOVE (Ignore Z before this) ---
            if not is_first_point_reached:
                if x_match or y_match:
                    if x_match: self.last_raw_x = float(x_match.group(1))
                    if y_match: self.last_raw_y = float(y_match.group(1))
                    
                    target_x = (self.last_raw_x * SCALE) / 10.0
                    target_y = (self.last_raw_y * SCALE) / 10.0
                    
                    print(f"   [Start] Traveling to First Point...")
                    # Force Pen Up
                    self.curr_z = PEN_UP_Z
                    mx, my, ma = self.calculate_ik(target_x, target_y)
                    
                    # Move to X/Y with Pen Up
                    if self.send_command(mx, my, PEN_UP_Z, ma, wait=True) == "STOP": 
                        self.shutdown_sequence(); return
                    
                    self.curr_x, self.curr_y, self.curr_a = mx, target_y, ma
                    is_first_point_reached = True
                
                # SKIP everything else (including Z commands) until we reach first X/Y
                continue 

            # --- 2. HANDLE Z COMMANDS (Only after arrival) ---
            if z_match:
                gcode_z = float(z_match.group(1))
                
                # Logic: Map G-code Z to Robot Z
                if gcode_z <= 0.5: 
                    target_robot_z = PEN_DOWN_Z # e.g. 13.5
                    state = "DOWN"
                else: 
                    target_robot_z = PEN_UP_Z   # e.g. 18.0
                    state = "UP"

                if self.curr_z != target_robot_z:
                    print(f"   [Z-Cmd] Pen {state} ({target_robot_z}cm)")
                    self.curr_z = target_robot_z
                    mx, my, ma = self.calculate_ik(self.curr_x, self.curr_y)
                    if self.send_command(mx, my, self.curr_z, ma, wait=True) == "STOP":
                        self.shutdown_sequence(); return

            # --- 3. HANDLE X/Y MOVES ---
            if x_match or y_match:
                if x_match: self.last_raw_x = float(x_match.group(1))
                if y_match: self.last_raw_y = float(y_match.group(1))

                target_x = (self.last_raw_x * SCALE) / 10.0
                target_y = (self.last_raw_y * SCALE) / 10.0

                if self.move_to_coordinate(target_x, target_y) == "STOP": 
                    self.shutdown_sequence(); return

        print("Drawing Complete.")
        self.shutdown_sequence()

if __name__ == "__main__":
    bot = GCodeRobot()
    fname = input("Enter filename: ")
    bot.run_gcode(fname)