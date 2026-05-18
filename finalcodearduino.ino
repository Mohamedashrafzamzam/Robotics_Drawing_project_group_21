#include <AccelStepper.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// --- PIN DEFINITIONS ---
#define X_STEP 2
#define X_DIR  5
#define Y_STEP 3
#define Y_DIR  6
#define Z_STEP 4
#define Z_DIR  7
#define A_STEP 12
#define A_DIR  13 
#define ENABLE_PIN 8
#define ESTOP_PIN A0 

// --- CALIBRATION ---
float STEPS_X = 100.0;
float STEPS_Y = 160.0;
float STEPS_Z = 80.0;
float STEPS_A = 71.11;

// --- MOTOR OBJECTS ---
AccelStepper rail(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper arm(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper lift(AccelStepper::DRIVER, Z_STEP, Z_DIR);
AccelStepper rot(AccelStepper::DRIVER, A_STEP, A_DIR);

// --- LCD SETUP ---
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- STATE VARIABLES ---
String inputString = "";
bool stringComplete = false;
bool isStopped = false; 
bool isMovingState = false;

void setup() {
  Serial.begin(115200);
  
  // 1. LCD INIT
  lcd.init();
  lcd.backlight();
  Wire.setWireTimeout(3000, true); 
  
  lcd.setCursor(0,0); lcd.print("ROBOT ONLINE");
  
  // 2. PINS
  pinMode(ESTOP_PIN, INPUT_PULLUP); 
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); 

  // 3. MOTORS
  rail.setPinsInverted(true, false, false); 
  arm.setPinsInverted(false, false, false);
  lift.setPinsInverted(true, false, false); 
  rot.setPinsInverted(false, false, false);

  rail.setMaxSpeed(4000);  rail.setAcceleration(8000);
  arm.setMaxSpeed(4000);   arm.setAcceleration(8000);
  lift.setMaxSpeed(2000);  lift.setAcceleration(4000);
  rot.setMaxSpeed(2000);   rot.setAcceleration(2000);
  
  inputString.reserve(200);
  Serial.println("READY");
  updateLcdIdle(); 
}

void loop() {
  int buttonState = digitalRead(ESTOP_PIN);

  // 1. SAFETY: TRIGGER E-STOP
  if (buttonState == HIGH && !isStopped) {
      triggerEmergencyStop();
  }
  // 2. SAFETY: RESET (BUTTON RELEASED)
  else if (buttonState == LOW && isStopped) {
    isStopped = false;
    
    
    rail.setCurrentPosition(rail.currentPosition());
    arm.setCurrentPosition(arm.currentPosition());
    lift.setCurrentPosition(lift.currentPosition());
    rot.setCurrentPosition(rot.currentPosition());

    rail.moveTo(rail.currentPosition());
    arm.moveTo(arm.currentPosition());
    lift.moveTo(lift.currentPosition());
    rot.moveTo(rot.currentPosition());
    
    
    Serial.println("AUTO_RESET");
    lcd.clear(); lcd.print("SAFETY RESET");
    delay(500); 
    updateLcdIdle();
  }

  // 3. LCD MANAGER
  bool motorsBusy = (rail.distanceToGo() != 0 || arm.distanceToGo() != 0 || 
                     lift.distanceToGo() != 0 || rot.distanceToGo() != 0);

  if (motorsBusy && !isMovingState) {
    if (!isStopped) { 
        lcd.clear();
        lcd.setCursor(0,0); lcd.print(">> MOVING... >>"); 
        isMovingState = true;
    }
  }
  else if (!motorsBusy && isMovingState) {
    updateLcdIdle(); 
    isMovingState = false;
  }

  // 4. RUN MOTORS (Only if NOT stopped)
  if (!isStopped) {
    rail.run(); arm.run(); lift.run(); rot.run();
  }

  // 5. PARSE COMMANDS
  if (stringComplete) {
    if (!isStopped) parseAndUpdate(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void updateLcdIdle() {
  if (isStopped) return; 

  float x_pos = rail.currentPosition() / STEPS_X;
  float y_pos = arm.currentPosition() / STEPS_Y;
  float z_pos = lift.currentPosition() / STEPS_Z;

  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("X:"); lcd.print(x_pos, 1); 
  lcd.print(" Y:"); lcd.print(y_pos, 1);
  
  lcd.setCursor(0,1);
  lcd.print("Z:"); lcd.print(z_pos, 1);
  lcd.print("  IDLE");
}

void triggerEmergencyStop() {
  // CRITICAL: Do NOT use .stop() here. .stop() calculates a deceleration ramp.
  // We want to freeze coordinates instantly.
  rail.moveTo(rail.currentPosition());
  arm.moveTo(arm.currentPosition());
  lift.moveTo(lift.currentPosition());
  rot.moveTo(rot.currentPosition());
  
  Serial.println("STOPPED");
  isStopped = true;
  
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("!! E-STOPPED !!");
  lcd.setCursor(0,1); lcd.print("RELEASE BUTTON");
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '!') {
      triggerEmergencyStop();
      inputString = "";
      stringComplete = false;
    } 
    else if (inChar == '\n') stringComplete = true;
    else inputString += inChar;
  }
}

void parseAndUpdate(String cmd) {
  float tX = rail.targetPosition() / STEPS_X;
  float tY = arm.targetPosition() / STEPS_Y;
  float tZ = lift.targetPosition() / STEPS_Z;
  float tA = rot.targetPosition() / STEPS_A;

  int idx = cmd.indexOf('X'); if(idx!=-1) tX = cmd.substring(idx+1).toFloat();
  idx = cmd.indexOf('Y'); if(idx!=-1) tY = cmd.substring(idx+1).toFloat();
  idx = cmd.indexOf('Z'); if(idx!=-1) tZ = cmd.substring(idx+1).toFloat();
  idx = cmd.indexOf('A'); if(idx!=-1) tA = cmd.substring(idx+1).toFloat();

  rail.moveTo(tX * STEPS_X);
  arm.moveTo(tY * STEPS_Y);
  lift.moveTo(tZ * STEPS_Z);
  rot.moveTo(tA * STEPS_A);

  if (cmd.indexOf('W') != -1) {
    if (!isStopped) {
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("DRAWING POINT...");
    }
    
    while (rail.distanceToGo() != 0 || arm.distanceToGo() != 0 || 
           lift.distanceToGo() != 0 || rot.distanceToGo() != 0) {
      rail.run(); arm.run(); lift.run(); rot.run();
      
      // If E-STOP is pressed DURING a W command loop
      if (digitalRead(ESTOP_PIN) == HIGH) { 
          triggerEmergencyStop(); 
          return; 
      }
    }
    updateLcdIdle();
  }

  Serial.println("DONE");
}