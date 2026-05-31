//-----------------------------------------------------------------------------------
// Optimized BluePad32 with Dual Mode: 1:1 Camp Filtering vs. Open Home Mode
// Proportional Bucket Servo Control via Analog L2 / R2 Triggers Included
//-----------------------------------------------------------------------------------

#include <Bluepad32.h>
#include <ESP32Servo.h> 

#define rightMotor0 26
#define rightMotor1 25

#define leftMotor0 32
#define leftMotor1 33

#define armMotor0 21
#define armMotor1 19

#define bucketServoPin  23
#define clawServoPin 22

#define auxLights0 18
#define auxLights1 5

// --- DUAL MODE CONFIGURATION ---
const uint8_t ASSIGNED_CONTROLLER_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

Servo bucketServo;
Servo clawServo;

int bucketServoValue = 140;
int clawServoValue = 150;
int lastBucketValue = -1;  // Track changes to prevent interrupt spam
int lastClawValue = -1;    // Track changes to prevent interrupt spam
int servoDelay = 0;

bool moveClawServoUp = false;
bool moveClawServoDown = false;

int deadzone = 12; // Slightly widened to absorb knock-off controller stick drift
ControllerPtr activeGamepad = nullptr;

void moveMotorAnalog(int pin0, int pin1, int value, int deadzone=12)
{
  if (value > deadzone)
  {
    analogWrite(pin0, value);
    analogWrite(pin1, LOW);
  }
  else if (value < -deadzone)
  {
    analogWrite(pin0, LOW);
    analogWrite(pin1, -value);
  }
  else
  {
    analogWrite(pin0, LOW);
    analogWrite(pin1, LOW);
  }
}

void toggleLights(int pin)
{
  digitalWrite(pin, !digitalRead(pin));
}

bool isMacBlank() {
    for (int i = 0; i < 6; i++) {
        if (ASSIGNED_CONTROLLER_MAC[i] != 0x00) return false;
    }
    return true;
}

void onConnectedController(ControllerPtr cptr) {
    // Extract the controller properties
    ControllerProperties properties = cptr->getProperties();
    
    // --- PRINT THE MAC ADDRESS TO THE CONSOLE ---
    Serial.printf("CONNECTED CONTROLLER MAC: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", 
        properties.btaddr[0], properties.btaddr[1], properties.btaddr[2], properties.btaddr[3], properties.btaddr[4], properties.btaddr[5]);

    // Check if we are running in Open Home Mode
    if (isMacBlank()) {
        Serial.println("Home Mode Detected (Blank MAC).");
        activeGamepad = cptr;
        cptr->setColorLED(0, 0, 255); // Solid Blue
        return;
    }

    // Enforce Camp Mode MAC Filtering
    bool isMatch = true;
    for (int i = 0; i < 6; i++) {
        if (properties.btaddr[i] != ASSIGNED_CONTROLLER_MAC[i]) {
            isMatch = false;
            break;
        }
    }

    if (isMatch) {
        Serial.println("Assigned PS4 Controller Connected (Camp Mode)!");
        activeGamepad = cptr;
        cptr->setColorLED(0, 255, 0); // Solid Green
    } else {
        Serial.println("Incorrect controller. Disconnecting...");
        cptr->disconnect(); 
    }
}

void onDisconnectedController(ControllerPtr cptr) {
    if (cptr == activeGamepad) {
        Serial.println("Controller disconnected.");
        activeGamepad = nullptr;
    }
}

void processGamepadInput(ControllerPtr gp) {
    // Read and scale sticks down to original parameters
    int LXValue = gp->axisX() / 4;  
    int LYValue = gp->axisY() / 4;  
    int RXValue = gp->axisRX() / 4;
    int RYValue = gp->axisRY() / 4;

    // --- REVERTED ORIGINAL MINI SKIDI SINGLE-AXIS DRIVING LOGIC ---
    if (abs(LXValue) > deadzone) {
        // Steering takes absolute priority when stick is pushed left/right
        moveMotorAnalog(rightMotor0, rightMotor1, 2 * LXValue);
        moveMotorAnalog(leftMotor0, leftMotor1, -2 * LXValue);
    } 
    else if (abs(LYValue) > deadzone) {
        // Drive forward/backward only when steering stick is resting
        moveMotorAnalog(rightMotor0, rightMotor1, 2 * LYValue);
        moveMotorAnalog(leftMotor0, leftMotor1, 2 * LYValue);
    } 
    else {
        // Safe stop condition inside deadzones
        moveMotorAnalog(rightMotor0, rightMotor1, 0);
        moveMotorAnalog(leftMotor0, leftMotor1, 0);
    }

    // Right Stick Control (Arm Lift)
    if (abs(RYValue) > deadzone) {
        moveMotorAnalog(armMotor0, armMotor1, 2 * RYValue);
    } else {
        moveMotorAnalog(armMotor0, armMotor1, 0);
    }

    // --- DIGITAL BUTTONS ---
    moveClawServoUp   = (gp->buttons() & BUTTON_SHOULDER_L); // L1
    moveClawServoDown = (gp->buttons() & BUTTON_SHOULDER_R); // R1

    static bool r3Pressed = false;
    if (gp->buttons() & BUTTON_THUMB_R) {
        if (!r3Pressed) {
            toggleLights(auxLights1);
            r3Pressed = true;
        }
    } else {
        r3Pressed = false;
    }

    // --- PROPORTIONAL BUCKET LOGIC (Analog Triggers L2 / R2) ---
    // BluePad32 trigger ranges run from 0 to 1023
    int L2Analog = gp->throttle(); // Left Trigger (L2)
    int R2Analog = gp->brake();    // Right Trigger (R2)

    if (R2Analog > 50) { // Right trigger moves bucket up
        // Map pressure scale into dynamic step adjustments (1 to 5 degrees)
        int step = map(R2Analog, 50, 1023, 1, 3);
        if (bucketServoValue < 170) bucketServoValue += step;
    }
    else if (L2Analog > 50) { // Left trigger moves bucket down
        int step = map(L2Analog, 50, 1023, 1, 3);
        if (bucketServoValue > 10) bucketServoValue -= step;
    }

    // --- OPTIMIZED CLAW SERVO TIMING AND STEP LOGIC ---
    if (moveClawServoUp) {
        if (servoDelay >= 2) {
            if (clawServoValue < 170) clawServoValue++;
            servoDelay = 0;
        }
        servoDelay++;
    }
    if (moveClawServoDown) {
        if (servoDelay >= 2) {
            if (clawServoValue > 10) clawServoValue--;
            servoDelay = 0;
        }
        servoDelay++;
    }

    // Ensure values remain bounded safely within structural physical limits
    clawServoValue   = constrain(clawServoValue, 10, 170);
    bucketServoValue = constrain(bucketServoValue, 10, 170);

    // ONLY write to hardware pins if a value has actually changed!
    if (clawServoValue != lastClawValue) {
        clawServo.write(clawServoValue);
        lastClawValue = clawServoValue;
    }
    if (bucketServoValue != lastBucketValue) {
        bucketServo.write(bucketServoValue);
        lastBucketValue = bucketServoValue;
    }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing BluePad32 Engine...");

  // Turn off diagnostic debug noise to avoid core performance loss
  esp_log_level_set("*", ESP_LOG_NONE);

  // Controller engine updates
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // DISABLE VIRTUAL DEVICE EMULATION (Fixes the DS4 console error!)
  BP32.enableVirtualDevice(false);

  // Only enable if you want to forget connected controllers
  //BP32.forgetBluetoothKeys();

  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(armMotor0, OUTPUT);
  pinMode(armMotor1, OUTPUT);

  pinMode(auxLights0, OUTPUT);
  pinMode(auxLights1, OUTPUT);
  digitalWrite(auxLights0, LOW);
 
  bucketServo.attach(bucketServoPin);
  clawServo.attach(clawServoPin);

  bucketServo.write(bucketServoValue);
  clawServo.write(clawServoValue);
  
  Serial.println("Engine Ready.");
}

void loop() {
  // Execute at maximum speed so the internal hardware radio registers incoming bytes instantly
  BP32.update();

  // Handle data mapping on a disciplined non-blocking intervals 
  static uint32_t lastExecution = 0;
  if (millis() - lastExecution >= 15) {
      lastExecution = millis();

      if (activeGamepad && activeGamepad->isConnected()) {
          processGamepadInput(activeGamepad);
      }
  }
}
