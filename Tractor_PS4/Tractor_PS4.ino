//-----------------------------------------------------------------------------------
// Tractor - BluePad32 with Dual Mode: 1:1 Camp Filtering vs. Open Home Mode
//
//   Camp Mode : fill ASSIGNED_CONTROLLER_MAC with the Bluetooth address of the one
//               controller that belongs to this tractor. Every other controller is
//               rejected and disconnected, so a room full of tractors and gamepads
//               stays paired 1:1.
//   Home Mode : leave ASSIGNED_CONTROLLER_MAC all zeros and the tractor accepts the
//               first controller that connects.
//
// The MAC of any controller that connects is printed to the serial console at
// 115200 baud, so the easiest way to set up a camp build is to flash it once with a
// blank MAC, connect the assigned controller, copy the printed address into
// ASSIGNED_CONTROLLER_MAC, and re-flash.
//
// Controls
//   Left stick Y ....... drive forward / reverse
//   Right stick X ...... steering
//   D-pad left/right ... steering trim
//   D-pad down x3 ...... shift into high range
//   D-pad up x3 ........ shift into low range
//   R3 ................. cycle lights: off -> on -> turn signals -> hazards -> off
//   Triangle / Cross ... attachment lift up / down
//   Circle / Square .... PTO servo up / down
//-----------------------------------------------------------------------------------

#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>

// --- DUAL MODE CONFIGURATION ---
// Camp Mode: paste the assigned controller's MAC here.
//const uint8_t ASSIGNED_CONTROLLER_MAC[6] = {0xA0, 0x5A, 0x5E, 0xA4, 0x7F, 0xDA};
// Home Mode: all zeros pairs with any controller.
const uint8_t ASSIGNED_CONTROLLER_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// --- PIN ASSIGNMENTS ---
#define LT1 27  // Left turn signal / hazard output
#define LT2 14  // Right turn signal / hazard output

#define steeringServoPin 23
#define hiLoServoPin 22
#define attachmentLiftServoPin 15
#define ptoServoPin 21

#define driveMotor0 4  // \ Used for controlling drive motor movement
#define driveMotor1 2

// Reserved header pins - wired on the board but not driven by this sketch.
#define auxMotor0 26  // \ Power take off point
#define auxMotor1 25
#define auxMotor2 18  // \ Auxillary motors
#define auxMotor3 19
#define auxMotor4 16
#define auxMotor5 17

Servo steeringServo;
Servo hiLoServo;
Servo attachmentLiftServo;
Servo ptoServo;

// --- LIGHT STATE ---
unsigned long lightSwitchTime = 0;        // last turn signal / hazard blink toggle
unsigned long lightSwitchButtonTime = 0;  // last R3 press accepted
const unsigned long BLINK_INTERVAL = 300;  // ms between blink toggles
const unsigned long BUTTON_REPEAT = 300;   // ms of debounce on the light button
int lightMode = 0;
bool lightsOn = false;
bool blinkLT = false;
bool hazardLT = false;
bool hazardsOn = false;

// --- STEERING / SERVO STATE ---
int adjustedSteeringValue = 90;
int attachmentLiftServoValue = 90;
int ptoServoValue = 90;
int steeringTrim = 0;

// Hi/Lo range servo: driven only while shifting, then detached so it does not
// buzz or fight the gearbox while parked.
const int targetValueHigh = 115;
const int targetValueLow = 5;
const unsigned long HILO_HOLD_TIME = 3000;  // ms to hold the shift before detaching
                                            // (raise if the gearbox needs longer)
unsigned long servoTimer = 0;
bool servoActive = false;

// Servo stepping rates - these replace the blocking delay() calls that used to
// pace the attachment lift, PTO and steering trim.
const unsigned long SERVO_STEP_INTERVAL = 10;  // ms per attachment / PTO step
const unsigned long TRIM_STEP_INTERVAL = 50;   // ms per steering trim step
unsigned long lastServoStepTime = 0;
unsigned long lastTrimStepTime = 0;

// D-pad bit masks - Bluepad32 reports the D-pad as a bitfield
const int DPAD_UP_BIT = 0x01;
const int DPAD_DOWN_BIT = 0x02;
const int DPAD_RIGHT_BIT = 0x04;
const int DPAD_LEFT_BIT = 0x08;

// Triple-tap tracking for the hi/lo shift
int tapCount = 0;
unsigned long firstTapTime = 0;
const unsigned long tapWindow = 800;  // time window to count taps (ms)
int lastDpadValue = 0;

// Failsafe: cut the drive motors if the controller stops reporting
unsigned long lastInputTime = 0;
const unsigned long INPUT_TIMEOUT = 40;  // ms - adjust if needed

ControllerPtr activeGamepad = nullptr;
bool unsupportedReported = false;  // so a non-gamepad is logged once, not every loop

// Forward declarations
void stopDriveMotors();
void setTurnSignal(int pin, bool on);

//-----------------------------------------------------------------------------------
// Controller pairing
//-----------------------------------------------------------------------------------

bool isMacBlank() {
  for (int i = 0; i < 6; i++) {
    if (ASSIGNED_CONTROLLER_MAC[i] != 0x00) return false;
  }
  return true;
}

void onConnectedController(ControllerPtr ctl) {
  ControllerProperties properties = ctl->getProperties();

  // --- PRINT THE MAC ADDRESS TO THE CONSOLE ---
  Serial.printf("CONNECTED CONTROLLER MAC: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
                properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
                properties.btaddr[3], properties.btaddr[4], properties.btaddr[5]);
  Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(),
                properties.vendor_id, properties.product_id);

  // NOTE: do not call ctl->isGamepad() here. Bluepad32 has not received the
  // controller's first report yet, so the device class is still unset and even a
  // DualShock 4 reports false. The check belongs in loop(), once data arrives.

  // Only one controller drives the tractor - ignore anything that shows up after it.
  if (activeGamepad != nullptr && activeGamepad != ctl) {
    Serial.println("A controller is already connected. Disconnecting...");
    ctl->disconnect();
    return;
  }

  // Check if we are running in Open Home Mode
  if (isMacBlank()) {
    Serial.println("Home Mode Detected (Blank MAC).");
    activeGamepad = ctl;
    unsupportedReported = false;
    lastInputTime = millis();
    ctl->setColorLED(0, 0, 255);  // Solid Blue
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
    activeGamepad = ctl;
    unsupportedReported = false;
    lastInputTime = millis();
    ctl->setColorLED(0, 255, 0);  // Solid Green
  } else {
    Serial.println("Incorrect controller. Disconnecting...");
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (ctl == activeGamepad) {
    Serial.println("Controller disconnected.");
    activeGamepad = nullptr;
    unsupportedReported = false;
    stopDriveMotors();
  }
}

//-----------------------------------------------------------------------------------
// Motors, steering and lights
//-----------------------------------------------------------------------------------

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 15) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -15) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}

void stopDriveMotors() {
  moveMotor(driveMotor0, driveMotor1, 0);
}

void processThrottle(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
  moveMotor(driveMotor0, driveMotor1, adjustedThrottleValue);
}

void processSteering(int axisRXValue) {
  adjustedSteeringValue = (90 - (axisRXValue / 6) - steeringTrim);
  steeringServo.write(adjustedSteeringValue);
}

void processTrim(int dpadValue) {
  if (millis() - lastTrimStepTime < TRIM_STEP_INTERVAL) return;

  if (dpadValue == DPAD_RIGHT_BIT && steeringTrim < 20) {
    steeringTrim++;
    lastTrimStepTime = millis();
  } else if (dpadValue == DPAD_LEFT_BIT && steeringTrim > -20) {
    steeringTrim--;
    lastTrimStepTime = millis();
  }
}

// LT1 and LT2 are the left and right signal outputs. Every state change is echoed
// on the serial console as a status code: 11 = LT1 off, 12 = LT1 on,
// 13 = LT2 off, 14 = LT2 on. Unchanged states are skipped so the blinker logic
// running every loop does not flood the console.
bool lt1State = false;
bool lt2State = false;

void setTurnSignal(int pin, bool on) {
  bool &state = (pin == LT1) ? lt1State : lt2State;
  if (state == on) return;

  state = on;
  digitalWrite(pin, on ? HIGH : LOW);
  if (pin == LT1) {
    Serial.println(on ? 12 : 11);
  } else {
    Serial.println(on ? 14 : 13);
  }
}

void processLights(bool buttonValue) {
  if (!buttonValue || (millis() - lightSwitchButtonTime) <= BUTTON_REPEAT) return;

  lightMode++;
  if (lightMode == 1) {  // Steady on
    setTurnSignal(LT1, true);
    setTurnSignal(LT2, true);
  } else if (lightMode == 2) {  // Blink off/on once, then run turn signals
    setTurnSignal(LT1, false);
    setTurnSignal(LT2, false);
    delay(100);
    setTurnSignal(LT1, true);
    setTurnSignal(LT2, true);
    blinkLT = true;
  } else if (lightMode == 3) {  // Hazards
    blinkLT = false;
    hazardLT = true;
  } else if (lightMode == 4) {  // All off
    hazardLT = false;
    setTurnSignal(LT1, false);
    setTurnSignal(LT2, false);
    lightMode = 0;
  }
  lightSwitchButtonTime = millis();
}

// Turn signals follow the steering angle: left, right, or both when centered.
void updateTurnSignals() {
  if (blinkLT && (millis() - lightSwitchTime) > BLINK_INTERVAL) {
    if (!lightsOn) {
      if (adjustedSteeringValue <= 70) {
        setTurnSignal(LT1, true);
      } else if (adjustedSteeringValue >= 110) {
        setTurnSignal(LT2, true);
      }
      lightsOn = true;
    } else {
      if (adjustedSteeringValue <= 70) {
        setTurnSignal(LT1, false);
        setTurnSignal(LT2, true);
      } else if (adjustedSteeringValue >= 110) {
        setTurnSignal(LT2, false);
        setTurnSignal(LT1, true);
      }
      lightsOn = false;
    }
    lightSwitchTime = millis();
  }

  if (blinkLT && adjustedSteeringValue > 70 && adjustedSteeringValue < 110) {
    setTurnSignal(LT1, true);
    setTurnSignal(LT2, true);
  }

  if (hazardLT && (millis() - lightSwitchTime) > BLINK_INTERVAL) {
    setTurnSignal(LT1, !hazardsOn);
    setTurnSignal(LT2, !hazardsOn);
    hazardsOn = !hazardsOn;
    lightSwitchTime = millis();
  }
}

//-----------------------------------------------------------------------------------
// Attachments
//-----------------------------------------------------------------------------------

// Triangle / Cross raise and lower the attachment lift, Circle / Square drive the
// PTO servo. Stepping is rate limited instead of using blocking delay() calls.
void processAttachments(ControllerPtr ctl) {
  if (millis() - lastServoStepTime < SERVO_STEP_INTERVAL) return;

  if (ctl->y() && attachmentLiftServoValue < 170) {
    attachmentLiftServoValue++;
    attachmentLiftServo.write(attachmentLiftServoValue);
  } else if (ctl->a() && attachmentLiftServoValue > 10) {
    attachmentLiftServoValue--;
    attachmentLiftServo.write(attachmentLiftServoValue);
  } else if (ctl->b() && ptoServoValue < 140) {
    ptoServoValue++;
    ptoServo.write(ptoServoValue);
  } else if (ctl->x() && ptoServoValue > 40) {
    ptoServoValue--;
    ptoServo.write(ptoServoValue);
  } else {
    return;  // nothing moved, do not consume the step interval
  }

  lastServoStepTime = millis();
}

// Triple tapping D-pad down shifts into high range, D-pad up into low range.
void processHiLoShift(int dpadValue) {
  // Detect edge (new press)
  if (dpadValue != lastDpadValue && dpadValue != 0) {
    // Check if tap window expired
    if (millis() - firstTapTime > tapWindow) {
      tapCount = 0;  // reset taps
      firstTapTime = millis();
    }

    tapCount++;                                  // increment tap count
    if (tapCount == 1) firstTapTime = millis();  // start timer on first tap

    // Triple-tap detected
    if (tapCount >= 3) {
      int targetPosition = -1;
      if (dpadValue == DPAD_DOWN_BIT) {
        targetPosition = targetValueHigh;
      } else if (dpadValue == DPAD_UP_BIT) {
        targetPosition = targetValueLow;
      }

      if (targetPosition != -1) {
        hiLoServo.attach(hiLoServoPin);
        hiLoServo.write(targetPosition);
        servoTimer = millis();
        servoActive = true;
      }

      // Reset tap count after activation
      tapCount = 0;
      firstTapTime = 0;
    }
  }

  // Save last D-pad value
  lastDpadValue = dpadValue;
}

// Release the hi/lo servo once the shift has had time to complete.
void updateHiLoServo() {
  if (servoActive && millis() - servoTimer >= HILO_HOLD_TIME) {
    hiLoServo.detach();
    servoActive = false;
  }
}

//-----------------------------------------------------------------------------------
// Main input handling
//-----------------------------------------------------------------------------------

void processGamepad(ControllerPtr ctl) {
  // Record that we received fresh input
  lastInputTime = millis();

  processThrottle(ctl->axisY());   // Throttle
  processSteering(ctl->axisRX());  // Steering
  processTrim(ctl->dpad());        // Steering trim
  processLights(ctl->thumbR());    // Lights
  processAttachments(ctl);         // Attachment lift and PTO
  processHiLoShift(ctl->dpad());   // Hi/Lo range shift

  // Release the hi/lo servo here, on the controller frame, exactly as the
  // original sketch did. Running it from loop() instead cuts the shift short.
  updateHiLoServo();
}

//-----------------------------------------------------------------------------------
// Arduino entry points
//-----------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  Serial.println(isMacBlank() ? "Pairing: Home Mode (any controller)."
                              : "Pairing: Camp Mode (assigned controller only).");

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Only enable if you want to forget connected controllers
  //BP32.forgetBluetoothKeys();

  // DISABLE VIRTUAL DEVICE EMULATION (Fixes the DS4 console error!)
  BP32.enableVirtualDevice(false);

  pinMode(driveMotor0, OUTPUT);
  pinMode(driveMotor1, OUTPUT);
  pinMode(LT1, OUTPUT);
  pinMode(LT2, OUTPUT);

  digitalWrite(driveMotor0, LOW);
  digitalWrite(driveMotor1, LOW);
  digitalWrite(LT1, LOW);
  digitalWrite(LT2, LOW);

  steeringServo.attach(steeringServoPin);
  steeringServo.write(adjustedSteeringValue);

  // Park the hi/lo servo in low range. It stays attached and holding until the
  // first shift - do NOT arm the detach timer here, or the gearbox is left
  // unpowered before the driver has shifted at all.
  hiLoServo.attach(hiLoServoPin);
  hiLoServo.write(targetValueLow);

  attachmentLiftServo.attach(attachmentLiftServoPin);
  attachmentLiftServo.write(attachmentLiftServoValue);

  ptoServo.attach(ptoServoPin);
  ptoServo.write(ptoServoValue);

  lastInputTime = millis();  // initialize failsafe timer

  Serial.println("Engine Ready.");
}

void loop() {
  // This call fetches all the controllers' data.
  bool dataUpdated = BP32.update();

  if (dataUpdated && activeGamepad && activeGamepad->isConnected() && activeGamepad->hasData()) {
    // The device class is only known once the controller has sent a report, so
    // this is the earliest point where isGamepad() gives a trustworthy answer.
    if (activeGamepad->isGamepad()) {
      processGamepad(activeGamepad);
    } else if (!unsupportedReported) {
      Serial.println("Unsupported controller - not a gamepad.");
      unsupportedReported = true;
    }
  } else {
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
    vTaskDelay(1);
  }

  // Blinkers keep running even when no fresh controller frame arrived.
  updateTurnSignals();

  // Failsafe check: if no input for too long, stop motors
  if (millis() - lastInputTime > INPUT_TIMEOUT) {
    stopDriveMotors();
  }
}
