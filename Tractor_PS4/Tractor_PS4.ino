#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define LT1 27
#define LT2 14

#define steeringServoPin 23
#define hiLoServoPin 22
#define attachmentLiftServoPin 15
#define ptoServoPin 21

Servo steeringServo;
Servo hiLoServo;
Servo attachmentLiftServo;
Servo ptoServo;

#define driveMotor0 4  // \ Used for controlling drive motor movement
#define driveMotor1 2

#define auxMotor0 26  // \ Used for controlling the power take off point
#define auxMotor1 25

#define auxMotor2 18  // \ Used for controlling auxillary motors
#define auxMotor3 19
#define auxMotor4 16
#define auxMotor5 17

int lightSwitchTime = 0;
int lightSwitchButtonTime = 0;
int lightMode = 0;
bool lightsOn = false;
bool blinkLT = false;
bool hazardLT = false;
bool hazardsOn = false;
bool PTOOn = false;
int adjustedSteeringValue = 90;
int attachmentLiftServoValue = 90;
int ptoServoValue = 90;
int steeringTrim = 0;
int targetValueHigh = 125;
int targetValueLow = 10;

// Triple-tap tracking
unsigned long lastInputTime = 0;
const unsigned long INPUT_TIMEOUT = 40;  // ms — adjust if needed
unsigned long servoTimer = 0;
bool servoActive = false;

int tapCount = 0;
unsigned long firstTapTime = 0;
const unsigned long tapWindow = 800;  // time window to count taps (ms)
int lastDpadValue = 0;

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {

  // Record that we received fresh input
  lastInputTime = millis();

  //Throttle
  processThrottle(ctl->axisY());

  //Steering
  processSteering(ctl->axisRX());
  //Steering Trim
  processTrim(ctl->dpad());
  //Lights
  processLights(ctl->thumbR());

  if (ctl->y() && attachmentLiftServoValue < 170) {
    attachmentLiftServoValue++;
    attachmentLiftServo.write(attachmentLiftServoValue);
    delay(10);
  } else if (ctl->a() && attachmentLiftServoValue > 10) {
    attachmentLiftServoValue--;
    attachmentLiftServo.write(attachmentLiftServoValue);
    delay(10);
  } else if (ctl->b() && ptoServoValue < 140) {
    ptoServoValue++;
    ptoServo.write(ptoServoValue);
    delay(10);
  } else if (ctl->x() && ptoServoValue > 40) {
    ptoServoValue--;
    ptoServo.write(ptoServoValue);
    delay(10);
  }

  // if (!PTOOn) {
  //   if (ctl->a() == 1) {
  //     digitalWrite(PTOMotor0, HIGH);
  //     digitalWrite(PTOMotor1, LOW);
  //   } else if (ctl->y() == 1) {
  //     digitalWrite(PTOMotor0, LOW);
  //     digitalWrite(PTOMotor1, HIGH);
  //   } else if (ctl->a() == 0 && ctl->y() == 0) {
  //     digitalWrite(PTOMotor0, LOW);
  //     digitalWrite(PTOMotor1, LOW);
  //   }
  // }
  if (blinkLT && (millis() - lightSwitchTime) > 300) {
    if (!lightsOn) {
      if (adjustedSteeringValue <= 70) {
        digitalWrite(LT1, HIGH);
        Serial.println(12);
      } else if (adjustedSteeringValue >= 110) {
        digitalWrite(LT2, HIGH);
        Serial.println(14);
      }
      lightsOn = true;
    } else {
      if (adjustedSteeringValue <= 70) {
        digitalWrite(LT2, HIGH);
        digitalWrite(LT1, LOW);
        Serial.println(11);
        delay(10);
        Serial.println(14);
      } else if (adjustedSteeringValue >= 110) {
        digitalWrite(LT1, HIGH);
        digitalWrite(LT2, LOW);
        Serial.println(13);
        delay(10);
        Serial.println(12);
      }
      lightsOn = false;
    }
    lightSwitchTime = millis();
  }
  if (blinkLT && adjustedSteeringValue > 70 && adjustedSteeringValue < 110) {
    digitalWrite(LT1, HIGH);
    digitalWrite(LT2, HIGH);
    Serial.println(12);
    delay(10);
    Serial.println(14);
  }
  if (hazardLT && (millis() - lightSwitchTime) > 300) {
    if (!hazardsOn) {
      digitalWrite(LT1, HIGH);
      digitalWrite(LT2, HIGH);
      Serial.println(12);
      delay(10);
      Serial.println(14);
      hazardsOn = true;
    } else {
      digitalWrite(LT1, LOW);
      digitalWrite(LT2, LOW);
      Serial.println(11);
      delay(10);
      Serial.println(13);
      hazardsOn = false;
    }
    lightSwitchTime = millis();
  }

  int dpadValue = ctl->dpad();  // get current D-pad value
  int targetPosition = -1;

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
      if (dpadValue == 2) {
        targetPosition = targetValueHigh;  
      }
      else if (dpadValue == 1) {
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

  // Turn off servo after 2 seconds
  if (servoActive && millis() - servoTimer >= 2000) {
    hiLoServo.detach();
    servoActive = false;
  }

  // Save last D-pad value
  lastDpadValue = dpadValue;
}


void processThrottle(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
  moveMotor(driveMotor0, driveMotor1, adjustedThrottleValue);
}

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

void processSteering(int axisRXValue) {
  adjustedSteeringValue = (90 - (axisRXValue / 6) - steeringTrim);
  steeringServo.write(adjustedSteeringValue);
}
void processTrim(int dpadValue) {
  if (dpadValue == 4 && steeringTrim < 20) {
    steeringTrim = steeringTrim + 1;
    delay(50);
  } else if (dpadValue == 8 && steeringTrim > -20) {
    steeringTrim = steeringTrim - 1;
    delay(50);
  }
}

void processLights(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchButtonTime) > 300) {
    lightMode++;
    if (lightMode == 1) {
      digitalWrite(LT1, HIGH);
      digitalWrite(LT2, HIGH);
      Serial.println(12);
      delay(10);
      Serial.println(14);
    } else if (lightMode == 2) {
      digitalWrite(LT1, LOW);
      digitalWrite(LT2, LOW);
      delay(100);
      digitalWrite(LT1, HIGH);
      digitalWrite(LT2, HIGH);
      blinkLT = true;
    } else if (lightMode == 3) {
      blinkLT = false;
      hazardLT = true;
    } else if (lightMode == 4) {
      hazardLT = false;
      digitalWrite(LT1, LOW);
      digitalWrite(LT2, LOW);
      Serial.println(11);
      delay(10);
      Serial.println(13);
      lightMode = 0;
    }
    lightSwitchButtonTime = millis();
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {

  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.forgetBluetoothKeys();

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

  hiLoServo.attach(hiLoServoPin);
  hiLoServo.write(10);

  attachmentLiftServo.attach(attachmentLiftServoPin);
  attachmentLiftServo.write(attachmentLiftServoValue);

  ptoServo.attach(ptoServoPin);
  ptoServo.write(ptoServoValue);


  lastInputTime = millis();  // initialize failsafe timer
}



// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { vTaskDelay(1); }
  // Failsafe check: if no input for too long, stop motors
  if (millis() - lastInputTime > INPUT_TIMEOUT) {
    digitalWrite(driveMotor0, LOW);
    digitalWrite(driveMotor1, LOW);
  }
}
