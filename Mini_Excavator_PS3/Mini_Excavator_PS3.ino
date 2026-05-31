#include <Ps3Controller.h>
#include <ESP32Servo.h>  // by Kevin Harrington

// defines
#define clawServoPin 23
#define cabLights 5

#define mainBoom0 32         // Controls boom movement
#define mainBoom1 19         // Controls boom movement
#define secondBoom0 21       // Controls dipper movement
#define secondBoom1 22       // Controls dipper movement
#define tiltAttach0 33       // Controls bucket movement
#define tiltAttach1 18       // Controls bucket movement
#define thumb0 16            // Controls claw rotation movement
#define thumb1 26            // Controls claw rotation movement

#define leftMotor0 4         // Controls the left motor movement
#define leftMotor1 27        // Controls the left motor movement
#define rightMotor0 17       // Controls the right motor movement
#define rightMotor1 25       // Controls the right motor movement
#define pivot0 13            // Controls cab swing movement              
#define pivot1 12            // Controls cab swing movement

#define controllerMAC "a0:5a:5d:00:53:55" // MAC address of PS3 controller

//-----------------------------------------------------------
// Removed MCP for new PCB board 

Servo clawServo;

int dly = 250;
int clawServoValue = 90;
int clawSpeed = 128;
int player = 0;
int battery = 0;
int servoDelay = 0;


bool cabLightsOn = false;
bool moveClawServoUp = false;
bool moveClawServoDown = false;

int deadzone = 10;

/* USAGE
smoothIt(from, to, value, power, reverse);
from: where does the curve start? Normally 0
to: where does the curve stop? 
value: the actual value you want to have calculated and returned
power: the power of smoothness. 1 is flat. A good value is from 2 to 5
*/

int smoothIt(int from, int to, int val, int power) {
  float to2 = to - from;
  int ret = pow((val - from) / to2, power) * to2 + from;
  return ret;
}

// Moving a motor proportionally
void moveMotorAnalog(int pin0, int pin1, int value, int deadzone=10)
{
  //int power = 2*smoothIt(0, 127, abs(value), 2);
  int power = 2*abs(value);

  if (value > deadzone)
  {
    analogWrite(pin0, power);
    analogWrite(pin1, LOW);
  }
  else if (value < -deadzone)
  {
    analogWrite(pin0, LOW);
    analogWrite(pin1, power);
  }
  else
  {
    analogWrite(pin0, LOW);
    analogWrite(pin1, LOW);
  }
}

// Moving a motor on/off
void moveMotorDigital(int pin0, int pin1, int value)
{
  if (value > 0)
  {
    digitalWrite(pin0, HIGH);
    digitalWrite(pin1, LOW);
  }
  else if (value < 0)
  {
    digitalWrite(pin0, LOW);
    digitalWrite(pin1, HIGH);
  }
  else
  {
    digitalWrite(pin0, LOW);
    digitalWrite(pin1, LOW);
  }
}

// Servo turn proportional - increment based on setting
int moveServo(Servo servo, int value, int speed, int maxValue, int minValue)
{
   int newValue = value + (speed / 20);
   if(newValue < maxValue && newValue > minValue) {
     value = newValue;
     servo.write(newValue);
   }
   return value;
}

// Turn things on and off 
void toggleLights(int pin)
{
  digitalWrite(pin, !digitalRead(pin));
}

void notify() {
  //--------------- Digital D-pad button events --------------
  if (Ps3.event.button_down.up) {
    moveMotorAnalog(thumb0, thumb1, clawSpeed);
  }
  if (Ps3.event.button_up.up) {
    moveMotorAnalog(thumb0, thumb1, 0);
  }
  if (Ps3.event.button_down.down) {
    moveMotorAnalog(thumb0, thumb1, -clawSpeed);
  }
  if (Ps3.event.button_up.down) {
    moveMotorAnalog(thumb0, thumb1, 0);
  }
  
  //---------------- Analog stick value events ---------------
  aa:aa:aa:aa:aa:aa
    int LXValue = Ps3.data.analog.stick.lx;
    int LYValue = Ps3.data.analog.stick.ly;
    moveMotorAnalog(pivot0, pivot1, LXValue); 
    moveMotorAnalog(mainBoom0, mainBoom1, LYValue);
  }

  if (abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2) {
    int RXValue = (Ps3.data.analog.stick.rx);
    int RYValue = (Ps3.data.analog.stick.ry);
    moveMotorAnalog(tiltAttach0, tiltAttach1, RXValue);
    moveMotorAnalog(secondBoom0, secondBoom1, RYValue);
  }

  //------------- Digital shoulder button events -------------
  if (Ps3.event.button_down.l1) {
    moveMotorDigital(leftMotor0, leftMotor1, 1);
  }
  if (Ps3.event.button_up.l1) {
    moveMotorDigital(leftMotor0, leftMotor1, 0);
  }
  if (Ps3.event.button_down.r1) {
    moveMotorDigital(rightMotor0, rightMotor1, 1);
  }
  if (Ps3.event.button_up.r1) {
    moveMotorDigital(rightMotor0, rightMotor1, 0);
  }

  //-------------- Digital trigger button events -------------
  if (Ps3.event.button_down.l2) {
    moveMotorDigital(leftMotor0, leftMotor1, -1);
  }
  if (Ps3.event.button_up.l2) {
    moveMotorDigital(leftMotor0, leftMotor1, 0);
  }
  if (Ps3.event.button_down.r2) {
    moveMotorDigital(rightMotor0, rightMotor1, -1);
  }
  if (Ps3.event.button_up.r2) {
    moveMotorDigital(rightMotor0, rightMotor1, 0);
  }


  //--- Digital cross/square/triangle/circle button events ---
  if (Ps3.event.button_down.cross) {
    moveClawServoUp = true;
  }
  if (Ps3.event.button_up.cross) {
    moveClawServoUp = false;
  }
  if (Ps3.event.button_down.triangle) {
    moveClawServoDown = true;
  }
  if (Ps3.event.button_up.triangle) {
    moveClawServoDown = false;
  }

  //--------------- Digital stick button events --------------
  if (Ps3.event.button_down.l3) {
    toggleLights(cabLights);
  }

  //--------------- Servo movement ---------------------------
  if (moveClawServoUp) {
    if (servoDelay == 3) {
      if (clawServoValue >= 10 && clawServoValue < 170) {
        clawServoValue = clawServoValue + 1;
        clawServo.write(clawServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (moveClawServoDown) {
    if (servoDelay == 3) {
      if (clawServoValue <= 170 && clawServoValue > 10) {
        clawServoValue = clawServoValue - 1;
        clawServo.write(clawServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
}

void onConnect() {
  Serial.println("Connected.");

  // Blink lights 
  for (int i = 0; i <= 5; i++) {
    toggleLights(cabLights);
    delay(200);
  }
}

void setup() {

  Serial.begin(115200);

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin(controllerMAC);

  Serial.println("Ready.");

  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(pivot0, OUTPUT);
  pinMode(pivot1, OUTPUT);

  pinMode(mainBoom0, OUTPUT);
  pinMode(mainBoom1, OUTPUT);
  pinMode(secondBoom0, OUTPUT);
  pinMode(secondBoom1, OUTPUT);
  pinMode(tiltAttach0, OUTPUT);
  pinMode(tiltAttach1, OUTPUT);
  pinMode(thumb0, OUTPUT);
  pinMode(thumb1, OUTPUT);
  pinMode(clawServoPin, OUTPUT);

  pinMode(cabLights, OUTPUT);

  clawServo.attach(clawServoPin);
  clawServo.write(clawServoValue);
}


void loop() {
  if (!Ps3.isConnected())
    return;
  delay(500);
}
