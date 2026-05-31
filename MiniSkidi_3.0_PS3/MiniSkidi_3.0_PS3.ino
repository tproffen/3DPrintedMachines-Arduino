//-----------------------------------------------------------------------------------
// Changes:
//   - Driving and arm now analog controls
//   - Removed delay(10)'s
//-----------------------------------------------------------------------------------

#include <Ps3Controller.h>
#include <ESP32Servo.h> // by Kevin Harrington

#define rightMotor0 25
#define rightMotor1 26

#define leftMotor0 33
#define leftMotor1 32

#define armMotor0 21
#define armMotor1 19

#define bucketServoPin  23
#define clawServoPin 22

#define auxLights0 18
#define auxLights1 5

#define controllerMAC "30:76:f5:93:6e:b2"

Servo bucketServo;
Servo clawServo;

int bucketServoValue = 140;
int clawServoValue = 150;
int servoDelay = 0;

bool moveClawServoUp = false;
bool moveClawServoDown = false;
bool moveBucketServoUp = false;
bool moveBucketServoDown = false;

int deadzone = 10;

// Moving a motor proportionally
void moveMotorAnalog(int pin0, int pin1, int value, int deadzone=10)
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

// Called when controller input changed
void notify()
{
  //---------------- Analog stick value events ---------------
  if ( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ) {
   // Steer
    int LXValue = Ps3.data.analog.stick.lx;
    moveMotorAnalog(rightMotor0, rightMotor1, 2*LXValue);
    moveMotorAnalog(leftMotor0, leftMotor1, -2*LXValue);

    // Drive - only if we are NOT turning
    if(abs(LXValue) < deadzone) {
      int LYValue = Ps3.data.analog.stick.ly;
      moveMotorAnalog(rightMotor0, rightMotor1, 2*LYValue);
      moveMotorAnalog(leftMotor0, leftMotor1, 2*LYValue);
    }
  }

  if ( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ) {
    int LYValue = Ps3.data.analog.stick.ry;
    moveMotorAnalog(armMotor0, armMotor1, 2*LYValue);

    int LXValue = Ps3.data.analog.stick.rx;
    //bucketServoValue = moveServo(bucketServo, bucketServoValue, LXValue, 170, 90);
  }

  //------------- Digital shoulder button events -------------

  if ( Ps3.event.button_down.l1 )
  {
    moveClawServoUp = true;
  }
  if ( Ps3.event.button_up.l1 )
  {
    moveClawServoUp = false;
  }
  if ( Ps3.event.button_down.r1 )
  {
    moveClawServoDown = true;
  }
  if ( Ps3.event.button_up.r1 )
  {
    moveClawServoDown  = false;
  }
  if ( Ps3.event.button_down.l2 )
  {
    moveBucketServoDown = true;
  }
  if ( Ps3.event.button_up.l2 )
  {
    moveBucketServoDown = false;
  }
  if ( Ps3.event.button_down.r2 )
  {
    moveBucketServoUp = true;
  }
  if ( Ps3.event.button_up.r2 )
  {
    moveBucketServoUp = false;
  }
  if ( Ps3.event.button_down.r3 )
  {
    toggleLights(auxLights1);
  }

  if(moveClawServoUp)
  {
    if(servoDelay == 2)
    {
    if (clawServoValue >= 10 && clawServoValue < 170)
    {
      clawServoValue = clawServoValue + 1;
      clawServo.write(clawServoValue);
    }
    servoDelay = 0;
    }
    servoDelay++;
  }
  if(moveClawServoDown)
  {
    if(servoDelay == 2)
    {
    if (clawServoValue <= 170 && clawServoValue > 10)
    {
      clawServoValue = clawServoValue - 1;
      clawServo.write(clawServoValue);
    }
    servoDelay = 0;
    }
    servoDelay++;
  }
  if(moveBucketServoUp)
  {
    if(servoDelay == 2)
    {
      if(bucketServoValue >= 10 && bucketServoValue < 170)
      {
        bucketServoValue = bucketServoValue + 1;
        bucketServo.write(bucketServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if(moveBucketServoDown)
  {
    if(servoDelay == 2)
    {
      if(bucketServoValue <= 170 && bucketServoValue > 10)
      {
        bucketServoValue = bucketServoValue - 1;
        bucketServo.write(bucketServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
}

void onConnect() {
  Serial.println("Connected.");

  // Blink lights 
  for (int i = 0; i <= 6; i++) {
    toggleLights(auxLights1);
    delay(200);
  }
}

void setup() {

  Serial.begin(115200);

  Serial.println("Connecting.");
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Serial.println(controllerMAC);
  Ps3.begin(controllerMAC);
  Serial.println("Ready.");
  
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(armMotor0, OUTPUT);
  pinMode(armMotor1, OUTPUT);

  pinMode(auxLights0, OUTPUT);
  pinMode(auxLights1, OUTPUT);
  digitalWrite(auxLights0, LOW); // gets swithced on auxLights 1 (LED polarity)
 
  bucketServo.attach(bucketServoPin);
  clawServo.attach(clawServoPin);

  bucketServo.write(bucketServoValue);
  clawServo.write(clawServoValue);

}

void loop() {
  if (!Ps3.isConnected())
    return;
  delay(500);

}
