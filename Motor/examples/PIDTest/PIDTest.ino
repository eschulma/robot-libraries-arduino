#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
#include <Odometer.h>
#include <math.h>

// Quadrature encoders
// left = 0 and right = 1 in the arrays
// for interrupts, 0 = pin 2, 1 = pin 3
const short c_encoderInterruptPin[2] = {2, 3};
const short c_encoderInterrupt[2] = {0, 1};
const boolean _bIsMotorReversed[2] = {false, true};
volatile boolean _bLastDigitalValue[2] = {LOW, LOW};
volatile long _lEncoderTicks[2] = {0, 0};

// L293D motor driver
const short c_motorEnablePin[2] = {10, 11}; // PWM
const short c_motorDirectionPin[2][2] = {{8, 9}, {12, 13}};
boolean bIsMotorRunningForward[2];

// Pins for digitalReadFast must be defined at compile time,
// so no arrays here
// Right encoder
#define c_RightEncoderPinDirection 7
#define c_RightTriggerPin 3
// Left encoder
#define c_LeftEncoderPinDirection 4
#define c_LeftTriggerPin 2

static boolean debouncing = true; // necessary!!!
long oldRightTicks = -256;
Odometer *robotOdometer;
float commandedVelocity = 0.0f;
float commandedAngularVelocity = 0.0f;

void setup() {
  Serial.begin(57600);

  // disable motors first
  SetupMotors();
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
    
  // all values in meters
  double wheelDiameter = 0.06985;
  double trackWidth = 0.1778;
  int ticksPerRevolution = 128;
  
  float velocityPParam = 0.1f;
  float velocityIParam = 0.0f;
  float turnPParam = 0.4f;
  float turnIParam = 0.0f;
    
  // in meters: wheel diameter, track width, ticks per revolution, current time
  robotOdometer = new Odometer(wheelDiameter, trackWidth, ticksPerRevolution, millis());
  
  // setup PID parameters
  robotOdometer->setPIDValues(velocityPParam, velocityIParam, turnPParam, turnIParam);
  
  SetupEncoders();   
  
  Serial.println("Firefighter Arduino setup complete.");
}

void loop() {  
  static long lastPublish = 0; 
  
  debouncing = true;
    
  robotOdometer->calculate(_lEncoderTicks[0], _lEncoderTicks[1], millis());
  
  // compare our speed to desired velocity, output new voltages
  robotOdometer->getNewVelocity(commandedVelocity, commandedAngularVelocity);
  
  // for debugging
  long ardNow = millis();
  if(ardNow - lastPublish > 500) {
    Serial.print("Velocity left: ");
    Serial.println(robotOdometer->VLeft);
    Serial.print("Velocity right: ");
    Serial.println(robotOdometer->VRight);
    Serial.print("PID velocity left: ");
    Serial.println(robotOdometer->NormalizedLeftCV);
    Serial.print("PID velocity right: ");
    Serial.println(robotOdometer->NormalizedRightCV);
   
    lastPublish = ardNow;
  }
    
  setMotorSpeed(0, robotOdometer->NormalizedLeftCV);
  setMotorSpeed(1, robotOdometer->NormalizedRightCV);
  
  delay(5);
}

void SetupMotors() {
  // Serial.println("Disabling motors...");  
  for(int i = 0; i < 2; i++) {
    pinMode(c_motorEnablePin[i], OUTPUT);
    analogWrite(c_motorEnablePin[i], 0);
  }
  // Serial.println("Motors disabled."); 
  
  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 2; j++) {
      pinMode(c_motorDirectionPin[i][j], OUTPUT);
      digitalWrite(c_motorDirectionPin[i][j], LOW);
    }
    
    setMotorDirection(i, true);
  }    
}

// Set motor spin as forwards or backwards
void setMotorDirection(int nMotorIndex, boolean bForward) {
  if(bForward) {
    digitalWrite(c_motorDirectionPin[nMotorIndex][0], LOW);
    digitalWrite(c_motorDirectionPin[nMotorIndex][1], HIGH); 
    bIsMotorRunningForward[nMotorIndex] = true;
  }
  else {
    digitalWrite(c_motorDirectionPin[nMotorIndex][0], HIGH);
    digitalWrite(c_motorDirectionPin[nMotorIndex][1], LOW);
    bIsMotorRunningForward[nMotorIndex] = false;    
  }  
}

void setMotorSpeed(int nMotorIndex, float normVelocity) {
  // ignore fluctuations about 0
  const float velocityDeadzone = 0.01;
  if(fabs(normVelocity) < velocityDeadzone) {
    normVelocity = 0;
  }
  
  if(normVelocity < 0) {
    // change direction first, if needed
    if(bIsMotorRunningForward[nMotorIndex]) {
      analogWrite(c_motorEnablePin[nMotorIndex], LOW);
      setMotorDirection(nMotorIndex, false);
    }
    normVelocity *= -1;  // make this a positive number
  }
  else {
    // change direction first, if needed
    if(!bIsMotorRunningForward[nMotorIndex]) {
      analogWrite(c_motorEnablePin[nMotorIndex], LOW);
      setMotorDirection(nMotorIndex, true);
    }    
  }
  
  // calculate new speed with floating point
  float scaledVelocity = normVelocity * 255;
  // Serial.print("Scaled velocity: ");
  // Serial.println(scaledVelocity);
  // write out new speed
  analogWrite(c_motorEnablePin[nMotorIndex], scaledVelocity);
}

// Quadrature encoders
void SetupEncoders()
{
  // set relevant interrupt pins as input
  pinMode(c_LeftTriggerPin, INPUT);
  digitalWrite(c_LeftTriggerPin, LOW);  // turn on pulldown resistor
  pinMode(c_RightTriggerPin, INPUT);
  digitalWrite(c_RightTriggerPin, LOW);  // turn on pulldown resistor
  
  // set direction pins as input
  pinMode(c_LeftEncoderPinDirection, INPUT);
  digitalWrite(c_LeftEncoderPinDirection, LOW);
  pinMode(c_RightEncoderPinDirection, INPUT);
  digitalWrite(c_RightEncoderPinDirection, LOW);
   
  // attach interrupts for each encoder
  attachInterrupt(c_encoderInterrupt[0], HandleLeftMotorInterrupt, CHANGE); 
  attachInterrupt(c_encoderInterrupt[1], HandleRightMotorInterrupt, CHANGE); 
}

void HandleLeftMotorInterrupt() {
  HandleMotorInterrupt(0);
}

void HandleRightMotorInterrupt() {
  // Serial.println("Hi right!");
  HandleMotorInterrupt(1);
}

// Interrupt service routines for the right motor's quadrature encoder
void HandleMotorInterrupt(int nMotorIndex)
{  
  if(debouncing) {
    delay(1);
  }
     
  boolean _bClockDirection;
  boolean bThisRead;
  
  if(nMotorIndex == 0) {
    _bClockDirection = digitalReadFast(c_LeftEncoderPinDirection);   // read the input pin
    bThisRead = digitalReadFast(c_LeftTriggerPin);
  }
  else {
    _bClockDirection = digitalReadFast(c_RightEncoderPinDirection);   // read the input pin
    bThisRead = digitalReadFast(c_RightTriggerPin);
  }

  // debounce
  if(bThisRead != _bLastDigitalValue[nMotorIndex]) {
    _bLastDigitalValue[nMotorIndex] = bThisRead;
    
    // debouncing = false; // works only if loop has no delay
    
    // and adjust counter + if running counter-clockwise    
    if(_bIsMotorReversed[nMotorIndex]) {
      _lEncoderTicks[nMotorIndex] -= _bClockDirection ? -1 : +1;      
    }
    else {
      _lEncoderTicks[nMotorIndex] += _bClockDirection ? -1 : +1;
    }        
  }
}



