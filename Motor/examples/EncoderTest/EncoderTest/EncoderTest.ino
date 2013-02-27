#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
#include <Motor.h>
#include <L293GearMotor.h>
#include <math.h>
#include <Odometer.h>

// Quadrature encoders
// left = 0 and right = 1 in the arrays
// for interrupts, 0 = pin 2, 1 = pin 3
const short c_encoderInterruptPin[2] = {2, 3};
const short c_encoderInterrupt[2] = {0, 1};
volatile boolean _bLastDigitalValue[2] = {LOW, LOW};

long oldRightTicks = 0;
long oldLeftTicks = 0;

// L293D motor driver
L293GearMotor motor[2] = { L293GearMotor(10, 8, 9, true),
                              L293GearMotor(11, 12, 13, false) };
// initializes with wheel diameter [m], trackwidth [m], ticks per revolution, current time
Odometer robotOdometer(motor, 0.06985, 0.1778, 128, millis());                           

// Pins for digitalReadFast must be defined at compile time,
// so no arrays here
// Right encoder
#define c_RightEncoderPinDirection 7
#define c_RightTriggerPin 3
// Left encoder
#define c_LeftEncoderPinDirection 4
#define c_LeftTriggerPin 2

void setup() {
  Serial.begin(57600);
  
  // don't do this, messes up motors!!
  // pinMode(13, OUTPUT);
  // digitalWriteFast(13, LOW);
       
  SetupEncoders();   
  
  Serial.println("Firefighter Arduino setup complete.");
  
  Serial.println("slowly ahead");  
  // full steam ahead!
  for(int i = 0; i < 2; i++) {
    motor[i].setNormalizedVelocity(0.2);
  }  
}

void loop() {  
  static long lastDebugPublish = 0; 
  static long lastPositionUpdate = 0;
  
  long now = millis();       
  robotOdometer.calculate(now);
    
 // for debugging
  if(now - lastDebugPublish > 25) {
    // put Serial debug in here   
    long leftTicks = motor[MOTOR_LEFT].getOdometerValue();
    long rightTicks = motor[MOTOR_RIGHT].getOdometerValue();
    
    if(leftTicks != oldLeftTicks) {
      Serial.print("L : ");
      Serial.print(leftTicks);
  
      Serial.print("  R : ");
      Serial.println(rightTicks);
      
      oldRightTicks = rightTicks;
    }  
      
    lastDebugPublish = now;
  }
}

void stop() {
  for(int i = 0; i < 2; i++) {
    motor[i].stop();
  }
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
  attachInterrupt(c_encoderInterrupt[MOTOR_LEFT], HandleLeftMotorInterrupt, CHANGE); 
  attachInterrupt(c_encoderInterrupt[MOTOR_RIGHT], HandleRightMotorInterrupt, CHANGE); 
}

void HandleLeftMotorInterrupt() {
  HandleMotorInterrupt(MOTOR_LEFT);
}

void HandleRightMotorInterrupt() {
  // Serial.println("Hi right!");
  HandleMotorInterrupt(MOTOR_RIGHT);
}

// Interrupt service routines for the right motor's quadrature encoder
void HandleMotorInterrupt(int nMotorIndex)
{      
  boolean _bClockDirection;
  boolean bThisRead;
  
  if(nMotorIndex == MOTOR_LEFT) {
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
        
    // and adjust counter + if running counter-clockwise    
    if(motor[nMotorIndex].isOdometerPositiveForward()) {
     _bClockDirection ? motor[nMotorIndex].decrementOdometer() : motor[nMotorIndex].incrementOdometer();             
    }
    else {
     _bClockDirection ? motor[nMotorIndex].incrementOdometer() : motor[nMotorIndex].decrementOdometer();  
    }
  }
}



