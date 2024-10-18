
#include <Adafruit_MotorShield.h>
#include <PID_v1.h>

#define PIN_INPUT 0 //Left sensor
#define PIN_OUTPUT 3 //Right sensor

int leftSensorPin = A0;   // select the input pin for the potentiometer
int rightSensorPin = A1;   // select the input pin for the potentiometer
int leftSensorRaw = 0;  // variable to store the value coming from the sensor
int rightSensorRaw = 0;  // variable to store the value coming from the sensor
int leftSensorAdjusted = 0; 
int rightSensorAdjusted = 0; 
float scale = 0.7; // Scaler to change so we don't need to adjust setpoint individually

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
// You can also make another motor on port M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

//Define Variables we'll be connecting to
double Setpoint, leftInput, leftOutput, rightInput, rightOutput;;


//Specify the links and initial tuning parameters
double Kp=0.75, Ki=0, Kd=0.12;
PID leftWheelPID(&leftInput, &leftOutput, &Setpoint, Kp, Ki, Kd, DIRECT);//left wheel uses the right sensor
PID rightWheelPID(&rightInput, &rightOutput, &Setpoint, Kp, Ki, Kd, DIRECT);//right weheel uses the left sensor 

//the sensor are switched bc PID would make it go faster and it would need to go slower if it was on black; can be written better later for the reprot 

void setup() {
  // put your setup code here, to run once:
  // declare the ledPin as an OUTPUT:
   if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  Serial.begin(9600);

  Setpoint = scale * 75; //75 is the average sensor output when displayed on the tape and scaled

  //Setting intial speed 
  leftMotor->setSpeed(120);
  rightMotor->setSpeed(120);
  
  //Motor move forward
  rightMotor->run(FORWARD);
  leftMotor->run(FORWARD);

  // turn on motor
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  
  //Setting up PID
  leftWheelPID.SetMode(AUTOMATIC);
  rightWheelPID.SetMode(AUTOMATIC);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  //Read the raw value from the left IR sensor 
  leftSensorRaw = analogRead(leftSensorPin);
  //Adjusted the raw value and map it to a range of 0 to 255
  leftSensorAdjusted = map(leftSensorRaw, 0, 1023, 0, 255);
  delay(2);

  // Repeating but for right sensor 
  rightSensorRaw = analogRead(rightSensorPin);
  rightSensorAdjusted = map(rightSensorRaw, 0, 1023, 0, 255);
  delay(2);

  //Scaling the input for the PID computation 
  leftInput = scale * leftSensorAdjusted;
  rightInput = scale * rightSensorAdjusted;
  leftWheelPID.Compute();
  rightWheelPID.Compute();

  //this should basically mean that 
  leftMotor->setSpeed(leftOutput);
  rightMotor->setSpeed(rightOutput);

  rightMotor->run(FORWARD);
  leftMotor->run(FORWARD);

  //Transmits sensor reading and both servo angles, separated by commas for easy splitting later
  Serial.print(leftSensorAdjusted);
  Serial.print(",");
  Serial.print(rightSensorAdjusted);
  Serial.print(",");
  Serial.print(leftOutput);
  Serial.print(",");
  Serial.println(rightOutput);

}

