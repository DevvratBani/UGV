#include <Encoder.h> // this maynot be used

#include <nRF24L01.h> 
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>

RF24 radio(9, 10); // CE, CSN pins
const byte address[6] = "00001";

Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(); // First Motor Shield
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(); // Second Motor Shield

Adafruit_StepperMotor *stepper = AFMS1.getStepper(200, 1);
Adafruit_DCMotor      *leftMotor = AFMS1.getMotor(3);
Adafruit_DCMotor      *rightMotor = AFMS1.getMotor(4);
Adafruit_DCMotor      *oppositeTurnMotor = AFMS2.getMotor(1);

Servo flexServo;

int data[4];

void setup() 
{
  AFMS1.begin();
  AFMS2.begin();
  stepper->setSpeed(100); // Adjust the speed as needed based on your motor
  
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.startListening();
  flexServo.attach(10); // Servo motor pin
}

void loop() 
{
  if (radio.available()) 
  {
    int data[4] = {0};
    radio.read(&data, sizeof(data));

    int xAxisValue = data[0];
    int yAxisValue = data[1];
    int steps      = data[2];
    int flexSensorValue = data[3];

    if (radio.available()) {
    radio.read(&data, sizeof(data));
    Serial.println("Data received: " + String(data[0]) + ", " + String(data[1]) + ", " + String(data[2]) + ", " + String(data[3]));}

    leftMotor->setSpeed(abs(yAxisValue)); // try to put actual speed value here, OR
    rightMotor->setSpeed(abs(yAxisValue)); // try to put setSpeed(yAxisValue), OR 
                                           // define speed and map it with yAxisValue
                                           
    // Control the main motors forward/backward based on joystick input
    leftMotor->run(yAxisValue > 0 ? FORWARD : BACKWARD);
    rightMotor->run(yAxisValue > 0 ? FORWARD : BACKWARD);
    //troubleshoot these turns 
    if(xAxisValue > 0)
    {
      leftMotor->run(FORWARD);
      rightMotor->run(BACKWARD);
    }
    else if(xAxisValue < 0)
    {
      leftMotor->run(BACKWARD);
      rightMotor->run(FORWARD);
    }
    else 
    {
      leftMotor->setSpeed(0); 
      rightMotor->setSpeed(0);
    }
    
    // Use the received step count to control the stepper motor
    // For example, move the motor by the received steps
    stepper->step(steps, FORWARD, SINGLE);

    // Control the servo motor based on flex sensor input
    flexServo.write(flexSensorValue);

    // troubleshoot this turret too
    turret->setSpeed(abs(motorspeed); //this liine wasnt her, but just experimenting
                                       //it needs to be calibrated anyways 
    turret->run(xAxisValue < 0 ? FORWARD : BACKWARD);
  }  
}
