#include <nRF24L01.h> 
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include <Wire.h>

#include <Servo.h>
#include <Encoder.h> 
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>


RF24 radio(9, 10); // Define the NRF24 module pins (CE, CSN)

const uint64_t pipeAddress = 0xF0F0F0F0E1LL; // Receiver pipe address, should match the transmitter

Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(); // First Motor Shield
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(); // Second Motor Shield

Adafruit_StepperMotor *stepper = AFMS1.getStepper(200, 1);
Adafruit_DCMotor      *leftMotor = AFMS1.getMotor(3);
Adafruit_DCMotor      *rightMotor = AFMS1.getMotor(4);
Adafruit_DCMotor      *turret = AFMS2.getMotor(1);

Servo flexServo;

struct SensorData {
  int xAxisValue;
  int yAxisValue;
  int steps;
  int flexSensorValue;
};

SensorData sensorReading;
int lastEncoderPosition = 0;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, pipeAddress);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();

  AFMS1.begin();
  AFMS2.begin();
  stepper->setSpeed(10); // Adjust the speed as needed based on your motor
  flexServo.attach(10); // Servo motor pin
}

void loop() {
  // Receive sensor data from the transmitter
  if (radio.available()) {
    radio.read(&sensorReading, sizeof(SensorData));
    
    // Use sensorReading values to control the motors

    int xAxisValue = sensorReading.xAxisValue; 
    int yAxisValue = sensorReading.yAxisValue; 
    int steps = sensorReading.steps;
    int flexSensorValue = sensorReading.flexSensorValue;

    // Read the current position of the encoder
    
  

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
    int currentEncoderPosition = steps;

  // Check if the encoder position has changed by 1 value
  if (currentEncoderPosition != lastEncoderPosition) {
    if (currentEncoderPosition - lastEncoderPosition == 1) {
      // If the encoder has been turned by 1 value clockwise, move the stepper motor 4 steps clockwise
      stepper->step(4, FORWARD, SINGLE);  // Adjust the number of steps and mode as needed
    } else if (currentEncoderPosition - lastEncoderPosition == -1) {
      // If the encoder has been turned by 1 value counterclockwise, move the stepper motor 4 steps counterclockwise
      stepper->step(4, BACKWARD, SINGLE);  // Adjust the number of steps and mode as needed
    }

    // Update the last encoder position
    lastEncoderPosition = currentEncoderPosition;
  }

    // Control the servo motor based on flex sensor input
    flexServo.write(flexSensorValue);

    // troubleshoot this turret too
    turret->setSpeed(abs(xAxisValue)); //this liine wasnt her, but just experimenting,it needs to be calibrated anyways                                   
    turret->run(xAxisValue < 0 ? FORWARD : BACKWARD);
   
  
    // Print received data to Serial monitor
    Serial.print("Received Data: ");
    Serial.print(sensorReading.xAxisValue);
    Serial.print(", ");
    Serial.print(sensorReading.yAxisValue);
    Serial.print(", ");
    Serial.print(sensorReading.steps);
    Serial.print(", ");
    Serial.println(sensorReading.flexSensorValue);
  
  }
}
