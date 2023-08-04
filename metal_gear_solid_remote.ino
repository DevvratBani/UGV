
#include <nRF24L01.h> // nrf library moment
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h>
#include <Servo.h>
#include <Encoder.h>

RF24 radio(9, 10); // CE, CSN pins
const byte address[6] = "00001";

// Define analog input pins for joysticks and flex sensor
const int xAxisPin = A0;
const int yAxisPin = A1;
const int flexSensorPin = A2;
const int encoderPinA = 2;
const int encoderPinB = 3;

Encoder myEncoder(encoderPinA, encoderPinB);
int xAxisValue, yAxisValue, steps, flexSensorValue;

void setup() 
{
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH); // try deleting this line it wasnt
                                  // there before. also try LOW

  pinMode(xAxisPin, INPUT);
  pinMode(yAxisPin, INPUT);
  pinMode(flexSensorPin, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);


   Serial.begin(9600);
}

void loop() 
{
  steps = myEncoder.read();
  xAxisValue = analogRead(xAxisPin);  // Read joystick and sensor values
  yAxisValue = analogRead(yAxisPin);
  flexSensorValue = analogRead(flexSensorPin);

  // Prepare data to send
  int data[4];
  data[0] = map(xAxisValue, 0, 1023, -255, 255); // Joystick X-axis value
  data[1] = map(yAxisValue, 0, 1023, -255, 255); // Joystick Y-axis value
  data[2] = steps; // Turret stepper motor value 
  data[3] = map(flexSensorValue, 0, 1023, 0, 180); // Flex sensor value (0-180 degrees)

  // Send data via NRF24L01 module
  radio.write(&data, sizeof(data));

  delay(50); // Adjust this delay as needed
}
