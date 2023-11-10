#include <nRF24L01.h> // nrf library 
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#include <SPI.h>
#include <Servo.h>
#include <Encoder.h>

RF24 radio(9, 10); // Define the NRF24 module pins (CE, CSN)

const uint64_t pipeAddress = 0xF0F0F0F0E1LL; // Transmitter pipe address

const int xAxisPin = A0;
const int yAxisPin = A1;
const int flexSensorPin = A2;
const int encoderPinA = 2;
const int encoderPinB = 3;

Encoder myEncoder(encoderPinA, encoderPinB);

struct SensorData {
  int xAxisValue;
  int yAxisValue;
  int steps;
  int flexSensorValue;
};

SensorData sensorReading;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(pipeAddress);
  radio.setPALevel(RF24_PA_HIGH);
  radio.stopListening();

  pinMode(xAxisPin, INPUT);
  pinMode(yAxisPin, INPUT);
  pinMode(flexSensorPin, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
}

void loop() {
  // Read sensor data (You need to implement this part)
  sensorReading.xAxisValue = map(analogRead(A0), 0, 1023, -255, 255);
  sensorReading.yAxisValue = map(analogRead(A1), 0, 1023, -255, 255);
  sensorReading.steps = myEncoder.read();
  sensorReading.flexSensorValue = map(analogRead(A2), 0, 1023, 0, 180);
  // Transmit sensor data to the receiver
  radio.write(&sensorReading, sizeof(SensorData));

  // Add a delay or other logic as needed
  delay(10); // Send data every 1 second, for example
}
