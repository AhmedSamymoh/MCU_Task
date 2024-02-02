/**
 * @file: RoboTech_ESP32_PWM_Control.ino
 * @date: January 13, 2024
 * @author: Ahmed Samy
 *
 * @brief: ESP32 PWM Control for Blue Robotics T200 Thrusters
 *         This code controls the speed of 6 Brushed DC Motors (BCDM T200) using an ESP32.
 *         It replaces the PCA9685 driver and generates PWM signals to control motor speed.
 *
 * @version: 1.0
 *
 **/

#include <Wire.h>
#include <ESP32Servo.h>

#define NUM_MOTORS 6
#define I2C_SLAVE_ADDRESS 8

/* Define the pins for each motor */
const int motorPins[NUM_MOTORS] = {32, 33, 25, 26, 27, 14};

Servo motors[NUM_MOTORS];

bool raspberryPiConnected = false; 

unsigned long previousMillis = 0;
const long interval = 3000; // 3 seconds

void setup() {
  /* for debugging */
  Serial.begin(9600);
  /* Initialize I2C communication as slave */
  Wire.begin(I2C_SLAVE_ADDRESS); 
  Wire.onReceive(receiveEvent);
  
 /* Attach each BDCM to its corresponding pin */
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].setPeriodHertz(50); // 50hz - 20ms 
    motors[i].attach(motorPins[i], 1000, 2000); 
    /* send "stop" signal to ESC */
    motors[i].writeMicroseconds(1500);
  }
  delay(7000); // delay to allow the ESCs to recognize the stopped signal
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (raspberryPiConnected) {
      // Raspberry Pi is connected, display motor speeds
      Serial.println("Raspberry Pi connected. Motor speeds:");
      for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(motors[i].readMicroseconds()); // Print the current motor speed
      }
    } else {
      Serial.println("Pi not connected.");
    }
  }
}


/**
 * @brief Handles reception of signals from Raspberry Pi over I2C.
 * Validates data size and signal range before processing.
 * Sends valid signals to Our T200 motors.
 * @param numBytes Number of bytes received from Pi.
 */

void receiveEvent(int numBytes) {
  raspberryPiConnected = true; 

  if (numBytes >= NUM_MOTORS * 2) { // Ensure received data size is correct

    for (int i = 0; i < NUM_MOTORS; i++) {

      if (Wire.available() >= 2) {
        
        byte lowByte = Wire.read(); 
        byte highByte = Wire.read(); 

        /* Combine bytes to get the signal value */
        int signal = (highByte << 8) | lowByte; 

        /* Ensure the signal is within the range of (1100 :  1900) */
        if (signal < 1100 || signal > 1900)
            continue; // Skipp move to the next motor

        /*  Send signal to the corresponding motor */
        motors[i].writeMicroseconds(signal);

        /* Print the received signal */
        Serial.print("m");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(signal);

      } else {
        Serial.println(" Insufficient bytes available for reading motor signal !!!!");
        /* Consume remaining bytes */
        Wire.readBytes((uint8_t *)0, Wire.available());

        break;
      }

    }
    Serial.print("---------------");
  } else {
    /* Clear the buffer if data size is incorrect */
    Wire.readBytes((uint8_t *)0, numBytes);

    Serial.println("Incomplete data !!");

  }
}


