#include <Arduino.h>

#define RXD2 16  // You can change this to other GPIO pins if needed
#define TXD2 17  // You can change this to other GPIO pins if needed

void setup() {
  // Start the default Serial for debugging (USB serial communication)
  Serial.begin(9600);
  
  // Initialize Serial2 with the selected baud rate and pins
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Let the user know that Serial2 is initialized
  Serial.println("Serial2 is initialized");
}

// the loop function runs over and over again forever
void loop() {
  
    if (Serial2.available() > 0) {
    // Read one byte from the serial buffer
    int incomingByte = Serial2.read();
    // Print the received byte as a character
    Serial.print("Received: ");
    Serial.println((char)incomingByte);
    }
}