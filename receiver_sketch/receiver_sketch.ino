/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
#define redLED 14
#define greenLED 15
#define blueLED 16
#define RX1PIN 7
#define TX1PIN 8   //UART1 configured

uint8_t dataReceived =0;
uint8_t ledState  =0; // start LED off

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  char a[12];
  int b;
  float c;
  bool d;
  uint8_t direction;
  int duration;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  dataReceived=1;
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
}
 
void setup() {
// Set up pins
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

// LED pins are active low - start with Blue on
  digitalWrite(blueLED, LOW);
  digitalWrite(greenLED, HIGH);
  digitalWrite(redLED, HIGH);

  // Initialize Serial Monitor
  Serial.begin(115200);
  // Initialize Serial1 to communicate with motor driver 
  Serial1.begin(19200, SERIAL_8N1, RX1PIN, TX1PIN);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

  //send empty command to motor driver
  Serial1.write(0x00);
  //check if data is received
  if (dataReceived){
    dataReceived=0; 
    //update the LED color
    Serial.print("Received direction: ");
    Serial.print (myData.direction);
    Serial.print(" duration: ");
    Serial.println(myData.duration);

    if (myData.direction==0){
      Serial.println("  PORT");
      digitalWrite(greenLED, HIGH);
      digitalWrite(redLED, LOW);
      digitalWrite(blueLED, HIGH);
      //motorA full speed forward
      Serial1.write(0x3F);
    }
    else if (myData.direction==1){
      Serial.println("  STARBOARD");
      digitalWrite(blueLED, HIGH);
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, LOW);
      //motorA full speed reverse
      Serial1.write(0x7F); 
    }
    else {
      Serial.println("  OFF");
      digitalWrite(greenLED, HIGH);
      digitalWrite(redLED, HIGH);
      digitalWrite(blueLED, LOW);
    }
  }
  delay(100);
}


// Cytron serial control

// Bit 7 (Channel):
// 0 for selecting motor LEFT.
// 1 for selecting motor RIGHT.

// Bit 6 (Direction):
// 0 to set motor direction to CW.
// 1 to set motor direction to CCW.

// Bit 0 - 5 (Speed):
// 0b000000 or 0 (decimal) to stop.
// 0b111111 or 63 (decimal) to full speed.

// packet = bytearray()
// speed = 0
// currentSpeed = 0
// previousSpeed = 0
// motorLeft = 0
// motorRight = 0
// motorDirection = False

// try:
//     while True:
//         if currentSpeed != previousSpeed:
//             print("Motor Speed: {}, Left: {}, Right: {}".format(speed, motorLeft, motorRight))
//             previousSpeed = currentSpeed
            
//             packet.append(motorLeft)
//             packet.append(motorRight)
//             serialPort.write(packet)
//             packet.clear()
