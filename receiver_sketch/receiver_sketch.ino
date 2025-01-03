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
int this_speed, last_speed =0;  // 0 (stop) to 63 (max)
uint8_t this_direction, last_direction =0;
uint8_t command =0;
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
  //check if data is received
  if (dataReceived){
    dataReceived=0; 
    //update the LED color
    Serial.print("Received direction: ");
    this_direction=myData.direction;
    Serial.print(this_direction);
//    Serial.print(" duration: ");
//    Serial.println(myData.duration);
    Serial.print(" last_speed: ");
    Serial.print(last_speed);
    
    // PORT
    if (this_direction==0){
      Serial.print("  PORT");
      digitalWrite(greenLED, HIGH);digitalWrite(redLED, LOW);digitalWrite(blueLED, HIGH);
      //speed control
      if(last_direction==0) // last command was same direction
        this_speed=0x3F; // full speed
      else this_speed=0x0F; // ramp up speed
      Serial.print(" this _speed: ");
      Serial.print(this_speed);
      command = (0b00000000|this_speed);
      Serial.print(" command= ");Serial.println(command);
      //motorA @ this_speed forward
      Serial1.write(command); 
      last_speed=this_speed;
      last_direction=this_direction;
    }
    else if (this_direction==1){
      Serial.print("  STARBOARD");
      digitalWrite(greenLED, LOW);digitalWrite(blueLED, HIGH);digitalWrite(redLED, HIGH);
      if(last_direction==1) // last command was same direction
        this_speed=0x3F; // full speed
      else this_speed=0x0F; // ramp up speed
      Serial.print(" this _speed: ");
      Serial.print(this_speed);
      command = (0b01000000|this_speed);
      Serial.print(" command= ");Serial.println(command);
      //motorA @ this_speed reverse
      Serial1.write(command); 
      last_speed=this_speed;
      last_direction=this_direction;
    }
    else {
      Serial.println("  OFF");
      digitalWrite(greenLED, HIGH);
      digitalWrite(redLED, HIGH);
      digitalWrite(blueLED, LOW);
      //send empty command to motor driver
      Serial1.write(0x00);
      last_speed=0;
      last_direction=this_direction;
    }
  }
  else{   // NO COMMS FROM TRANSMITTER
      //send empty command to motor driver
    Serial1.write(0x00);
    delay (200);
  }
  delay(100);

}


// Cytron serial control

// Bit 7 (Channel):
// 0 for selecting motor A (Cytron calls LEFT).
// 1 for selecting motor B (Cytrn calls RIGHT) BRIGHT.

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

