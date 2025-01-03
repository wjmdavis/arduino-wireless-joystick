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
#define stickRight 3//6 D # not pin number!
#define stickLeft 4//7 D# not pin number
#define stickGreen 5//8
#define stickRed 6//9

// RECEIVER MAC is 74:4d:bd:a2:3b:74   12/17/24
uint8_t broadcastAddress[] = {0x74, 0x4D, 0xBD, 0xA2, 0x3B, 0x74};

// Structure example to send data
// Must match the receiver structure
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

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Set up pins
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(stickRight, INPUT_PULLUP);
  pinMode(stickLeft, INPUT_PULLUP);
  pinMode(stickGreen, OUTPUT);
  pinMode(stickRed,OUTPUT);

  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  digitalWrite(stickGreen,1);
  delay (500);
  digitalWrite(stickGreen,0);
  digitalWrite(stickRed,1);
  delay (500);
  digitalWrite(stickGreen,1);
  digitalWrite(stickRed,0);
  delay(500);
  digitalWrite(stickGreen,0);

}
 
void loop() {

  //Read stick position
  if (!digitalRead(stickLeft)){
    myData.direction=0;
    myData.duration=100;  
    Serial.println(" LEFT  ");
    digitalWrite(stickRed,1);
    digitalWrite(stickGreen,0);
  }
  else if(!digitalRead(stickRight)){
    myData.direction=1;
    myData.duration=100;
    Serial.println("  RIGHT  "); 
    digitalWrite(stickRed,0);
    digitalWrite(stickGreen,1);    
  }
  else {
    myData.direction=2;
    myData.duration=100;
    digitalWrite(stickRed,1);
    digitalWrite(stickGreen,1);
  }
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }
  delay(250);
}