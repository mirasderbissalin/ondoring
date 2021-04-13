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

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x8D, 0xD4, 0x28};
unsigned long lastFlip;
bool state;


// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int a;
  byte b;
} struct_message;

// Create a struct_message called myData
struct_message myData;

#define thumb 27
#define direct 12
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  //Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //Serial.println("Failed to add peer");
    return;
  }
  pinMode(thumb, INPUT_PULLUP);
  pinMode(direct, INPUT_PULLUP);
  pinMode(2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(thumb), stateflip, RISING);
}
void stateflip() {

  //Serial.println("flipping");
  if (lastFlip + 800 < millis()) {
    lastFlip = millis();
    state = !state;

    //digitalWrite(2, state);
    //Serial.println(state);
  }
}
void loop() {
  // Set values to send

  myData.a = 8192 * !state + state * ((16383 - analogRead(36) - 6392) * !digitalRead(direct) + (analogRead(36) + 6392) * digitalRead(direct)); //2200
  int preMod = myData.b * !state + state * ((127 - map(analogRead(39), 2200, 1800, 0, 127)) * !digitalRead(direct) + map(analogRead(39), 1800, 1400, 0, 127) * digitalRead(direct));

  if (preMod > 127) {
    myData.b = 127;
  }
  else if (preMod < 0) {
    myData.b = 0;
  }
  else {
    myData.b = preMod;
  }



  /*Serial.print(digitalRead(direct));
    Serial.print("  ");
    Serial.print(analogRead(36));
    Serial.print("  ");
    Serial.print(analogRead(39));
    Serial.print("  ");
    Serial.print(myData.a);
    Serial.print("  ");
    Serial.print(myData.b);
    Serial.println();*/
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  /*if (result == ESP_OK) {
    Serial.println("Sent with success");
    }
    else {
    Serial.println("Error sending the data");
    }*/
  //delay(2000);
}
