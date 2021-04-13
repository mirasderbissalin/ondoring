/*
  Code based on :
  https://github.com/neilbags/arduino-esp32-BLE-MIDI/blob/master/BLE_MIDI.ino (write midi messages)
  https://gist.github.com/johnty/cbfa66c3369a692410f3493c20b2b3e2 (read and write)
  https://circuitdigest.com/microcontroller-projects/esp32-ble-client-connecting-to-fitness-band-to-trigger-light (initiate the connection to another bluetooth device)
  and Arduino BLE_Client example sketch for ESP32 boards
*/

#include <Arduino.h>
#include <BLEMidi.h>
#include <MIDI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>
#include <EEPROM.h>
#define EEPROM_SIZE 4
//#include "SSD1306.h"
ESP32Encoder encoder;

long oldPosition  = 0;
int change = 0;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define midiH MIDI


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

MIDI_CREATE_INSTANCE(HardwareSerial, Serial, midiH);

int8_t mod = 0;
word pit1 = 0;
word pit2 = 0;
int pitch = 0;

word premod = 0;
word prepitch = 0;

int8_t chnl1 = 0;
int8_t chnl2 = 0;

int8_t CC1 = 0;
int8_t CC2 = 0;

byte knobM = 0;
unsigned long lastFlip = 0;

byte channel;
bool connection = 0;
word dog = 1;

void connected();
void disconnected();

void onPitchBendCallback(uint8_t channel, uint16_t value)
{
  pitch = value;
  //Serial.print("pitch  ");
  //Serial.println(pitch);

  //delayMicroseconds(dog);
  delay(dog);
}

void onControlChange(uint8_t channel, uint8_t controller, uint8_t value)
{
  //Serial.printf("Received control change : channel %d, controller %d, value %d\n", channel, controller, value);
  /*if (controller == 1) {
    mod = value;
    }*/
  mod = value;
  //delayMicroseconds(dog);
  delay(dog);
  //Serial.print("mod  ");
  //Serial.println(mod);
  /*  else if (controller == 118) {
      pit1 = value;
    }
    else if (controller == 119) {
      pit2 = value;
    }*/
}

void screen() {

  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(12, 0);
  if (CC1 == -1) {

    display.print(F("Pitch"));
  }
  else {

    display.print(CC1 + 1);
  }
  display.print(F("   "));
  display.setCursor(88, 0);
  display.println(CC2 + 1);

  display.setCursor(12, 24);
  display.print(chnl1 + 1);
  display.print(F("   "));
  display.setCursor(88, 24);
  display.println(chnl2 + 1);
  if (knobM == 0) {

    display.setCursor(0, 0);
  }
  else if (knobM == 1) {

    display.setCursor(76, 0);
  }
  else if (knobM == 2) {

    display.setCursor(0, 24);
  }
  else if (knobM == 3) {

    display.setCursor(76, 24);
  }

  display.print(F(">"));

  if (connection == 1) {

    display.setCursor(12, 46);
    display.print(F("OndoRing"));
  }
  display.display();
}

void connected()
{
  //Serial.println("Connected");
}

void setup() {
  //Serial.begin(115200);
  midiH.begin(MIDI_CHANNEL_OMNI);
  BLEMidiClient.begin("OndoStation");
  //BLEMidiServer.setNoteOnCallback(onNoteOn);
  //BLEMidiServer.setNoteOffCallback(onNoteOff);
  BLEMidiClient.setControlChangeCallback(onControlChange);
  BLEMidiClient.setPitchBendCallback(onPitchBendCallback);
  //BLEMidiServer.enableDebugging();
  Wire.begin(5, 4);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    //Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  EEPROM.begin(EEPROM_SIZE);

  CC1 = EEPROM.read(0);
  CC2 = EEPROM.read(1);
  chnl1 = EEPROM.read(2);
  chnl2 = EEPROM.read(3);


  delay(10);
  pinMode(16, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(16), handleKnob, RISING);

  //display.clearDisplay();

  encoder.attachHalfQuad(13, 15);
}

void hardwarePitch() {

  dacWrite(25, pitch / 65 + 3);

  if (CC1 == -1) {
    midiH.sendPitchBend(pitch - 8192, chnl1 + 1);
  }
  else {
    midiH.sendControlChange(CC1 + 1, pitch / 128, chnl1);
  }

  //midiH.sendPitchBend(pitch, chnl1);
}

void hardwareMod() {
  byte dacRW = mod * 2;
  if (dacRW > 255) {
    dacRW = 255;
  }
  dacWrite(26, dacRW);

  midiH.sendControlChange(CC2 + 1, mod, chnl2);
}

void passThr() {

  if (midiH.read())
  {
    // Thru on A has already pushed the input message to out A.
    // Forward the message to out B as well.
    midiH.send(midiH.getType(),
               midiH.getData1(),
               midiH.getData2(),
               midiH.getChannel());
  }
}
void handleKnob() {
  //Serial.print("Knob    ");
  if (lastFlip + 200 < millis()) {
    //Serial.println("In");
    knobM++;
    if  (knobM > 3) {
      knobM = 0;
    }
  }
}

void coder() {
  /*Serial.println("coder");
    Serial.println(String((int32_t)encoder.getCount()));*/
  long newPosition = encoder.getCount() / 2;
  if (newPosition != oldPosition) {
    change = newPosition - oldPosition;
    oldPosition = newPosition;
    /*Serial.print("change  ");
      Serial.println(change);
      Serial.print("knobM  ");
      Serial.println(knobM);*/
    if (knobM == 0) {
      CC1 = CC1 + change;
      if (CC1 > 118) {
        CC1 = -1;
      }
      else if (CC1 < -1) {
        CC1 = 118;
      }
      EEPROM.write(0, CC1);
      EEPROM.commit();
    }
    else if (knobM == 1) {
      CC2 = CC2 + change;
      if (CC2 > 118) {
        CC2 = 0;
      }
      else if (CC2 < 0) {
        CC2 = 118;
      }
      EEPROM.write(1, CC2);
      EEPROM.commit();
    }
    else if (knobM == 2) {
      chnl1 = chnl1 + change;
      if (chnl1 > 15) {
        chnl1 = 0;
      }
      else if (chnl1 < 0) {
        chnl1 = 15;
      }
      EEPROM.write(2, chnl1);
      EEPROM.commit();
    }
    else if (knobM == 3) {
      chnl2 = chnl2 + change;
      if (chnl2 > 15) {
        chnl2 = 0;
      }
      else if (chnl2 < 0) {
        chnl2 = 15;
      }
      EEPROM.write(3, chnl2);
      EEPROM.commit();
    }
  }
}

void loop() {


  if (!BLEMidiClient.isConnected()) {
    connection = 0;
    //Serial.println("Connecting");
    // If we are not already connected, we try te connect to the first BLE Midi device we find
    int nDevices = BLEMidiClient.scan();
    if (nDevices > 0) {
      //Serial.println("Connection found");
      if (BLEMidiClient.connect(0)) {
        //Serial.println("Connection established");

        connection = 1;
      }
      else {
        //Serial.println("Connection failed");
        connection = 0;
        delay(1);    // We wait 3s before attempting a new connection
      }
    }
  }
  /*else {
    pitch = pit1 * 128 + pit2;

    if (BLEMidiServer.isConnected()) {
      if (prepitch != pitch) {
        hardwarePitch();
        prepitch = pitch;
      }
      if (premod != mod) {
        hardwareMod();
        premod = mod;
      }
    }
    }*/
  //Serial.print("read   ");
  //Serial.println(digitalRead(16));
  if (prepitch != pitch) {
    hardwarePitch();
    prepitch = pitch;
  }
  if (premod != mod) {
    hardwareMod();
    premod = mod;
  }


  coder();
  passThr();
  screen();
  /*Serial.print("pitch  ");
    Serial.println(pitch);
    Serial.print("mod  ");
    Serial.println(mod);*/
  //delayMicroseconds(dog);
  delay(dog);
}
