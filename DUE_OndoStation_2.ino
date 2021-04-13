/*
   NRF24L01       Arduino_ Uno    Arduino_Mega    Blue_Pill(stm32f01C)
   __________________________________________________________________________
   VCC        |       3.3v      |     3.3v      |     3.3v
   GND        |       GND       |     GND       |      GND
   CSN        |   Pin10 SPI/SS  | Pin10 SPI/SS  |     A4 NSS1 (PA4) 3.3v
   CE         |   Pin9          | Pin9          |     B0 digital (PB0) 3.3v
   SCK        |   Pin13         | Pin52         |     A5 SCK1   (PA5) 3.3v
   MOSI       |   Pin11         | Pin51         |     A7 MOSI1  (PA7) 3.3v
   MISO       |   Pin12         | Pin50         |     A6 MISO1  (PA6) 3.3v


*/

//
////
//
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#include <Encoder.h>
#include <DueFlashStorage.h>
#include <MIDI.h>
#include "MIDIUSB.h"
DueFlashStorage dueFlashStorage;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Encoder myEnc(11, 10);
RF24 radio(39, 37); // CE, CSN
#define midiH MIDI
#define midiH2 MIDI2
#define midiH3 MIDI3
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiH);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, midiH2);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, midiH3);


//RF24 radio(PB0, PA4); // CE, CSN on Blue Pill
const uint64_t address = 0xF0F0F0F0E1LL;
boolean button_state = 0;
//int led_pin = PB6;  //set the led indicator to pin B6
typedef struct struct_message {
  int a;
  byte b;
} struct_message;
struct_message myData;


int mod = 0;
int pitch = 0;
int dacPitch = 0;

word premod = 0;
word prepitch = 0;

int8_t chnl1 = 1;
int8_t chnl2 = 1;

int8_t CC1 = 1;
int8_t CC2 = 1;

byte knobM = 0;
unsigned long lastFlip = 0;
long oldPosition  = 0;
int change = 0;

#define knob 9
void screen() {

  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(12, 0);
  if (CC1 == 0) {

    display.print(F("Pitch"));
  }
  else {

    display.print(CC1);
  }
  display.print(F("   "));
  display.setCursor(88, 0);
  display.println(CC2);

  display.setCursor(12, 24);
  display.print(chnl1);
  display.print(F("   "));
  display.setCursor(88, 24);
  display.println(chnl2);


  display.setCursor(12, 48);
  display.print(map(analogRead(A0), 0, 4095, 0, 100));
  display.print(F("   "));
  display.setCursor(88, 48);
  display.print(map(analogRead(A1), 0, 4095, 0, 100));

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


  display.display();
}

void setup() {
  //pinMode(led_pin, OUTPUT);
  //Serial.begin(115200);
  //Serial.println("go");
  radio.begin();
  //Serial.print("ADDRESS :");
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(0);
  radio.startListening();              //This sets the module as receiver


  midiH.begin(MIDI_CHANNEL_OMNI);
  midiH2.begin(MIDI_CHANNEL_OMNI);
  midiH3.begin(MIDI_CHANNEL_OMNI);

  pinMode(knob, INPUT_PULLUP);

  /* pinMode(10, INPUT);
    pinMode(11, INPUT);*/
  attachInterrupt(digitalPinToInterrupt(knob), handleKnob, RISING);
  Wire.begin();
  //if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    //Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  /*
    CC1 = -1;
    Serial.println(CC1);
    CC2 = 0;
    chnl1 = 0;
    chnl2 = 0;*/


  CC1 = dueFlashStorage.read(4);
  CC2 = dueFlashStorage.read(1);
  chnl1 = dueFlashStorage.read(2);
  chnl2 = dueFlashStorage.read(3);

  analogReadResolution(12);
  analogWriteResolution(12);
}
void loop()
{
  getRadio();

  if (prepitch != pitch) {
    hardwarePitch();
    softwarePitch();
    prepitch = pitch;
  }
  if (premod != mod) {

    hardwareMod();
    softwareMod ();
    premod = mod;
  }

  passThr();
  coder();
  screen();
}

void softwarePitch() {
  if (CC1 == 0) {
    /*Serial.print ("pitch");
    Serial.print ("  ");
    Serial.println (pitch);*/
    pitchBendChange(chnl1 - 1, pitch);
  }
  else {
    controlChangeU(chnl1 - 1, CC1, map (pitch, 0, 16383, 0, 127));

  }
  MidiUSB.flush();
}

void softwareMod () {

  controlChangeU(chnl2 - 1, CC2, mod);
  MidiUSB.flush();
}

void hardwarePitch() {
  if (pitch > 16383) {
     dacPitch = 16383;
  }
  else if ( pitch < 0) {
     dacPitch = 0;
  }
  else {

     dacPitch = pitch;
  }
  analogWrite(DAC0, map (dacPitch, 0, 16383, 0, 4095));
  //dacWrite(25, pitch / 65 + 3);

  /*Serial.print (dacPitch);
  Serial.print ("  ");
  Serial.print ( map (dacPitch, 0, 16383, 0, 4095));
  Serial.println ();*/
  if (CC1 == -1) {
    midiH.sendPitchBend(pitch - 8192, chnl1 + 1);
  }
  else {
    midiH.sendControlChange(CC1 + 1, pitch / 128, chnl1);
  }

  //midiH.sendPitchBend(pitch, chnl1);
}

void hardwareMod() {
  /*byte dacRW = mod * 2;
    if (dacRW > 255) {
    dacRW = 255;
    }*/
  //dacWrite(26, dacRW);

  analogWrite(DAC1, map (mod, 0, 127, 0, 4095));

  midiH.sendControlChange(CC2 + 1, mod, chnl2);
} void passThr() {

  if (midiH.read())
  {
    // Thru on A has already pushed the input message to out A.
    // Forward the message to out B as well.
    midiH.send(midiH.getType(),
               midiH.getData1(),
               midiH.getData2(),
               midiH.getChannel());
    //midiEventPacket_t event = {0x0E, 0xE0 | channel, lowValue, highValue};

  }
  if (midiH2.read())
  {
    // Thru on A has already pushed the input message to out A.
    // Forward the message to out B as well.
    midiH.send(midiH2.getType(),
               midiH2.getData1(),
               midiH2.getData2(),
               midiH2.getChannel());

  }
  if (midiH3.read())
  {
    // Thru on A has already pushed the input message to out A.
    // Forward the message to out B as well.
    midiH.send(midiH3.getType(),
               midiH3.getData1(),
               midiH3.getData2(),
               midiH3.getChannel());

  }
}
void getRadio () {

  //Serial.println("in radio");
  if (radio.available())              //Looking for the data.
  {
    //Serial.println("Radio is sniffing");
    //Saving the incoming data
    //Reading the data


    radio.read(&myData, sizeof(myData));    //Reading the data

    /*
        Serial.print (myData.a);
        Serial.print ("  ");
        Serial.print (myData.b);
        Serial.println ();*/
    //-1650-2047-2450
    pitch = map (myData.a, 1647, 2447, 8192 - 2 * analogRead (0), 8192 + 2 * analogRead (0));
    mod = map (myData.b, 0, 200 - map(analogRead (1), 0, 4095, 0, 50 ), 0, 127);
    if (mod > 127) {
      mod = 127;
    }
    else if (mod < 0) {
      mod = 0;
    }

    /*Serial.print (analogRead (0));
      Serial.print ("  ");
      Serial.print (analogRead (1));
      Serial.print ("  ");
      Serial.print (map(analogRead (1), 0, 4095, 0, 50 ));
      Serial.print ("  ");*/
    /*
      Serial.print (pitch);
      Serial.print ("  ");
      Serial.print (mod);
      Serial.println ();
    */
    /*byte chack;
      radio.read(&chack, sizeof(chack));
      Serial.print (chack);
      Serial.println ();
    */


  }
}
void coder() {
  long newPosition = myEnc.read() / 4;
  if (newPosition != oldPosition) {
    change = newPosition - oldPosition;
    oldPosition = newPosition;
    /*Serial.print("change  ");
      Serial.println(change);
      Serial.print("knobM  ");
      Serial.println(knobM);*/
    if (knobM == 0) {
      CC1 = CC1 + change;
      if (CC1 > 119) {
        CC1 = 0;
      }
      else if (CC1 < 0) {
        CC1 = 119;
      }
      dueFlashStorage.write(4, CC1);
      //Serial.println (CC1);
      //EEPROM.write(0, CC1);      EEPROM.commit();
    }
    else if (knobM == 1) {
      CC2 = CC2 + change;
      if (CC2 > 119) {
        CC2 = 1;
      }
      else if (CC2 < 1) {
        CC2 = 119;
      }
      dueFlashStorage.write(1, CC2);
      //EEPROM.write(1, CC2);      EEPROM.commit();
    }
    else if (knobM == 2) {
      chnl1 = chnl1 + change;
      if (chnl1 > 16) {
        chnl1 = 1;
      }
      else if (chnl1 < 1) {
        chnl1 = 16;
      }
      dueFlashStorage.write(2, chnl1);
      //EEPROM.write(2, chnl1);      EEPROM.commit();
    }
    else if (knobM == 3) {
      chnl2 = chnl2 + change;
      if (chnl2 > 16) {
        chnl2 = 1;
      }
      else if (chnl2 < 1) {
        chnl2 = 16;
      }
      dueFlashStorage.write(3, chnl2);
      //EEPROM.write(3, chnl2);      EEPROM.commit();
    }
  }
}
void handleKnob() {
  //Serial.print("Knob    ");
  if (lastFlip + 200 < millis()) {
    knobM++;
    if  (knobM > 3) {
      knobM = 0;
    }
    lastFlip = millis();
    //Serial.println(knobM);
  }
}
void controlChangeU(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}
void pitchBendChange(byte channel, int value) {
  byte lowValue = value & 0x7F;
  byte highValue = value >> 7;
  midiEventPacket_t event = {0x0E, 0xE0 | channel, lowValue, highValue};
  MidiUSB.sendMIDI(event);
}
