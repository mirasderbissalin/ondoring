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



#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//RF24 radio(9, 10); // CE, CSN
RF24 radio(PB0, PA4); // CE, CSN on Blue Pill
const uint64_t address = 0xF0F0F0F0E1LL;
unsigned long lastFlip;
bool state;
bool preState;
byte chack;
int preMod;

typedef struct struct_message {
  int a;
  byte b;
} struct_message;
struct_message myData;

#define thumb PB12
#define direct PB10
#define led PC13
void stateflip() {

  //Serial.println("flipping");
  if (lastFlip + 100 < millis()) {
    lastFlip = millis();
    state = !state;

    digitalWrite(led, !state);
    //Serial.println(state);
  }
}
void setup() {

  //Serial.begin(115200);
  //Serial.println("w");

  analogReadResolution(12);
  pinMode(thumb, INPUT_PULLUP);
  pinMode(direct, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  digitalWrite(led, !state);
  attachInterrupt(digitalPinToInterrupt(thumb), stateflip, RISING);

  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MAX);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(0);
  radio.stopListening();          //This sets the module as transmitter
}

void loop()
{
  if (state == 1)
  {

    preState = 1;
    myData.a = digitalRead(direct) * (4095 - analogRead(PA0)) + !digitalRead(direct) * analogRead(PA0);
    preMod = !digitalRead(direct) * (4095 - analogRead(PA1)) + digitalRead(direct) * analogRead(PA1) - 2056;
    if (preMod < 0) {

      myData.b = 0;
    }
    else {
      myData.b = map (preMod, 0, 360, 0, 200);
    }

    /*
    Serial.print(myData.a);
    Serial.print("    ");
    Serial.print(preMod);
    Serial.print("    ");
    Serial.print(myData.b);
    Serial.println();*/
    radio.write(&myData, sizeof(myData));

    /*
      radio.write(&chack, sizeof(chack));
      Serial.println(chack);
      chack++;*/
  }
  else if (preState == 1) {
    myData.a = 500;
    radio.write(&myData, sizeof(myData));
    preState = 0;
  }
}
