#include <Arduino.h>
#include <BLEMidi.h>
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>


//Adafruit_MPU6050 mpu;

int mod = 0;
word premod = 0;
int pitch = 0;
int pitchA = 0;
int pitchB = 0;
word prepitch = 0;

unsigned long lastFlip = 0;

bool state = 0;
bool ran = 0;

float semitonesSend;
float rangeSend = 82;
float devider;
float centerer;

#define thumb 27
#define direct 12

void setup() {
  //Serial.begin(115200);
  //float pitchF = ((float)pitch / 4095.75) - 2.00;
  centerer = rangeSend / 2;
  devider = 16383 / rangeSend;
  //Serial.println("Initializing bluetooth");
  BLEMidiServer.begin("OndoRing");
  //Serial.println("Waiting for connections...");
  //BLEMidiServer.enableDebugging();  // Uncomment if you want to see some debugging output from the library (not much for the server class...)

  pinMode(thumb, INPUT_PULLUP);
  pinMode(direct, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(thumb), stateflip, RISING);
/*
  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }*/
  /*mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);*/
}

void stateflip() {

  //Serial.println("flipping");
  if (lastFlip + 800 < millis()) {
    lastFlip = millis();
    state = !state;

    //Serial.println(state);
  }
}

void getValues() {

  //sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp);

  //mod calculations
  //int accelX = a.acceleration.x * 100;
  //mod = map(accelX, -1000, 0, 127 + analogRead(35) / 32, 0);
  /*if (mod > 127) {
    mod = 127;
  }
  else if (mod < 0) {
    mod = 0;
  }*/

  //pitch calculations from acceleration
  //int accelY = a.acceleration.y * 100;
  //pitch = map(accelY, - 1000, 1000, 0, analogRead(32) * 4);

  //pitch calculations from gyroposition
  //int gyroY = g.gyro.y * 1000;

  //pitchB = map(gyroY, - 1000, 1000, -1 * (analogRead(33) * 4), analogRead(33) * 4);

  //final pitch calculations
  //pitch = (pitchA + pitchB);
  /*if (pitch > 16383) {
    pitch = 16383;
  }
  else if (pitch < 0) {
    pitch = 0;
  }*/
  /*
    Serial.print ("state  ");
    Serial.print (state);
  */
  /*
    Serial.print ("  35  ");
    Serial.print (analogRead(35) / 32);
    Serial.print ("  32  ");
    Serial.print (analogRead(32) * 4);
    Serial.print ("  33  ");
    Serial.print (analogRead(33) * 4);*/
  /*
    Serial.print ("  accelX  ");
    Serial.print (accelX);
    Serial.print ("  accelY  ");
    Serial.print (accelY);
    Serial.print ("  gyroY  ");
    Serial.print (gyroY);
  */
  /* Serial.print ("  pitchA  ");
    Serial.print (pitchA);
    Serial.print (" pitchB  ");
    Serial.print (pitchB);

    Serial.print ("  mod  ");
    Serial.print (mod);
    Serial.print (" pitch  ");
    Serial.println (pitch);*/
}
void getValues2() {
  /*Serial.print ("  36  ");
    Serial.print (analogRead(36));
    Serial.print ("  39  ");
    Serial.print (analogRead(39));
    Serial.print ("  34  ");
    Serial.println (analogRead(34));*/
  //1800-1400
  mod = map(digitalRead(direct) * analogRead(39), 1800, 1400, 0, 127 + analogRead(35) / 68);
  /*
    mod = ((1700 - (analogRead(39) - (1400 + analogRead(35) / 500))) / 2.4);
    Serial.print (analogRead(35));
    Serial.print ("  ");
    Serial.print (analogRead(39));
    Serial.print ("  ");
    Serial.print (analogRead(35)/1300);
    Serial.print ("  ");
    //Serial.print (((1800 - analogRead(39)) / (float)((analogRead(35)/1900))+1));
    Serial.print (map(analogRead(39), 1800, 1400, 0, 127 + analogRead(35)/68.25));
    Serial.println ("  ");
    //pitch = (analogRead(36) - 1800);*/

  pitch = map(digitalRead(direct) * analogRead(36), 1400, 2200, 8192 - analogRead(33) * 2, 8192 + analogRead(33) * 2);
}

void loop() {
  //getValues();
  getValues2();
  if (BLEMidiServer.isConnected() ) {            // If we've got a connection, we send an A4 during one second, at full velocity (127)
    if (state == 1) {

      if (premod != mod) {
        BLEMidiServer.controlChange(0, 1, mod);
        premod = mod;
      }

      if (prepitch != pitch) {
        //BLEMidiServer.controlChange(0, 118, pitch / 128);
        //BLEMidiServer.controlChange(0, 119, pitch % 128);
        //float pitchF = ((float)pitch / 4095.75) - 2.00;
        float pitchF = ((float)pitch / devider) - centerer;

        BLEMidiServer.pitchBend(0, pitchF, rangeSend);
        /*Serial.print (pitch);
        Serial.print ("  ");
        Serial.println (pitchF);*/
        prepitch = pitch;       // Then we make a delay of one second before returning to the beginning of the loop
      }
      ran = 1;
    }
    else if (ran == 1) {
      BLEMidiServer.pitchBend(0, (float)0);
      ran = 0;
    }

  }
  else {
    delayMicroseconds(1);   // we feed the ESP32 watchdog when there is no connection
  }
}
