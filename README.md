# ondoring
Wearable MIDI device to control music equipment 

Files come in pairs. 6 files for 3 versions - BLE,  ESPNOW and NRF24-based
In that pair of files onw is for reciever hub and another for wearable transmitter.

Every hub version has 2 CV outputs, wire your opamps according to voltages boards produce.
Every hub version has Hardware MIDI outputs. Only NRF24-based version has USB-MIDI implemented. In theory swapping ESP32 for ESP32-S2 in other two options will let to implement USB-MIDI.

Hub interface  hardware needed - rotary encoder (4 pulses per step), I2C screen (128*96), 5DIN ports, 3.5 mm female jacks. 
Other hardware include opamps (for CV voltages), optocouplers, diodes (for MIDI), resistors, linear potentiometers 

Wearable uses ADXL335 analog axxelerometer

Deduce wiring from the code provided. Hit me to produce a diagram, if struggling. If produced yourself - please share
