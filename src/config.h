#ifndef CONFIG_H
#define CONFIG_H
// CONFIGURATION FILE
#include <Arduino.h>

int jsonVersion = -99;              // Versión del json. Se debe actualizar con el valor real.
const int SOFTWARE_VERSION = 3;     // Versión de software
const int HARDWARE_VERSION = 2;     // Versión del hardware.
int sensoresActivos = 1;            // Cantidad de sensores con los que trabaja el sistema. [1 -12]

const long BAUDRATE = 115200;       // Baudrte for serial com
const long MIDI_BAUDRATE = 31250;    // Baudrate for MIDI port
#define SerialMidi Serial2




// PARA EL PEDAL DE SUSTAIN
int sustainCurrentState = 1;    // =1 por el INPUT_PULLUP
int sustainlastState = 1;

// JSON FILE NAME
//#define JSON_FILE_NAME "/config.json" 
const char* JSON_FILE_NAME = "/config.json";

// Direcciones para I2C
int octava = 0;

#endif

