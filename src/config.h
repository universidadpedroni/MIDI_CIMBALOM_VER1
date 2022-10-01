// CONFIGURATION FILE
#include <Arduino.h>

 

const long BAUDRATE = 115200;       // Baudrte for serial com
const long MIDI_BAUDRATE = 31250;    // Baudrate for MIDI port
#define SerialMidi Serial2

// ledBlink constants
const long LED_INTERVAL = 500;
const long LED_INTERVAL_OTA = 250;
const long LED_INTERVAL_SETUP = 100;
// TODO Verificar la operación de este pin cuando se agregue la comunicación I2C
const int PIN_SPEED = GPIO_NUM_21;            // pin para medir la velocidad de las rutinas.
const int PIN_LED = GPIO_NUM_23;

const int PIN_SUSTAIN_PEDAL = GPIO_NUM_5;     // pin para el pedal de sustain
const int PIN_OTA = GPIO_NUM_18;              // Pin para activar el OTA
const int PIN_SETUP = GPIO_NUM_19;            // Pin para entrar al Setup

// PARA EL PEDAL DE SUSTAIN
int sustainCurrentState = 1;    // =1 por el INPUT_PULLUP
int sustainlastState = 1;

// JSON FILE NAME
#define JSON_FILE_NAME "/config.json" 

