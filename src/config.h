// CONFIGURATION FILE
#include <Arduino.h>

// *** CONFIGURATION OPTIONS *** //
const bool DEBUG =  true;
const bool  PLOT_SIGNALS =  false;
 int plotSensor = 3;
const bool TEST_MIDI = false;
// **************************** //
 

const long BAUDRATE = 115200;       // Baudrte for serial com
const long MIDI_BAUDRATE = 31250;    // Baudrate for MIDI port
#define SerialMidi Serial2

// ledBlink constants
const long LED_INTERVAL = 250;

const int PIN_SPEED = GPIO_NUM_23;            // pin para medir la velocidad de las rutinas
const int PIN_LED = GPIO_NUM_22;

const int PIN_SUSTAIN_PEDAL = GPIO_NUM_5;     // pin para el pedal de sustain

// PARA EL PEDAL DE SUSTAIN
int sustainCurrentState = 1;    // =1 por el INPUT_PULLUP
int sustainlastState = 1;

// JSON FILE NAME
#define JSON_FILE_NAME "/config.json" 

