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



const int PIN_SUSTAIN_PEDAL = GPIO_NUM_23;     // Pin para el pedal de sustain
const int PIN_OTA = GPIO_NUM_15;              // Pin para activar el OTA
//const int PIN_SETUP = GPIO_NUM_2;            // Pin para entrar al Setup
const int PIN_NO_DETECTAR_VELOCIDAD = GPIO_NUM_4;      // Pin que determina si se usa la función findNotes3NoVelocity(i) o findNotes3(i);

const int PIN_OCT_SEL_0 = GPIO_NUM_19;
const int PIN_OCT_SEL_1 = GPIO_NUM_18;
const int PIN_OCT_SEL_2 = GPIO_NUM_5;

// PARA EL PEDAL DE SUSTAIN
int sustainCurrentState = 1;    // =1 por el INPUT_PULLUP
int sustainlastState = 1;

// JSON FILE NAME
#define JSON_FILE_NAME "/config.json" 

int jsonVersion = -99;      // Se debe actualizar en el config.json
int softwareVersion = 2;
int hardWareVersion = 2;
int sensoresActivos = 1;

