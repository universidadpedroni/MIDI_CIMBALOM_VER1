// CONFIGURATION FILE
#include <Arduino.h>

#define DEBUG true

const long BAUDRATE = 115200;       // Baudrte for serial com
const long MIDI_BAUDRATE = 31250;    // Baudrate for MIDI port
#define SerialMidi Serial2

#define FIRMWARE F("MIDI Cimbalom Ver 1.0\n")
#define CR F("\n")
#define TAB F("\t")
#define COMMA F(",")

// ledBlink constants
const long LED_INTERVAL = 250;

const int PIN_SPEED = GPIO_NUM_23;            // pin para medir la velocidad de las rutinas
const int PIN_LED = GPIO_NUM_22;
const int SERIAL_MIDI_TX = GPIO_NUM_17;        // pines para la transmisicón del protocolo MIDI.
const int SERIAL_MIDI_RX = GPIO_NUM_16;

const int PIN_SUSTAIN_PEDAL = GPIO_NUM_5;     // pin para el pedal de sustain



// Midi constants
// PARA LOS PIEZOS
//const int SENSOR_NUMBER_PER_MULTIPLEXOR = 8;     // Amount of sensors per Multiplexer
//const int MULTIPLEXORS = 2;                   // Amount of multiplexors per controller
const int SENSOR_TOTAL_NUMBER = 4; //MULTIPLEXORS * SENSOR_NUMBER_PER_MULTIPLEXOR;
long THRESHOLD[SENSOR_TOTAL_NUMBER] = {30}; //(int) 0.025 * 1023 / 5; Dejo previsto que sea ajustable por sensor
long currentValue[SENSOR_TOTAL_NUMBER] = {0};
long lastValue[SENSOR_TOTAL_NUMBER] = {0};
unsigned long lastTimeStateChange[SENSOR_TOTAL_NUMBER] = {0};
unsigned long DELTA_TIME[SENSOR_TOTAL_NUMBER] = {100};     // [mseg] // Puede ser más. Las notas duran alrededor de 100ms


// notes to play, corresponding to the 16 sensors:
char control[16] = { 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,72, 73, 74, 75};

// PARA EL PEDAL DE SUSTAIN
int sustainCurrentState = 1;    // =1 por el INPUT_PULLUP
int sustainlastState = 1;

// GENERALES
//https://anotherproducer.com/online-tools-for-musicians/midi-cc-list/

const int MIDI_CHANNEL = 5;
const int NOTE_ON = 0x90;
const int NOTE_OFF = 0x80;
const int CONTROL_CHANGE = 0xB0;
const int SUSTAIN = 0x40;       // sustain pedal
// const int AFTERTOUCH = 0xA0;

