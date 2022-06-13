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
 
const int NUM_SENSORES = 4;


const int CANALES_ADC[] = {GPIO_NUM_4,      // ADC2_0
                           GPIO_NUM_2,      // ADC2_2
                           GPIO_NUM_15,     // ADC2_3
                           GPIO_NUM_13};    // ADC2_4

const float CANALES_GAIN[] ={1.0, 1.25, 1.0, 1.0};

// notes to play, corresponding to the 4 sensors:
const int CONTROL[] = { 60, 61, 62, 63}; //, 64, 65, 66, 67, 68, 69, 70, 71,72, 73, 74, 75};


const float MIDI_GAIN = 127.0 / 8000.0;
const int MIDI_MINIMUM_VELOCITY = 40;   // Debajo de este valor no envío la nota.
const int MAX_LATENCY = 200;
const float SAMPLETIME = 65e-6;     // La rutina findnotes2 demora 65useg. 
// TODO: revisar este número
const float DERIVATIVE_THRESHOLD_LOW = 10000;
const float DERIVATIVE_THRESHOLD_HIGH = 30000;
const unsigned long DETECTION_TIME = 300;