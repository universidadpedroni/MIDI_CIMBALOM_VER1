// CONFIGURATION FILE
#include <Arduino.h>

const long BAUDRATE = 115200;       // Baudrte for serial com
const long MIDI_BAUDRATE = 31250;    // Baudrate for MIDI port


#define FIRMWARE F("MIDI Cimbalom Ver 1.0\n")
#define CR F("\n")
#define TAB F("\t")
#define COMMA F(",")

// ledBlink constants
const long LED_INTERVAL = 250;

// PINOUT!
const int MUX_PIN_0 = 2;            // pines para direccionar los multiplexores
const int MUX_PIN_1 = 3;
const int MUX_PIN_2 = 4;
const int PIN_SPEED = 5;            // pin para medir la velocidad de las rutinas
const int SERIAL_MIDI_TX = 6;       // pines para la transmisicón del protocolo MIDI.
const int SERIAL_MIDI_RX = 7;

const int PIN_SUSTAIN_PEDAL = 8;     // pin para el pedal de sustain

// https://community.platformio.org/t/how-to-use-multiple-target-boards-with-same-project/16041/5
#ifdef ARDUINO_ARCH_AVR
const int MUX_0_ADC = A0;    // En A0 va colgado un multiplexor
const int MUX_1_ADC = A1;    // En A1 va colgado el otro multiplexor
#endif

#ifdef ARDUINO_ARCH_ESP32
//https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html
const int MUX_0_ADC = GPIO_NUM_32;      // ADC1
const int MUX_1_ADC = GPIO_NUM_0;       // ADC2
#endif

// Midi constants
// PARA LOS PIEZOS
const int SENSOR_NUMBER_PER_MULTIPLEXOR = 8;     // Amount of sensors per Multiplexer
const int MULTIPLEXORS = 2;                   // Amount of multiplexors per controller
const int SENSOR_NUMBER_TOTAL = MULTIPLEXORS * SENSOR_NUMBER_PER_MULTIPLEXOR;
int currentState[16] = {0};
int lastState[16] =  {0};    // the last state of the sensors
const int THRESHOLD = 0x90;
int lastLedVal = 0;
// notes to play, corresponding to the 16 sensors:
char control[16] = { 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71,72, 73, 74, 75};

// PARA EL PEDAL DE SUSTAIN
int sustainCurrentState = 1;    // =1 por el INPUT_PULLUP
int sustainlastState = 1;

// GENERALES
//https://anotherproducer.com/online-tools-for-musicians/midi-cc-list/

const int MIDI_CHANNEL = 1;
const int NOTE_ON = 0x90;
const int NOTE_OFF = 0x80;
const int SUSTAIN = 0x64;       // sustain pedal
// const int AFTERTOUCH = 0xA0;