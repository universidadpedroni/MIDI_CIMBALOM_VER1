#include <Arduino.h>


// GENERALES
//https://anotherproducer.com/online-tools-for-musicians/midi-cc-list/

const int MIDI_CHANNEL = 1;
const int NOTE_ON = 0x90;
const int NOTE_OFF = 0x80;
const int CONTROL_CHANGE = 0xB0;
const int SUSTAIN = 0x40;       // sustain pedal
// const int AFTERTOUCH = 0xA0;


const int NUM_SENSORES = 10; // Esta es la variable que manda con cuántos sensores se trabaja. 10 sensores, 700uSeg

const int CANALES_ADC[] = {GPIO_NUM_4,      // ADC2_0
                           GPIO_NUM_2,      // ADC2_2   //TODO: Change to ADC_5
                           GPIO_NUM_15,     // ADC2_3
                           GPIO_NUM_13,     // ADC2_4
                           GPIO_NUM_34,     // ADC1_6
                           GPIO_NUM_35,     // ADC1_7
                           GPIO_NUM_32,     // ADC1_4
                           GPIO_NUM_33,     // ADC1_5
                           GPIO_NUM_25,     // ADC2_8
                           GPIO_NUM_26};    // ADC2_9


// notes to play, corresponding to the 4 sensors:
const int CONTROL[] = { 36, 38, 40, 43,
                        45, 47, 48, 50,
                        52, 55}; 
const char *NOTAS[] = {"C2", "D2", "E2", "G2",
                       "A2", "B2", "C3", "D3",
                       "E3", "G3"}; 


  
float Threshold_ON[NUM_SENSORES] = {12000, 10000, 15000, 10000,
                                    4000, 4000, 4000, 5000,
                                    12000, 12000};

float Threshold_OFF[NUM_SENSORES] = {8000, 7000, 13000, 8000,
                                     3000, 3000, 3000, 4000,
                                     8000, 8000};

const float  MAX_VELOCITY[NUM_SENSORES] = {17000, 17000, 17000, 17000,
                                           17000, 17000, 17000, 17000,
                                           17000, 17000 }; // Maximum measured velocity (17.000), usada solo en finNotes3()


// TODO: Move all this to json
int noteState[NUM_SENSORES] = {NOTE_OFF, NOTE_OFF, NOTE_OFF, NOTE_OFF,
                               NOTE_OFF, NOTE_OFF, NOTE_OFF, NOTE_OFF,
                               NOTE_OFF, NOTE_OFF};
                               
bool notaEnviada[NUM_SENSORES] = {false, false, false, false,
                                  false, false, false, false,
                                  false, false};


float amplik[NUM_SENSORES][3] = {0}; // Sensor reading
float movingAvVelocityk[NUM_SENSORES] = {0};  // Sensor filtering.
unsigned long detectionTime[NUM_SENSORES] = {0};
const unsigned long DETECTION_TIME = 300;
float maxVelocity[NUM_SENSORES] = {0};
unsigned long MAX_LATENCY = 150; // Must be lower than DETECTION_TIME
