#include <Arduino.h>


// GENERALES
//https://anotherproducer.com/online-tools-for-musicians/midi-cc-list/
const int MIDI_CHANNEL_DEFAULT = 5; // Default MIDI CHANNEL
int MIDI_CHANNEL = MIDI_CHANNEL_DEFAULT;           
const int NOTE_ON = 0x90;
const int NOTE_OFF = 0x80;
const int CONTROL_CHANGE = 0xB0;
const int SUSTAIN = 0x40;       // sustain pedal
// const int AFTERTOUCH = 0xA0;

const int CANALES_ADC[] = { GPIO_NUM_13,    // ENV0
                            GPIO_NUM_12,    // ENV1
                            GPIO_NUM_14,    // ENV2
                            GPIO_NUM_27,    // ENV3
                            GPIO_NUM_26,    // ENV4
                            GPIO_NUM_25,    // ENV5
                            GPIO_NUM_33,    // ENV6
                            GPIO_NUM_32,    // ENV7
                            GPIO_NUM_35,    // ENV8 
                            GPIO_NUM_34,    // ENV9
                            GPIO_NUM_39,    // ENV10
                            GPIO_NUM_36,    // ENV11
                                
                        }; // 12 canales, 1 octava completa.
const int NUM_SENSORES = 12;

int noteState[NUM_SENSORES] = {NOTE_OFF, NOTE_OFF, NOTE_OFF, NOTE_OFF,
                               NOTE_OFF, NOTE_OFF, NOTE_OFF, NOTE_OFF,
                               NOTE_OFF, NOTE_OFF, NOTE_OFF, NOTE_OFF};
                               
bool notaEnviada[NUM_SENSORES] = {false, false, false, false,
                                  false, false, false, false,
                                  false, false, false, false};


float amplik[NUM_SENSORES][3] = {0}; // Sensor reading
float movingAvVelocityk[NUM_SENSORES] = {0};  // Sensor filtering.
unsigned long detectionTime[NUM_SENSORES] = {0};

float maxVelocity[NUM_SENSORES] = {0};
unsigned long MAX_LATENCY = 150; // Must be lower than DETECTION_TIME

int contadorMax[NUM_SENSORES] = {0};

// Constants with DEFAULT VALUES. All of this parameters are updated using a json file.

// notes to play, corresponding to all sensors: 
int CONTROL[NUM_SENSORES] = { 36, 38, 40, 43,
                              45, 47, 48, 50,
                              52, 55, 57, 59}; 
const float THRESHOLD_ON_DEFAULT = 100;
const float MAX_VELOCITY_DEFAULT = 17000;
const float GAIN_VELOCITY_DEFAULT = 1;
const int DURATION_VELOCITY_DEFAULT = 5;
const unsigned long DETECTION_TIME_DEFAULT = 300;


float Threshold_ON = THRESHOLD_ON_DEFAULT;
float Threshold_OFF =0.5 *  THRESHOLD_ON_DEFAULT;
float Max_Velocity = MAX_VELOCITY_DEFAULT;
float Gain_Velocity = GAIN_VELOCITY_DEFAULT;
int Duration_Velocity = DURATION_VELOCITY_DEFAULT;
unsigned long Detection_Time = DETECTION_TIME_DEFAULT;

