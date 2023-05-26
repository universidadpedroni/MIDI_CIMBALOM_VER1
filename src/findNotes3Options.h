#ifndef FINDNOTES3OPTIONS_H
#define FINDNOTES3OPTIONS_H

#include <Arduino.h>
#include <pinoutConfig.h>


// GENERALES
//https://anotherproducer.com/online-tools-for-musicians/midi-cc-list/
const int MIDI_CHANNEL_DEFAULT = 5; // Default MIDI CHANNEL
int MIDI_CHANNEL = MIDI_CHANNEL_DEFAULT;           
const int NOTE_ON = 0x90;
const int NOTE_OFF = 0x80;
const int CONTROL_CHANGE = 0xB0;
const int SUSTAIN = 0x40;       // sustain pedal
// const int AFTERTOUCH = 0xA0;

const int CANALES_ADC[] = { PIN_ENV_0,
                            PIN_ENV_1,
                            PIN_ENV_2,
                            PIN_ENV_3,
                            PIN_ENV_4,
                            PIN_ENV_5,
                            PIN_ENV_6,
                            PIN_ENV_7,
                            PIN_ENV_8,
                            PIN_ENV_9,
                            PIN_ENV_10,
                            PIN_ENV_11}; // 12 canales, 1 octava completa.

const int NUM_SENSORES = 12;

int noteState[NUM_SENSORES] = {NOTE_OFF};                               
bool notaEnviada[NUM_SENSORES] = {false};
float amplik[NUM_SENSORES][3] = {0}; // Sensor reading
float movingAvVelocityk[NUM_SENSORES] = {0};  // Sensor filtering.
unsigned long detectionTime[NUM_SENSORES] = {0};

float maxVelocity[NUM_SENSORES] = {0};
unsigned long MAX_LATENCY = 150; // Must be lower than DETECTION_TIME

int contadorMax[NUM_SENSORES] = {0};

// Constants with DEFAULT VALUES. All of this parameters are updated using a json file.

// notes to play, corresponding to all sensors: 
int CONTROL[NUM_SENSORES] = {0}; 

const float THRESHOLD_ON_DEFAULT = 100.0;
const float MAX_VELOCITY_DEFAULT = 17000.0;
const float GAIN_VELOCITY_DEFAULT = 1.0;
const int DURATION_VELOCITY_DEFAULT = 5;
const unsigned long DETECTION_TIME_DEFAULT = 300;


float Threshold_ON = THRESHOLD_ON_DEFAULT;
float Threshold_OFF =0.5 *  THRESHOLD_ON_DEFAULT;
float Max_Velocity = MAX_VELOCITY_DEFAULT;
float Gain_Velocity = GAIN_VELOCITY_DEFAULT;
int Duration_Velocity = DURATION_VELOCITY_DEFAULT;
unsigned long Detection_Time = DETECTION_TIME_DEFAULT;

#endif