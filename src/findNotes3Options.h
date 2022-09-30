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


const int NUM_SENSORES = 10; // Esta es la variable que manda con cuántos sensores se trabaja. 10 sensores, 700uSeg

const int CANALES_ADC[] = { GPIO_NUM_36,    // ENV0_ADC1_CH0
                            GPIO_NUM_39,    // ENV1_ADC1_CH3
                            GPIO_NUM_34,    // ENV2_ADC1_CH6
                            GPIO_NUM_35,    // ENV3_ADC1_CH7
                            GPIO_NUM_32,    // ENV4_ADC1_CH4
                            GPIO_NUM_33,    // ENV5_ADC1_CH5
                            GPIO_NUM_25,    // ENV6_ADC2_CH8
                            GPIO_NUM_26,    // ENV7_ADC2_CH9
                            GPIO_NUM_27,    // ENV8_ADC2_CH7 
                            GPIO_NUM_14,    // ENV9_ADC2_CH6
                            GPIO_NUM_12,    // ENV10_ADC2_CH5
                            GPIO_NUM_13,    // ENV11_ADC2_CH4
                            GPIO_NUM_4,     // ENV12_ADC2_CH0
                            GPIO_NUM_2,     // ENV13_ADC2_CH2
                            GPIO_NUM_15     // ENV14_ADC2_CH3    
                        };


int noteState[NUM_SENSORES] = {NOTE_OFF, NOTE_OFF, NOTE_OFF, NOTE_OFF,
                               NOTE_OFF, NOTE_OFF, NOTE_OFF, NOTE_OFF,
                               NOTE_OFF, NOTE_OFF};
                               
bool notaEnviada[NUM_SENSORES] = {false, false, false, false,
                                  false, false, false, false,
                                  false, false};


float amplik[NUM_SENSORES][3] = {0}; // Sensor reading
float movingAvVelocityk[NUM_SENSORES] = {0};  // Sensor filtering.
unsigned long detectionTime[NUM_SENSORES] = {0};

float maxVelocity[NUM_SENSORES] = {0};
unsigned long MAX_LATENCY = 150; // Must be lower than DETECTION_TIME

// Constants with DEFAULT VALUES. All of this parameters are updated using a json file.

// notes to play, corresponding to all sensors: 
int CONTROL[] = { 36, 38, 40, 43,
                        45, 47, 48, 50,
                        52, 55}; 
const float THRESHOLD_ON_DEFAULT = 6000;
const float MAX_VELOCITY_DEFAULT = 17000;
const unsigned long DETECTION_TIME_DEFAULT = 300;


float Threshold_ON = THRESHOLD_ON_DEFAULT;
float Threshold_OFF =0.5 *  THRESHOLD_ON_DEFAULT;
float Max_Velocity = MAX_VELOCITY_DEFAULT;
unsigned long Detection_Time = DETECTION_TIME_DEFAULT;

/*
float Threshold_ON[NUM_SENSORES] = {4000, 4000, 4000, 4000,
                                    4000, 4000, 4000, 5000,
                                    12000, 12000};

float Threshold_OFF[NUM_SENSORES] = {2000, 2000, 2000, 2000,
                                     2000, 3000, 3000, 4000,
                                     8000, 8000};

float  MAX_VELOCITY[NUM_SENSORES] = {17000, 17000, 17000, 17000,
                                     17000, 17000, 17000, 17000,
                                     17000, 17000 }; // Maximum measured velocity (17.000), usada solo en finNotes3()
*/                                     