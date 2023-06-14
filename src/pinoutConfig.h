#ifndef PINOUTCONFIG_H
#define PINOUTCONFIG_H

#include <Arduino.h>

// Pines de configuración

const int PIN_OPTIONS_0 = GPIO_NUM_15;                        // Pin para activar el OTA
const int PIN_OPTIONS_1 = GPIO_NUM_4;      // Pin que determina si se usa la función findNotes3NoVelocity(i) o findNotes3(i);
                                                       // Pin para ejecutar el MIDI TEST

/* Tabla de verdad de los pines
PIN OPTIONS 0   PIN OPTIONS 1
0               0                   Detectar Velocidad
0               1                   No detectar Velocidad
1               0                   Test Midi
1               1                   OTA
*/
const int OPTIONS_DETECTAR_VELOCIDAD = 0;
const int OPTIONS_NO_DETECTAR_VELOCIDAD = 1;
const int OPTIONS_TEST_MIDI = 2;
const int OPTIONS_OTA = 3;


// Pines para seleccionar el board.
const int PIN_OCT_SEL_0 = GPIO_NUM_19;
const int PIN_OCT_SEL_1 = GPIO_NUM_18;
const int PIN_OCT_SEL_2 = GPIO_NUM_5;

// Pines de los sensores
const int PIN_ENV_0 = GPIO_NUM_13;    // ENV0
const int PIN_ENV_1 = GPIO_NUM_12;    // ENV1
const int PIN_ENV_2 = GPIO_NUM_14;    // ENV2
const int PIN_ENV_3 = GPIO_NUM_27;    // ENV3
const int PIN_ENV_4 = GPIO_NUM_26;    // ENV4
const int PIN_ENV_5 = GPIO_NUM_25;    // ENV5
const int PIN_ENV_6 = GPIO_NUM_33;    // ENV6
const int PIN_ENV_7 = GPIO_NUM_32;    // ENV7
const int PIN_ENV_8 = GPIO_NUM_35;    // ENV8
const int PIN_ENV_9 = GPIO_NUM_34;    // ENV9
const int PIN_ENV_10 = GPIO_NUM_39;    // ENV10
const int PIN_ENV_11 = GPIO_NUM_36;    // ENV11

// Pin para el pedal de sustain
const int PIN_SUSTAIN_PEDAL = GPIO_NUM_23;     // Pin para el pedal de sustain

// Pines para la comunicación entre boards
const int PIN_TX_MASTER = GPIO_NUM_21;
const int PIN_RX_MASTER = GPIO_NUM_22;
const int PIN_TX_SLAVE = GPIO_NUM_22;
const int PIN_RX_SLAVE = GPIO_NUM_21;

#endif
