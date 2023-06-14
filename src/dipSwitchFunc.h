#ifndef DIPSWITCHFUNC_H
#define DIPSWITCHFUNC_H

#include<Arduino.h>



const int MAX_BOARD_NUMBER = 5; // Cantidad de boards que usaremos.

int findBoard(int pinOctaveSel0, int pinOctaveSel1, int pinOctaveSel2);
int determinarOpcionesDeDip4(int pinOptions0, int pinOptions1);

#endif