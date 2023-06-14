#include <dipSwitchFunc.h>



int findBoard(int pinOctaveSel0, int pinOctaveSel1, int pinOctaveSel2){
  int board = 0x00;
  bitWrite(board, 0, !digitalRead(pinOctaveSel0));
  bitWrite(board, 1, !digitalRead(pinOctaveSel1));
  bitWrite(board, 2, !digitalRead(pinOctaveSel2));
  if ( board >= MAX_BOARD_NUMBER) board = MAX_BOARD_NUMBER - 1;
  return board;
}

int determinarOpcionesDeDip4(int pinOptions0, int pinOptions1){
  int options = 0x00;
  bitWrite(options, 0, !digitalRead(pinOptions0));
  bitWrite(options, 1, !digitalRead(pinOptions1));
  return options;
}