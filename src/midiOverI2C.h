#ifndef MIDI_OVER_I2C_H
#define MIDI_OVER_I2C_H

#include <Wire.h>

class midiOverI2C {
  public:
    midiOverI2C();
    void begin(int boardNumber);
    void receiveMIDI(int& cmd, int& data1, int& data2, int slaveNumber);

  private:
    int boardNumber;
   
};

#endif



