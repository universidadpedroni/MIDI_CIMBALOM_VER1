#include <Arduino.h>
#include "midiNotesChart.h"

#pragma once


#define COMMA F(",")


void SerialPresentation(HardwareSerial &MySerial){
    MySerial.println();
    MySerial.println(F("MIDI Cimbalon Ver 1.2"));
    MySerial.print(F("Compitation Date: "));
    MySerial.print(__DATE__);
    MySerial.print(COMMA);
    MySerial.println(__TIME__);
    MySerial.println(F("Hardware: ESP32"));

}

void SerialComandosDisponibles(HardwareSerial &MySerial){
  MySerial.println(F("List of available commands"));
  MySerial.println(F("C,n - Midi Channel [1-16]")); 
  MySerial.println(F("D,n - Detection Time [0, 1000]"));
  MySerial.println(F("F,n - Threshold OFF [0, 0.8 * Threshold on]"));
  MySerial.println(F("G,n - Velocity Gain [1 10]"));
  MySerial.println(F("M,n - Test Midi [0,1]"));
  MySerial.println(F("T,n - Threshold ON [0, 200]"));

}

void SerialShowConfig(HardwareSerial &MySerial, 
                      int MIDI_CHANNEL,
                      float Threshold_ON,
                      float Threshold_OFF,
                      unsigned long Detection_Time,
                      float Gain_Velocity,
                      int Duration_Velocity,
                      bool TEST_MIDI)
{
  MySerial.println(F("System Config:"));
  MySerial.printf("Midi Channel Tx: %d \n", MIDI_CHANNEL);
  MySerial.printf("Threshold ON: %.2f, Threshold OFF: %.2f\n",Threshold_ON, Threshold_OFF );
  MySerial.printf("Detection Interval: %d ms\n",Detection_Time);
  MySerial.printf("Velocity Gain: %.2f\n", Gain_Velocity);
  MySerial.printf("Velocity Duration: %d\n", Duration_Velocity);
  
  MySerial.print(F("Initial MIDI Test: "));
  TEST_MIDI == true? MySerial.println(F("Yes")) : MySerial.println(F("No"));
  

}

void SerialShowConfigSensores(HardwareSerial &MySerial, int NUM_SENSORES,
                              int CONTROL[])
{
    Serial.println(F("Sensor Config:"));
    for (int i = 0; i < NUM_SENSORES; i++)
    {
       MySerial.printf("Sensor: %d, Note: %d, %s\n",
                        i,
                        CONTROL[i],
                        NOTAS[CONTROL[i]]); 
    }

}

void SerialSetupFinished(HardwareSerial &MySerial)
{
  MySerial.println(F("Setup Finished. Piezo Capture..."));
}


void SerialShowSignalsLabels(HardwareSerial &MySerial)
{
  Serial.println("Th_ON Th_OFF S0 Max, Sent");

}

void SerialShowSignals(HardwareSerial &MySerial,
                       float Threshold_ON,
                       float Threshold_OFF,
                       float movingAvVelocity,
                       float maxVelocity,
                       float MidiNoteSent)
{
  MySerial.printf("%.2f, %.2f, %.2f, %.2f, %.2f \n",
                    Threshold_ON,
                    Threshold_OFF,
                    movingAvVelocity,
                    maxVelocity,
                    MidiNoteSent);

}

void ShowMidiMessage(HardwareSerial &MySerial, int cmd, int data1, int data2)
{
  MySerial.printf("MIDI: CC: %d, Control: %d, Vel: %d\n",
                  cmd, data1, data2);

}


void sendMIDI(HardwareSerial &SerialMidi, int cmd, int data1, int data2)
{
  // plays a MIDI note or Control change..  Doesn't check to see that
  // cmd is greater than 127, or that data values are  less than 127:
  // Examples: 
  // cmd: NOTE_ON, data1: pitch, data2: 0 -127
  // cmd: NOTE_OFF, data1: pitch, data2: 0
  // cmd: CONTROL_CHANGE, data1: SUSTAIN, data2: 0 - 127
  cmd = cmd | char(MIDI_CHANNEL - 1);    // merge channel number
  
	SerialMidi.write(cmd);
	SerialMidi.write(data1);
	SerialMidi.write(data2);
}

void TestMIDI(HardwareSerial &MySerial,HardwareSerial &SerialMidi, int repeticiones)
{
  MySerial.println(F("Testing MIDI Channel by sending notes"));
  int i = 0;
  int j = 0;
  while(i < repeticiones)
  {
    sendMIDI(SerialMidi, NOTE_ON,CONTROL[j],127);
    
    sendMIDI(SerialMidi, NOTE_ON,CONTROL[j],0);
    j <  3? j++: j=0; 
    MySerial.printf("Sending Note N° %d, MIDI code %d, Note %s\n",i, CONTROL[j],NOTAS[CONTROL[j]]);
    delay(100);
    i++;
  }
  MySerial.println("Midi Test Finished");

}