#include <Arduino.h>
#include "midiNotesChart.h"

#pragma once


#define COMMA F(",")
#define ESTADO_OFF_ON 0
#define ESTADO_ON_OFF 1
#define ESTADO_ON_NEW_NOTE 2

int findOctave(int pinOctaveSel0, int pinOctaveSel1, int pinOctaveSel2){
  int octava = 0x00;
  bitWrite(octava, 0, !digitalRead(pinOctaveSel0));
  bitWrite(octava, 1, !digitalRead(pinOctaveSel1));
  bitWrite(octava, 2, !digitalRead(pinOctaveSel2));
  return octava;
}


void SerialDebugSignals(unsigned long time, int sensor, float velocity, int estado){
  
  switch (estado)
  {
  case ESTADO_OFF_ON:
    Serial.printf("%.3f  Note:  %s, OFF -> ON, Amplitud: %.2f \n",
                (float)time / 1000.0, 
                NOTAS[CONTROL[sensor]], 
                velocity);
    break;
  case ESTADO_ON_OFF:
    Serial.printf("%.3f  Note:  %s, ON -> OFF, Amplitud: %.2f \n",
                (float)time / 1000.0, 
                NOTAS[CONTROL[sensor]], 
                velocity); 
    break;
  case ESTADO_ON_NEW_NOTE:
    Serial.printf("%.3f  Note:  %s, ON -> NEW NOTE, Amplitud: %.2f \n",
                (float)time / 1000.0, 
                NOTAS[CONTROL[sensor]], 
                velocity); 

    break;

  default:
    break;
  }
 
    
}

void SerialPresentation(HardwareSerial &MySerial, int softwareVersion, int jsonVersion, int hardWareVersion){
    MySerial.println();
    MySerial.printf("Software Ver: %d\nJson Ver: %d\nHardware Ver: %d\n", softwareVersion, jsonVersion, hardWareVersion);
    MySerial.printf("Compilation Date: %s, %s\n", __DATE__ , __TIME__);
    

}

void SerialComandosDisponibles(HardwareSerial &MySerial){
  MySerial.println(F("List of available commands"));
  MySerial.println(F("B - Debug Signals on Serial Port ON / OFF"));
  MySerial.println(F("C,n - Midi Channel [1-16]")); 
  MySerial.println(F("D,n - Detection Time [0, 1000]"));
  MySerial.println(F("F,n - Threshold OFF [0, 0.8 * Threshold on]"));
  MySerial.println(F("G,n - Velocity Gain [1 10]"));
  MySerial.println(F("M,n - Test Midi [0,1]"));
  MySerial.println(F("P - Plot Signals ON / OFF"));
  MySerial.println(F("T,n - Threshold ON [0, 200]"));
  MySerial.println(F("S, n - enabled Sensors [1 - 16]" ));   // TODO: Ajustar a la cantidad de sensores disponibles.

}

void SerialShowConfig(HardwareSerial &MySerial, 
                      int MIDI_CHANNEL,
                      float Threshold_ON,
                      float Threshold_OFF,
                      unsigned long Detection_Time,
                      float Gain_Velocity,
                      int Duration_Velocity,
                      bool TEST_MIDI,
                      int octava)
{
  MySerial.println(F("System Config:"));
  MySerial.printf("Midi Channel Tx: %d \n", MIDI_CHANNEL);
  MySerial.printf("Threshold ON: %.2f, Threshold OFF: %.2f\n",Threshold_ON, Threshold_OFF );
  MySerial.printf("Detection Interval: %d ms\n",Detection_Time);
  MySerial.printf("Velocity Gain: %.2f\n", Gain_Velocity);
  MySerial.printf("Velocity Duration: %d\n", Duration_Velocity);
  MySerial.printf("Octave: %d\n", octava + 1);
  octava == 0? MySerial.println("ESP32 as Master") : MySerial.printf("ESP as Slave %d\n", octava);
  MySerial.print(F("Initial MIDI Test: "));
  TEST_MIDI == true? MySerial.println(F("Yes")) : MySerial.println(F("No"));
  

}

void SerialShowActiveSensors(HardwareSerial &MySerial, int activeSensors){
  for(int i = 1; i <= activeSensors; i++){
            Serial.printf("Sensor %d is active\n", i);
          }
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
  if (boardNumber != 0) SerialMidi.write('M');
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
    
    sendMIDI(SerialMidi, NOTE_ON,CONTROL[j] ,MIDI_VEL_TEST);
    
    sendMIDI(SerialMidi, NOTE_ON,CONTROL[j] ,0);
    j <  3? j++: j=0; 
    MySerial.printf("Sending Note N° %d, MIDI code %d, Note %s\n",i, CONTROL[j],NOTAS[CONTROL[j]]);
    delay(100);
    i++;
  }
  MySerial.println("Midi Test Finished");

}