#include <Arduino.h>
#include <config.h>
#include <Streaming.h>
#include <blink.h>
#include <SoftwareSerial.h>

// https://forum.arduino.cc/t/reading-piezo-for-midi-note-velocity/69277
// https://iq.opengenus.org/bitwise-division/
// https://ccrma.stanford.edu/~craig/articles/linuxmidi/misc/essenmidi.html
// Lo mejor está en un PDF que se llama Arduino-midi.pdf


blink ledBlink(LED_BUILTIN);
SoftwareSerial SerialMidi(SERIAL_MIDI_RX,SERIAL_MIDI_TX);


void peakDetect(int Voltage);
void doSensorScan();
void lookForChange();
void saveCurrentState();
void sendMIDI(int cmd, int data1, int data2);
void sustainPedalUpdate();
void TestMIDI(int repeticiones);
#ifdef ARDUINO_ARCH_AVR
  void AnalogReadArduinoUNO(int i);
#endif
#ifdef ARDUINO_ARCH_ESP32
  void AnalogReadESP32(int i);
#endif


void setup() 
{
  delay(2000);
  ledBlink.init();
  Serial.begin(BAUDRATE);
  //TestMIDI(5);
#if DEBUG
  Serial << FIRMWARE << F("Compitation Date: ") << __DATE__ << F(", ") << __TIME__ << CR;
#endif
  // Pinout Config
  pinMode(PIN_SPEED,OUTPUT);
  pinMode(MUX_PIN_0,OUTPUT);
  pinMode(MUX_PIN_1,OUTPUT);
  pinMode(MUX_PIN_2,OUTPUT);
  pinMode(PIN_SUSTAIN_PEDAL,INPUT_PULLUP);
  
  
  SerialMidi.begin(MIDI_BAUDRATE);
#ifdef ARDUINO_ARCH_ESP32
  analogReadResolution(10); // Para hacerlo compatible con Arduino UNO.
#endif
#if DEBUG
  Serial << F("Setup Finished. Piezo Capture...\n");
#endif
   
}

void loop() 
{
  digitalWrite(PIN_SPEED,HIGH);

  doSensorScan();
  lookForChange();
  saveCurrentState();
  sustainPedalUpdate();

  ledBlink.update(LED_INTERVAL);
  digitalWrite(PIN_SPEED,LOW);
  
}

void sustainPedalUpdate()
{
  // TO DO: Check if debouncing is needed
  sustainCurrentState = digitalRead(PIN_SUSTAIN_PEDAL);
  if (sustainCurrentState != sustainlastState)  // pedal changed state!
  {
    if (sustainCurrentState == 0) // se apretó el pedal
    {
      sendMIDI(CONTROL_CHANGE,SUSTAIN,127);;
    }
    else
    {
      sendMIDI(CONTROL_CHANGE,SUSTAIN,0);
    }
    sustainlastState = sustainCurrentState;
  }
}


void doSensorScan()
{
  for (int i = 0; i  < SENSOR_NUMBER_PER_MULTIPLEXOR; i++)
  {
    digitalWrite(MUX_PIN_0, i & 0x1);
    digitalWrite(MUX_PIN_1, (i>>1) & 0x1);
    digitalWrite(MUX_PIN_2, (i>>2) & 0x1);
#ifdef ARDUINO_ARCH_AVR
    AnalogReadArduinoUNO(i);
#endif
#ifdef ARDUINO_ARCH_ESP32
    AnalogReadESP32(i);
#endif
  }
}

// the value of threshold determins the on / off point
void lookForChange()
{
  int ledVal = 0;
  int ledMask = 1;

  for (int i = 0; i < SENSOR_NUMBER_TOTAL; i++)
  {
    if(currentState[i] < THRESHOLD)  ledVal |= ledMask;  // bitwise or operation https://www.arduino.cc/reference/tr/language/structure/compound-operators/compoundbitwiseor/
    ledMask = ledMask << 1;

  }
  if (lastLedVal != ledVal)   // something changed!
  {
    ledMask = 1;
    for( int i = 0; i < SENSOR_NUMBER_TOTAL; i++)
    {
      if ((ledMask & ledVal) != (ledMask & lastLedVal))
      {
        if((ledMask & ledVal) == 0) // note off
        {
          sendMIDI(NOTE_OFF, control[i], 0x00);
        }
        else
        {
          sendMIDI(NOTE_ON, control[i], (currentState[i] >> 3)); // SI CAMBIA EL CHIP, REVISAR QUE NÚMERO HAY QUE DIVIDIR POR 127 y AJUSTAR EL SHIFT
          //controlSend(0x90, control[i], currentState[i]>>3);   // 1023 / 127 = 8
        }
      }
      ledMask = ledMask << 1;
    }
    lastLedVal = ledVal;
  }

}

void saveCurrentState()
{  // save the current state for next time
 for(int i = 0; i < SENSOR_NUMBER_TOTAL; i++)
 {
   lastState[i] = currentState[i];
 }
}

// plays a MIDI note or Control change..  Doesn't check to see that
// cmd is greater than 127, or that data values are  less than 127:
// Examples: 
// cmd: NOTE_ON, data1: pitch, data2: 0 -127
// cmd: NOTE_OFF, data1: pitch, data2: 0
// cmd: CONTROL_CHANGE, data1: SUSTAIN, data2: 0 - 127
void sendMIDI(int cmd, int data1, int data2) 
{
  cmd = cmd | char(MIDI_CHANNEL - 1);    // merge channel number
	SerialMidi.write(cmd);
	SerialMidi.write(data1);
	SerialMidi.write(data2);

	 //USER FRIENDLY
#if DEBUG
	Serial.print(cmd);
	Serial.print("\t");
	Serial.print(pitch);
	Serial.print("\t");
	Serial.println(velocity);
#else
  
  Serial.write(cmd);
  Serial.write(data1);
  Serial.write(data2);
#endif

}
#ifdef ARDUINO_ARCH_AVR
void AnalogReadArduinoUNO(int i)
{
  currentState[i] = analogRead(MUX_0_ADC);
  delay(1);
  currentState[i + 8] = analogRead(MUX_1_ADC);

}
#endif

#ifdef ARDUINO_ARCH_ESP32
void AnalogReadESP32(int i)
{
  currentState[i] = analogRead(MUX_0_ADC);
  currentState[i + 8] = analogRead(MUX_1_ADC);
  
}
#endif

void TestMIDI(int repeticiones)
{
  
  int i = 0;
  while(i < repeticiones)
  {
    sendMIDI(CONTROL_CHANGE,SUSTAIN,127);
    delay(1000);
    sendMIDI(CONTROL_CHANGE,SUSTAIN,0);
    delay(1000);
    sendMIDI(NOTE_ON,60,127);
    delay(1000);
    sendMIDI(NOTE_ON,60,0);
    delay(1000);
    
    i++;
  }
  
}


