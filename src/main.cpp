#include <Arduino.h>
#include <config.h>
#include <Streaming.h>
#include <blink.h>
//#include <SoftwareSerial.h>
//#include <Wire.h>
//#include <Adafruit_BusIO_Register.h>
//#include <Adafruit_ADS1X15.h>


// https://forum.arduino.cc/t/reading-piezo-for-midi-note-velocity/69277
// https://iq.opengenus.org/bitwise-division/
// https://ccrma.stanford.edu/~craig/articles/linuxmidi/misc/essenmidi.html
// Lo mejor está en un PDF que se llama Arduino-midi.pdf


blink ledBlink(PIN_LED);
//Adafruit_ADS1015 ads[ADS_INSTALLED];


void sendMIDI(int cmd, int data1, int data2);
void sustainPedalUpdate();
void cimbalomUpdate();
void TestMIDI(int repeticiones);
void findNotes(int j);
void findNotes2(int j);
void sendADCtoSerial(unsigned long interval, uint32_t delayTime);


void setup()
{
  Serial.begin(BAUDRATE);
  SerialMidi.begin(MIDI_BAUDRATE);
    //TestMIDI(5);
  ledBlink.init();
  
  
 
  analogReadResolution(16);
  
  pinMode(PIN_SPEED,OUTPUT);

  pinMode(PIN_SUSTAIN_PEDAL,INPUT_PULLUP);
  
  // Serial Midi Port  
  SerialMidi.begin(MIDI_BAUDRATE);

#if DEBUG
  Serial << FIRMWARE << F("Compitation Date: ") << __DATE__ << F(", ") << __TIME__ << CR << F("Hardware:");
  Serial << F("ESP32") << CR;
  
  Serial << F("Core used: ") << xPortGetCoreID() << CR;
  Serial << F("Setup Finished. Piezo Capture...\n");
  
  /*
  analogRead(GPIO_NUM_15);
  delay(1000);
  for(int i = 5; i > 0 ; i--)
  {
    Serial << F("Starting capture in: ") << i << CR;
    delay(1000);
  }
  
  sendADCtoSerial(2000,1);
  */

#endif
//TODO: Enter Setup mode to adjust threshold and release time when button is pressed
}

void loop()
{
  digitalWrite(PIN_SPEED,HIGH);
  ledBlink.update(LED_INTERVAL);
  sustainPedalUpdate();
  cimbalomUpdate();
  
  
  digitalWrite(PIN_SPEED,LOW);
  
   
}

void sendADCtoSerial(unsigned long interval, uint32_t delayTime)
{
  unsigned long initialTime = millis();
  unsigned long currentTime = millis();
  while (currentTime - initialTime <= interval)
  {
    currentTime = millis();
    Serial << _FLOAT((float)(currentTime - initialTime) / 1000,4) << TAB << analogRead(GPIO_NUM_15) << CR;
    delay(delayTime);
  }

}


void cimbalomUpdate()
{
  // Lectura de ADCs. Se leen los 4 juntos.
  //currentValue[0] = analogRead(GPIO_NUM_15);  // ADC2_3
  //currentValue[2] = analogRead(GPIO_NUM_4);   // ADC2_0
  //currentValue[1] = analogRead(GPIO_NUM_2);   // ADC2_2
  //currentValue[3] = analogRead(GPIO_NUM_13);  // ADC2_4
  
  
  // TO DO: Moving average filter

  // Buscar notas
  //findNotes(0);
  //findNotes(1);
  findNotes2(0);
  delay(10);
  //findNotes(3);
  
}


void findNotes2(int j)
{
  static long notasDetectadas = 0;

  const float MIDI_GAIN = 127.0 / 10000.0;
  const float SAMPLETIME = 0.01;
  const float DERIVATIVE_THRESHOLD_LOW = 10000;
  const float DERIVATIVE_THRESHOLD_HIGH = 30000;
  const unsigned long DETECTION_TIME = 300;
  
  float movingAvVelocityk = 0;
  float movingAvDerivk = 0;
  static float movingAvVelocityk_1 = 0;
  static float derivk[5] = {0};
  static float amplik[3] = {0};
  static unsigned long detectionTime = 0;
  unsigned long currentTime = millis();
  static int noteState = NOTE_OFF;


  amplik[0] =  (float)analogRead(GPIO_NUM_15);
  movingAvVelocityk = (amplik[0] + amplik[1] + amplik[2]) / 3;
  derivk[0] = (movingAvVelocityk - movingAvVelocityk_1) / SAMPLETIME;
  movingAvDerivk = (derivk[0] + derivk[1] + derivk[2] + derivk[3] + derivk[4]) / 5;
  // Máquina de Estados
  // 1. Nota OFF -> Nota ON
  if ((noteState == NOTE_OFF) && (movingAvDerivk >= DERIVATIVE_THRESHOLD_LOW) && (currentTime - detectionTime >= DETECTION_TIME))
  {
    
    noteState = NOTE_ON;
    detectionTime = currentTime;
    #if DEBUG
    notasDetectadas++;
    Serial << "Nota: " << notasDetectadas;
    Serial <<", Derivada: " << movingAvDerivk; 
    Serial << ", Amp: " << movingAvVelocityk; 
    Serial << ", Vel:" << (int)(MIDI_GAIN * movingAvVelocityk) << CR;
    #endif
  }
  // 2. Nota ON -> Nota ON
  else if ((noteState == NOTE_ON) && (movingAvDerivk >= DERIVATIVE_THRESHOLD_HIGH) && (currentTime - detectionTime >= DETECTION_TIME))
  {
    noteState = NOTE_ON;
    detectionTime = currentTime;
    #if DEBUG
    notasDetectadas++;
    Serial << "Nota: " << notasDetectadas;
    Serial <<", Derivada: " << movingAvDerivk; 
    Serial << ", Amp: " << movingAvVelocityk; 
    Serial << ", Vel:" << (int)(MIDI_GAIN * movingAvVelocityk) << CR;
    #endif  
  }
  else if ((noteState == NOTE_ON) && (currentTime - detectionTime >= 4 * DETECTION_TIME))
  {
    noteState = NOTE_OFF;
  }
         
}


// ESTA FUNCIÓN FALLA CON MÁS DE UN ADC
void findNotes(int j)
{
  
  // State machine
    if ((lastValue[j] <= currentValue[j]) & (currentValue[j] >= THRESHOLD[j]) & ((millis() - lastTimeStateChange[j]) >= DELTA_TIME[j]))
    {
      //sendMIDI()
#if DEBUG
      static long contadorDeNotas = 1;
      Serial << F("Note Nº ") << contadorDeNotas << F(" found, Amplitude: ") << currentValue[j] << F("\n");
      contadorDeNotas++;
#else
      sendMIDI(NOTE_ON,60,currentValue[j]);
#endif

      lastTimeStateChange[j] = millis();
    }
    lastValue[j] = currentValue[j];

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
	Serial.print(data1);
	Serial.print("\t");
	Serial.println(data2);
#else
  
  Serial.write(cmd);
  Serial.write(data1);
  Serial.write(data2);
#endif

}


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


