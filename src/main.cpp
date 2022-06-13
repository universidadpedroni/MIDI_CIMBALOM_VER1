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
void TestMIDI(int repeticiones);
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
  analogRead(GPIO_NUM_15);
  delay(400);
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
  //float velocityVector[NUM_SENSORES] = {0,0,0,0};
  //bool sendToSerial = false;
  
  ledBlink.update(LED_INTERVAL);
  sustainPedalUpdate();

  findNotes2(0);
  findNotes2(1);
  findNotes2(2);
  findNotes2(3);
  /*
  for (int sensor = 0; sensor < NUM_SENSORES; sensor++)
  {
    velocityVector[sensor] = (int)(MIDI_GAIN * findNotes2(sensor));
    if (velocityVector[sensor] > 0) sendToSerial = true;
  }
  

  if (sendToSerial)
  {
    sendToSerial = false;
    Serial.printf("Sens 1: %.2f \t Sens 2: %.2f \t Sens 3: %.2f \t Sens 4: %.2f \n",
                   velocityVector[0],velocityVector[1],velocityVector[2],velocityVector[3]);
  } 
  */   

  
       
   
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



void findNotes2(int sensor)
{
  // SPEED: 65uSeg!!!!
  digitalWrite(GPIO_NUM_23,HIGH);
  static long notasDetectadas = 0;

  
  
  float movingAvVelocityk[NUM_SENSORES] = {0};
  float movingAvDerivk[NUM_SENSORES] = {0};
  static float movingAvVelocityk_1[NUM_SENSORES] = {0};
  static float derivk[NUM_SENSORES][5] = {0};
  static float amplik[NUM_SENSORES][3] = {0};
  static unsigned long detectionTime[NUM_SENSORES] = {0};
  unsigned long currentTime = millis();
  static int noteState[NUM_SENSORES] = {NOTE_OFF, NOTE_OFF, NOTE_OFF,NOTE_OFF};
  static bool notaEnviada[NUM_SENSORES] = {false, false, false, false};
  static float maxVelocity[NUM_SENSORES] = {0};


  // Reading the amplitude of the signal and going through moving average filter
  amplik[sensor][0] =  (float)analogRead(CANALES_ADC[sensor]) * CANALES_GAIN[sensor];
  movingAvVelocityk[sensor] = (amplik[sensor][0] + amplik[sensor][1] + amplik[sensor][2]) / 3;
  // Determining the first derivative of the signal ang going through moving average filter.
  derivk[sensor][0] = (movingAvVelocityk[sensor] - movingAvVelocityk_1[sensor]) / SAMPLETIME;
  
  movingAvDerivk[sensor] = (derivk[sensor][0] + derivk[sensor][1] + 
                            derivk[sensor][2] + derivk[sensor][3] + derivk[sensor][4]) / 5;
  
  // Máquina de Estados
  // 1. Nota OFF -> Nota ON
  if ((noteState[sensor] == NOTE_OFF) && 
      (movingAvDerivk[sensor] >= DERIVATIVE_THRESHOLD_LOW) &&
      (currentTime - detectionTime[sensor] >= DETECTION_TIME))
  {
    
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    notaEnviada[sensor] = false;
    Serial.printf("%d Sensor %d, OFF_ON \n",millis(), sensor);
    
  }
  // 2. Nota ON -> Nota ON
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvDerivk[sensor] >= DERIVATIVE_THRESHOLD_HIGH) &&
           (currentTime - detectionTime[sensor] >= DETECTION_TIME))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    notaEnviada[sensor] = false;
    Serial.printf("%d Sensor %d, ON_ON \n",millis(), sensor);

    
    
  }
  // 3. Nota ON -> Nota OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (currentTime - detectionTime[sensor] >= 6 * DETECTION_TIME))
  {
    noteState[sensor] = NOTE_OFF;
    Serial.printf("%d Sensor %d, ON_OFF \n",millis(), sensor);
    //sendMIDI(NOTE_OFF,CONTROL[sensor],0);
    
  }
  // una vez detectada la nota, busco el máximo valor (dentro de un intervalo de tiempo)
  if(noteState[sensor] == NOTE_ON &&
    (currentTime - detectionTime[sensor]) < MAX_LATENCY && 
     (notaEnviada[sensor] == false))
  {
    if ( maxVelocity[sensor] < movingAvVelocityk[sensor])
    {
      maxVelocity[sensor] = movingAvVelocityk[sensor];
    }
    else
    {
       notaEnviada[sensor] = true;
       if((int)(MIDI_GAIN * maxVelocity[sensor]) > MIDI_MINIMUM_VELOCITY)
       {
        //sendMIDI(NOTE_ON,CONTROL[sensor],(int)(MIDI_GAIN * maxVelocity[sensor]));
        
        
        #if DEBUG
        notasDetectadas++;
        Serial.printf("t: %d \t Nota N: %d \t Sens: %d \t Vel: %d \n",
                       millis(), notasDetectadas, sensor, (int)(MIDI_GAIN * maxVelocity[sensor]));
        #endif
        maxVelocity[sensor] = 0;
       }
       
    }
  }
  // Si no encuentro la velocidad máxima de la latencia máxima permitida, envío la velocidad actual
  else if (noteState[sensor] == NOTE_ON &&
    (currentTime - detectionTime[sensor]) > MAX_LATENCY && 
     (notaEnviada[sensor] == false))
     {
      Serial.println("..");
       if((int)(MIDI_GAIN * maxVelocity[sensor]) > MIDI_MINIMUM_VELOCITY)
       {
        //sendMIDI(NOTE_ON,CONTROL[sensor],(int)(MIDI_GAIN * maxVelocity[sensor]));
        maxVelocity[sensor] = 0;
       }

     }


  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];
  movingAvVelocityk_1[sensor] = movingAvVelocityk[sensor];
  derivk[sensor][4] = derivk[sensor][3];
  derivk[sensor][3] = derivk[sensor][2];
  derivk[sensor][2] = derivk[sensor][1];
  derivk[sensor][1] = derivk[sensor][0];

  

  digitalWrite(GPIO_NUM_23,LOW);    
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


// COPIA DE LA FUNCIÓN FINDNOTES2 QUE FUNCIONA
/*
float findNotes2(int sensor)
{
  // SPEED: 65uSeg!!!!
  digitalWrite(GPIO_NUM_23,HIGH);
  static long notasDetectadas = 0;

  
  
  float movingAvVelocityk[NUM_SENSORES] = {0};
  float movingAvDerivk[NUM_SENSORES] = {0};
  static float movingAvVelocityk_1[NUM_SENSORES] = {0};
  static float derivk[NUM_SENSORES][5] = {0};
  static float amplik[NUM_SENSORES][3] = {0};
  static unsigned long detectionTime[NUM_SENSORES] = {0};
  unsigned long currentTime = millis();
  static int noteState[NUM_SENSORES] = {NOTE_OFF, NOTE_OFF, NOTE_OFF,NOTE_OFF};
  
  // Reading the amplitude of the signal and going through moving average filter
  amplik[sensor][0] =  (float)analogRead(CANALES_ADC[sensor]);
  movingAvVelocityk[sensor] = (amplik[sensor][0] + amplik[sensor][1] + amplik[sensor][2]) / 3;
  // Determining the first derivative of the signal ang going through moving average filter.
  derivk[sensor][0] = (movingAvVelocityk[sensor] - movingAvVelocityk_1[sensor]) / SAMPLETIME;
  
  movingAvDerivk[sensor] = (derivk[sensor][0] + derivk[sensor][1] + 
                            derivk[sensor][2] + derivk[sensor][3] + derivk[sensor][4]) / 5;
  
  // Máquina de Estados
  // 1. Nota OFF -> Nota ON
  if ((noteState[sensor] == NOTE_OFF) && 
      (movingAvDerivk[sensor] >= DERIVATIVE_THRESHOLD_LOW) &&
      (currentTime - detectionTime[sensor] >= DETECTION_TIME))
  {
    
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    sendMIDI(NOTE_ON,CONTROL[sensor],(int)(MIDI_GAIN * movingAvVelocityk[sensor]));
    
    #if DEBUG
    notasDetectadas++;
    Serial << "Nota: " << notasDetectadas;
    Serial << ", Sensor" << sensor;
    Serial <<", Derivada: " << movingAvDerivk[sensor]; 
    Serial << ", Amp: " << movingAvVelocityk[sensor]; 
    Serial << ", Vel:" << (int)(MIDI_GAIN * movingAvVelocityk[sensor]) << CR;
    #endif
    
  }
  // 2. Nota ON -> Nota ON
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvDerivk[sensor] >= DERIVATIVE_THRESHOLD_HIGH) &&
           (currentTime - detectionTime[sensor] >= DETECTION_TIME))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    //sendMIDI(NOTE_OFF,CONTROL[sensor],0);
    sendMIDI(NOTE_ON,CONTROL[sensor],(int)(MIDI_GAIN * movingAvVelocityk[sensor]));
    
    #if DEBUG
    notasDetectadas++;
    Serial << "Nota: " << notasDetectadas;
    Serial << ", Sensor" << sensor;
    Serial << ", Derivada: " << movingAvDerivk[sensor]; 
    Serial << ", Amp: " << movingAvVelocityk[sensor]; 
    Serial << ", Vel:" << (int)(MIDI_GAIN * movingAvVelocityk[sensor]) << CR;
    #endif
      
  }
  else if ((noteState[sensor] == NOTE_ON) &&
           (currentTime - detectionTime[sensor] >= 4 * DETECTION_TIME))
  {
    noteState[sensor] = NOTE_OFF;
    //sendMIDI(NOTE_OFF,CONTROL[sensor],0);
    
    #if DEBUG
    Serial << "Nota: " << notasDetectadas;
    Serial << ", Sensor" << sensor;
    Serial << "OFF" << CR;
    #endif
     
  }
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];
  movingAvVelocityk_1[sensor] = movingAvVelocityk[sensor];
  derivk[sensor][4] = derivk[sensor][3];
  derivk[sensor][3] = derivk[sensor][2];
  derivk[sensor][2] = derivk[sensor][1];
  derivk[sensor][1] = derivk[sensor][0];

  return movingAvVelocityk[sensor];

  digitalWrite(GPIO_NUM_23,LOW);    
}

*/
