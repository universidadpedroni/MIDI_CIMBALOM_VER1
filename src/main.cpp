#include <Arduino.h>
#include <config.h>
#include <Streaming.h>
#include <blink.h>
#include <findNotes3Options.h>

// https://forum.arduino.cc/t/reading-piezo-for-midi-note-velocity/69277
// https://iq.opengenus.org/bitwise-division/
// https://ccrma.stanford.edu/~craig/articles/linuxmidi/misc/essenmidi.html
// Lo mejor está en un PDF que se llama Arduino-midi.pdf
// CABLE MIDI. AMARILL=,VCC. BLANCO, DATO

blink ledBlink(PIN_LED);


void sendMIDI(int cmd, int data1, int data2);
void sustainPedalUpdate();
void TestMIDI(int repeticiones);
void findNotes3(int sensor);
void findNotes3NoVelocity(int sensor);
void sendADCtoSerial(unsigned long interval, uint32_t delayTime);
void plotSignalAndThresholds(float Velocity, int noteState, float Threshold_ON, float Threshold_OFF);


// TODO: La interfaz midi funciona exclusivamente con 5v

void setup()
{
  delay(2000);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,HIGH);

  Serial.begin(BAUDRATE);
  SerialMidi.begin(MIDI_BAUDRATE);
  SerialMidi.flush();
  if (TEST_MIDI) TestMIDI(4);
  
  analogReadResolution(16);
  
  pinMode(PIN_SPEED,OUTPUT);

  pinMode(PIN_SUSTAIN_PEDAL,INPUT_PULLUP);
  
  
if (DEBUG)
{
  Serial << FIRMWARE << F("Compitation Date: ") << __DATE__ << F(", ") << __TIME__ << CR << F("Hardware:");
  Serial << F("ESP32") << CR;
  Serial << F("Core used: ") << xPortGetCoreID() << CR;
  Serial << F("Setup Finished. Piezo Capture...\n");
}  

//TODO: Enter Setup mode to adjust threshold and release time when button is pressed
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  ledBlink.init();
}

void loop()
{
  ledBlink.update(LED_INTERVAL);
  sustainPedalUpdate();
  
  for (int i = 0; i < NUM_SENSORES; i++)
  {
    findNotes3NoVelocity(i);

  }
     
}




// FUNCIONANDO, SIN VELOCIDAD
void findNotes3NoVelocity(int sensor)
{
  if (sensor == 0) digitalWrite(PIN_SPEED,HIGH);
  unsigned long currentTime = millis();
  
  // Reading the amplitude of the signal and going through moving average filter
  amplik[sensor][0] =  (float)analogRead(CANALES_ADC[sensor]);
  movingAvVelocityk[sensor] = (amplik[sensor][0] + amplik[sensor][1] + amplik[sensor][2]) / 3;
  

  // *** State Machine ***
  // 1. NOTE OFF -> NOTE OFF
  if((noteState[sensor] == NOTE_OFF) && 
     (movingAvVelocityk[sensor] < Threshold_ON[sensor]))
     {
        // Nothing to do here
     }
  // 2. NOTE OFF - NOTE ON   
  else if ((noteState[sensor] == NOTE_OFF) && 
           (movingAvVelocityk[sensor] > Threshold_ON[sensor]))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    if (DEBUG) Serial.printf("%d Nota:  %s, OFF -> ON \n",millis(), NOTAS[sensor]);
    sendMIDI(NOTE_ON,CONTROL[sensor],127);
  }
  // 3. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] < Threshold_OFF[sensor]))
  {
    noteState[sensor] = NOTE_OFF;
    if(DEBUG) Serial.printf("%d  Nota:  %s, ON -> OFF \n",millis(), NOTAS[sensor]);
    sendMIDI(NOTE_OFF,CONTROL[sensor],0);
  }
  // 4. NOTE ON - NEW NOTE
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] > Threshold_ON[sensor]) &&
           (currentTime - detectionTime[sensor] >= DETECTION_TIME))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    if (DEBUG) Serial.printf("%d  Nota:  %s, ON -> NEW NOTE \n",millis(), NOTAS[sensor]); 
    
    sendMIDI(NOTE_ON,CONTROL[sensor],127);
  }
  
  
  

  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];

  // Plot Sensor 2 and 3.
  if (PLOT_SIGNALS)
  {
    plotSignalAndThresholds(movingAvVelocityk[plotSensor],noteState[plotSensor],Threshold_ON[plotSensor],Threshold_OFF[plotSensor]);
  }
  
  
  if (sensor == 0) digitalWrite(PIN_SPEED,LOW);
}

void plotSignalAndThresholds(float Velocity, int noteState, float Threshold_ON, float Threshold_OFF)
{
  Serial.print(Velocity); Serial.print(","); Serial.print(160 * noteState); Serial.print(",");
    Serial.print(Threshold_ON);
    Serial.print(",");
    Serial.println(Threshold_OFF);

}


void findNotes3(int sensor)
{
  if (sensor == 0) digitalWrite(PIN_SPEED,HIGH);
  unsigned long currentTime = millis();
  
  // Reading the amplitude of the signal and going through moving average filter
  amplik[sensor][0] =  (float)analogRead(CANALES_ADC[sensor]);
  movingAvVelocityk[sensor] = (amplik[sensor][0] + amplik[sensor][1] + amplik[sensor][2]) / 3;
  

  // *** State Machine ***
  // 1. NOTE OFF -> NOTE OFF
  if((noteState[sensor] == NOTE_OFF) && 
     (movingAvVelocityk[sensor] < Threshold_ON[sensor]))
     {

     }
  // 2. NOTE OFF - NOTE ON   
  else if ((noteState[sensor] == NOTE_OFF) && 
           (movingAvVelocityk[sensor] > Threshold_ON[sensor]))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    notaEnviada[sensor] = false;
    //Serial.printf("%d Nota:  %s, OFF -> ON \n",millis(), NOTAS[sensor]);
    //sendMIDI(NOTE_ON,CONTROL[sensor],127);
  }
  // 3. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] < Threshold_OFF[sensor]))
  {
    noteState[sensor] = NOTE_OFF;
    Serial.printf("%d  Nota:  %s, ON -> OFF \n",millis(), NOTAS[sensor]);
    //sendMIDI(NOTE_OFF,CONTROL[sensor],0);
  }
  // 4. NOTE ON - NEW NOTE
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] > Threshold_ON[sensor]) &&
           (currentTime - detectionTime[sensor] >= DETECTION_TIME))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    notaEnviada[sensor] = false;
    //Serial.printf("%d  Nota:  %s, ON -> NEW NOTE \n",millis(), NOTAS[sensor]);
    //sendMIDI(NOTE_ON,CONTROL[sensor],127);
  }
  // *** State Machine ***
  
   // Once the note has been detected, must determine its velocity.
  if((noteState[sensor] == NOTE_ON) &&
     (currentTime - detectionTime[sensor]) < MAX_LATENCY && 
     (notaEnviada[sensor] == false))
  {
    // Searching for the peak...
    if ( maxVelocity[sensor] < movingAvVelocityk[sensor])
    {
      maxVelocity[sensor] = movingAvVelocityk[sensor];
    }
    // Peak Found!
    else
    {
      maxVelocity[sensor] = constrain(127 * maxVelocity[sensor] / MAX_VELOCITY[sensor], 0, 127);
      //Serial.printf("%d Nota: %s, Vel: %.2f (p) \n",millis(), NOTAS[sensor],maxVelocity[sensor]);
      sendMIDI(NOTE_ON,CONTROL[sensor],(int)maxVelocity[sensor]);
      notaEnviada[sensor] = true;
    }

  }
  // Si no encuentro la velocidad máxima de la latencia máxima permitida, envío la velocidad actual
  else if ((noteState[sensor] == NOTE_ON) &&
           (currentTime - detectionTime[sensor]) > MAX_LATENCY && 
          (notaEnviada[sensor] == false))
  {
    maxVelocity[sensor] = constrain(127 * maxVelocity[sensor] / MAX_VELOCITY[sensor], 0, 127);
    //Serial.printf("%d Nota: %s, Vel: %.2f (np) \n",millis(), NOTAS[sensor],maxVelocity[sensor]);
    sendMIDI(NOTE_ON,CONTROL[sensor],(int)maxVelocity[sensor]);
    notaEnviada[sensor] = true;
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];

  
  Serial.print(movingAvVelocityk[sensor]);
  Serial.print(",");
  Serial.print(Threshold_ON[sensor]);
  Serial.print(",");
  Serial.print(Threshold_OFF[sensor]);
  Serial.print(",");
  Serial.println(160 * noteState[sensor]);
  
  if (sensor == 0) digitalWrite(PIN_SPEED,LOW);
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
   /*
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
  */
}

void TestMIDI(int repeticiones)
{
  
  Serial.println(F("Testing MIDI Channel"));
  int i = 0;
  int j = 0;
  while(i < repeticiones)
  {
    //sendMIDI(CONTROL_CHANGE,SUSTAIN,127);
    
    //sendMIDI(CONTROL_CHANGE,SUSTAIN,0);
    
    sendMIDI(NOTE_ON,CONTROL[j],127);
    
    sendMIDI(NOTE_ON,CONTROL[j],0);
    j <  3? j++: j=0; 
    Serial.printf("Nota %d\n",i);
    delay(100);
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
/*
// This function is very sensitive to noise
void findNotes2(int sensor)
{
  // SPEED: 65uSeg!!!!
  digitalWrite(GPIO_NUM_23,HIGH);
  static long notasDetectadas = 0;
  const float MIDI_GAIN = 127.0 / 8000.0;
  const int MIDI_MINIMUM_VELOCITY = 40;   // Debajo de este valor no envío la nota.
  const int MAX_LATENCY = 200;
  const float SAMPLETIME = 65e-6;     // La rutina findnotes2 demora 65useg. 
  // TODO: revisar este número
  const float DERIVATIVE_THRESHOLD_LOW = 10000;
  const float DERIVATIVE_THRESHOLD_HIGH = 30000;
  const unsigned long DETECTION_TIME = 100;
 
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
  //derivk[sensor][0] = (movingAvVelocityk[sensor] - movingAvVelocityk_1[sensor]);

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
    //Serial.printf("%d Sensor %s, OFF_ON \n",millis(), NOTAS[sensor]);
    
  }
  // 2. Nota ON -> Nota ON
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvDerivk[sensor] >= DERIVATIVE_THRESHOLD_HIGH) &&
           (currentTime - detectionTime[sensor] >= DETECTION_TIME))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    notaEnviada[sensor] = false;
    //Serial.printf("%d Sensor %s, ON_ON \n",millis(), NOTAS[sensor]);

    
    
  }
  // 3. Nota ON -> Nota OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (currentTime - detectionTime[sensor] >= 6 * DETECTION_TIME))
  {
    noteState[sensor] = NOTE_OFF;
    //Serial.printf("%d Sensor %s, ON_OFF \n",millis(), NOTAS[sensor]);
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
        sendMIDI(NOTE_ON,CONTROL[sensor],(int)(MIDI_GAIN * maxVelocity[sensor]));
        
        
        #if DEBUG
        notasDetectadas++;
        //Serial.printf("t: %d \t Nota N: %d \t Sens: %d \t Vel: %d \n",
        //               millis(), notasDetectadas, sensor, (int)(MIDI_GAIN * maxVelocity[sensor]));
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
      //Serial.println("..");
       if((int)(MIDI_GAIN * maxVelocity[sensor]) > MIDI_MINIMUM_VELOCITY)
       {
        sendMIDI(NOTE_ON,CONTROL[sensor],(int)(MIDI_GAIN * maxVelocity[sensor]));
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

  //Serial.print(amplik[sensor][0]);
  //Serial.print(",");
  //Serial.println(movingAvDerivk[sensor]);  

  digitalWrite(GPIO_NUM_23,LOW);    
}
*/