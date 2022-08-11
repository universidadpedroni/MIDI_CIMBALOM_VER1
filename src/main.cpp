#include <Arduino.h>
#include <config.h>
#include <midiNotesChart.h>
#include <legends.h>
#include <Streaming.h>
#include <blink.h>
#include <findNotes3Options.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>


// https://forum.arduino.cc/t/reading-piezo-for-midi-note-velocity/69277
// https://iq.opengenus.org/bitwise-division/
// https://ccrma.stanford.edu/~craig/articles/linuxmidi/misc/essenmidi.html
// Lo mejor está en un PDF que se llama Arduino-midi.pdf
// CABLE MIDI. AMARILL=,VCC. BLANCO, DATO
// https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/
// https://randomnerdtutorials.com/decoding-and-encoding-json-with-arduino-or-esp8266/

blink ledBlink(PIN_LED);


void sendMIDI(int cmd, int data1, int data2);
void sustainPedalUpdate();
void TestMIDI(int repeticiones);
void findNotes3(int sensor);
void findNotes3NoVelocity(int sensor);
//void sendADCtoSerial(unsigned long interval, uint32_t delayTime);
void plotSignalAndThresholds(float Velocity, int noteState, float Threshold_ON, float Threshold_OFF);
bool CargarConfiguracionDesdeSPIFF();
bool GuardarConfiguracionEnSPIFF();
void showConfig();



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
    Serial << FIRMWARE << COMPILATION_DATE << __DATE__ << COMMA << __TIME__ << CR << HARDWARE ;
    Serial << CORE_USED << xPortGetCoreID() << CR;
  }
 
  bool ConfigCargadaDesdeSPIFF = CargarConfiguracionDesdeSPIFF();
  if (ConfigCargadaDesdeSPIFF)
  {
    if(DEBUG) Serial << CONFIG_FROM_JSON;
  }
  else
  {
    if(DEBUG) Serial << LOAD_DEFAULTS;
    // TODO : Cargar config por defecto
    
  }
  if(DEBUG) showConfig();
  
  // GuardarConfiguracionEnSPIFF();
  

//TODO: Enter Setup mode to adjust threshold and release time when button is pressed
  delay(1000);
  // TODO: Move LED To another pin
  digitalWrite(LED_BUILTIN,LOW);
  ledBlink.init();
  if (DEBUG)Serial << SETUP_FINISHED;
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

void showConfig()
{
  Serial << SYSTEM_CONFIG;
  for (int i = 0; i < NUM_SENSORES; i ++)
  {
    Serial << CHANNEL << i << TAB << NOTE << CONTROL[i] << TAB << NOTAS[CONTROL[i]] << TAB;
    Serial << THRES_ON << Threshold_ON[i] << TAB << THRES_OFF << Threshold_OFF[i] << TAB;
    Serial << VEL_MAX << MAX_VELOCITY[i] << CR;
  }

}

bool CargarConfiguracionDesdeSPIFF()
{
  // Nombre del archivo: config.json
  if(!SPIFFS.begin(true))
  {
    if(DEBUG) Serial << SPIFFS_ERROR;
    return false; 
  }
  File config = SPIFFS.open(JSON_FILE_NAME,"r");
  if(!config)
  {
    if(DEBUG) Serial.println();
    return false;
  }
  
  // Create
  DynamicJsonDocument doc(2048);
  // Deserialize the Json Doc.
  DeserializationError error = deserializeJson(doc, config);
  if (error)
  {
    if(DEBUG) Serial << FILE_ERROR;
    return false;
  }
  else
  {
    if(DEBUG) Serial << LOADING_CONFIG;
    for (int i = 0; i < NUM_SENSORES; i++)
    {
      CONTROL[i] = doc["CONTROL"][i];
      Threshold_ON[i] = doc["Threshold_ON"][i];
      Threshold_OFF[i] = doc["Threshold_OFF"][i];
      MAX_VELOCITY[i] = doc["MAX_VELOCITY"][i];
    }
    
    
  }
  
  config.close();
  return true;

}

bool GuardarConfiguracionEnSPIFF()
{
  // Nombre del archivo: config.json
  if(!SPIFFS.begin(true))
  {
    if(DEBUG) Serial << SPIFFS_ERROR;
    return false; 
  }
  File config = SPIFFS.open(JSON_FILE_NAME,"w");
  if(!config)
  {
    if(DEBUG) Serial << FILE_ERROR;
    return false;
  }
  
  // Create
  DynamicJsonDocument doc(2048);
  // Actualización de Valores
  for (int i = 0; i < NUM_SENSORES; i++)
  {
    doc["CONTROL"][i] = CONTROL[i];
    doc["Threshold_ON"][i] = Threshold_ON[i];
    doc["Threshold_OFF"][i] = Threshold_OFF[i];
    doc["MAX_VELOCITY"][i] = MAX_VELOCITY[i];
  }
  
  // Serialize JSON to file
  serializeJson(doc,config);
  config.close();
  return true;


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
    if (DEBUG) Serial.printf("%lu Note:  %s, OFF -> ON \n",millis(), NOTAS[CONTROL[sensor]]);
    sendMIDI(NOTE_ON,CONTROL[sensor],127);
  }
  // 3. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] < Threshold_OFF[sensor]))
  {
    noteState[sensor] = NOTE_OFF;
    if(DEBUG) Serial.printf("%lu  Note:  %s, ON -> OFF \n",millis(), NOTAS[CONTROL[sensor]]);
    sendMIDI(NOTE_OFF,CONTROL[sensor],0);
  }
  // 4. NOTE ON - NEW NOTE
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] > Threshold_ON[sensor]) &&
           (currentTime - detectionTime[sensor] >= DETECTION_TIME))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    if (DEBUG) Serial.printf("%lu  Note:  %s, ON -> NEW NOTE \n",millis(), NOTAS[CONTROL[sensor]]); 
    
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
    Serial << Velocity << COMMA << 160 * noteState << COMMA << Threshold_ON << COMMA << Threshold_OFF << CR;
    //Serial.print(Velocity); Serial.print(","); Serial.print(160 * noteState); Serial.print(",");
    //Serial.print(Threshold_ON);
    //Serial.print(",");
    //Serial.println(Threshold_OFF);

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
    Serial.printf("%lu  Nota:  %s, ON -> OFF \n",millis(), NOTAS[CONTROL[sensor]]);
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

  if (PLOT_SIGNALS)
  {
    plotSignalAndThresholds(movingAvVelocityk[plotSensor],noteState[plotSensor],Threshold_ON[plotSensor],Threshold_OFF[plotSensor]);
  }
  
  
  if (sensor == 0) digitalWrite(PIN_SPEED,LOW);
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
    Serial.printf("Note %d\n",i);
    delay(100);
    i++;
  }
  
}

/*
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

}*/



