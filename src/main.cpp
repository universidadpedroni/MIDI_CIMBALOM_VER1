#include <Arduino.h>
#include <config.h>
#include <midiNotesChart.h>
#include <legends.h>
#include <Streaming.h>
#include <blink.h>
#include <findNotes3Options.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "debugConstants.h"

// OTA con Access Point
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>


// https://forum.arduino.cc/t/reading-piezo-for-midi-note-velocity/69277
// https://iq.opengenus.org/bitwise-division/
// https://ccrma.stanford.edu/~craig/articles/linuxmidi/misc/essenmidi.html
// Lo mejor está en un PDF que se llama Arduino-midi.pdf
// CABLE MIDI. AMARILL=,VCC. BLANCO, DATO
// https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/
// https://randomnerdtutorials.com/decoding-and-encoding-json-with-arduino-or-esp8266/

blink ledBlink(PIN_LED);

void GPIOSetup();
void sendMIDI(int cmd, int data1, int data2);
void sustainPedalUpdate();
void TestMIDI(int repeticiones);
void findNotes3(int sensor);
void findNotes3NoVelocity(int sensor);
void plotSignalAndThresholds(float Velocity, int noteState, float Threshold_ON, float Threshold_OFF);
bool CargarConfiguracionDesdeSPIFF();
bool GuardarConfiguracionEnSPIFF();
void showConfig();
void OverTheAirUpdate();
void plotSignalsAndSetup();
void ComandosDisponibles();

const char* host = "esp32";
const char* ssid = "MyEspUpdater";

void setup()
{
  // Initial delay
  delay(2000);
  // GPIO Setup
  GPIOSetup();
  ledBlink.init();
  
  // Serial ports setup
  Serial.begin(BAUDRATE);
  Serial.flush();
  SerialMidi.begin(MIDI_BAUDRATE);
  SerialMidi.flush();
  
  // ADC Setup
  analogReadResolution(16);

  
  
  Debug(FIRMWARE); Debug(COMPILATION_DATE); Debug(__DATE__); Debug(COMMA); Debugln(__TIME__);
  Debug(HARDWARE);
  Debugf(CORE_USED, xPortGetCoreID());
  ComandosDisponibles();
  
  if (!digitalRead(PIN_OTA))
  {
    Debug(STARTING_OTA_UPDATE);
    OverTheAirUpdate();
  }

  

  bool ConfigCargadaDesdeSPIFF = CargarConfiguracionDesdeSPIFF();
  if (ConfigCargadaDesdeSPIFF)
  {
    Debug(CONFIG_FROM_JSON);
  }
  else
  {
    Debug(LOAD_DEFAULTS);
    // TODO : Cargar config por defecto
    
  }

  //TODO: Enter Setup mode to adjust threshold and release time when button is pressed
  
    // TODO Setup Function
  
  

  // SETUP y OTA quedaron con lógica negativa.
  if(!digitalRead(PIN_SETUP))
  {
    
  }
  plotSignalsAndSetup();
  


  if(DEBUG) showConfig();
  
  //GuardarConfiguracionEnSPIFF();
  if (TEST_MIDI) TestMIDI(4);


  //digitalWrite(PIN_LED,LOW);
  
  Debug(SETUP_FINISHED);
  
}

void loop()
{
  ledBlink.update(LED_INTERVAL);
  sustainPedalUpdate();
  findNotes3NoVelocity(0);
  findNotes3NoVelocity(1);
  findNotes3NoVelocity(2);
  findNotes3NoVelocity(3);
  findNotes3NoVelocity(4);

  /*
  for (int i = 0; i < NUM_SENSORES; i++)
  {
    findNotes3NoVelocity(i);

  }
  */
     
}

void plotSignalsAndSetup(){
  const int MIN_VALUE = 500;
  int midiTest = 0;
  unsigned long initialMillis = 0;
  uint16_t detection = 0;      // Variable usada para mostrar el detection time en el plot

  while (!digitalRead(PIN_SETUP))
  {
    
    
    
    // Chequear si hay algo en el serial port.
    if (Serial.available() > 0)
    {
      char comando = toupper(Serial.read());
      switch (comando)
      {
        case 'C':   // Midi Channel
          MIDI_CHANNEL = Serial.parseInt();
          MIDI_CHANNEL = constrain(MIDI_CHANNEL,1,16);
          break;
        case 'T':   // ThresHold ON
          Threshold_ON = Serial.parseFloat();
          constrain(Threshold_ON, 0, 60000);
          break;
        case 'F':   // ThresHold OFF
          Threshold_OFF = Serial.parseFloat();
          Threshold_OFF = constrain(Threshold_OFF, 0.0, 0.8 * Threshold_ON);
          break;
        case 'D':   // Detection time
          Detection_Time = (unsigned long) Serial.parseInt();
          Detection_Time = constrain(Detection_Time,0,1000);
          break;
        case 'M':   // Test midi
          midiTest = Serial.parseInt();
          midiTest = constrain(midiTest,0,1);
          midiTest == 1? TEST_MIDI = true : TEST_MIDI = false;
          break;
        default:
          break;
      }
      GuardarConfiguracionEnSPIFF();

    }
    // Chequear si se tocó una cuerda
    if((analogRead(CANALES_ADC[0]) > MIN_VALUE) || (analogRead(CANALES_ADC[1]) > MIN_VALUE) ||
        (analogRead(CANALES_ADC[2]) > MIN_VALUE) || (analogRead(CANALES_ADC[3]) > MIN_VALUE) ||
        (analogRead(CANALES_ADC[4]) > MIN_VALUE))
        {
          initialMillis = millis();
          Serial.println("Th_ON Th_OFF S0 S1 S2 S3 S4, Det");
          for(int i = 0; i < 300; i++)
          {
            millis() - initialMillis < Detection_Time ? detection = (uint16_t) (Threshold_ON / 2) : detection = 0;
            Serial.printf("%d, %d, %d, %d, %d, %d, %d, %d \n",
                    (uint16_t)Threshold_ON,
                    (uint16_t) Threshold_OFF,
                    analogRead(CANALES_ADC[0]),
                    analogRead(CANALES_ADC[1]),
                    analogRead(CANALES_ADC[2]),
                    analogRead(CANALES_ADC[3]),
                    analogRead(CANALES_ADC[4]),
                    detection);
      
       
          }
          delay(2000);  // una manera muy mala para que no entre dos veces en el bucle porque detecta la cola de la señal mayor a 500

        }

  ledBlink.update(LED_INTERVAL_SETUP);  
  }


}

void ComandosDisponibles()
{
  Serial << F("List of available commands") << CR;
  Serial << F("C,n - Midi Channel [1-16]") << CR; 
  Serial << F("T,n - Threshold oN [0, 60000]") << CR;
  Serial << F("F,n - Threshold OFF [0, 0.8 * Threshold on]") << CR;
  Serial << F("D,n - Detection Time [0, 1000]") << CR;
  Serial << F("M,n - Test Midi [0,1]") << CR;

}

void OverTheAirUpdate()
{
  AsyncWebServer server(80);
  
  WiFi.mode(WIFI_AP);
  //WiFi.softAP("MidiESP","12345678");
  WiFi.softAP("MidiESP");
  IPAddress myIP = WiFi.softAPIP();
  Serial << F("To upload, go to: ") << WiFi.softAPIP() << F("/update\n");
  
  server.on("/hello", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/html", "<p>Hello world!</p>");
  });
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();

  

  while(1)
  {
    
    ledBlink.update(LED_INTERVAL_OTA);
    delay(5);

  }

  
}


void showConfig()
{
  Serial.print(SYSTEM_CONFIG);
  Serial.printf("Midi Channel Tx: %d \n", MIDI_CHANNEL);
  for (int i = 0; i < NUM_SENSORES; i ++)
  {
    Serial << SENSOR << i << TAB << NOTE << CONTROL[i] << TAB << NOTAS[CONTROL[i]] << CR;
  }  
  Serial << THRES_ON << Threshold_ON << TAB << THRES_OFF << Threshold_OFF << TAB;
  Serial << F("Detection Interval: ") << TAB << Detection_Time << F("ms") << CR;
  Serial << F("Test MIDI: ");
  TEST_MIDI == true? Serial.println("Yes") : Serial.println("No");
  Serial << VEL_MAX << Max_Velocity << CR;
  

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
  else{
    if(DEBUG) Serial.printf("Reading config file: %s \n", JSON_FILE_NAME);
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
    MIDI_CHANNEL = (int)doc["MIDI_CHANNEL"];
    Threshold_ON = (float)doc["THRESHOLD_ON"];
    Threshold_OFF = (float)doc["THRESHOLD_OFF"];
    Max_Velocity = (float)doc["MAX_VELOCITY"];
    Detection_Time = (unsigned long) doc["DETECTION_TIME"];
    TEST_MIDI = (bool) doc["TEST_MIDI"];
    
    if(MIDI_CHANNEL < 1 || MIDI_CHANNEL > 16 ) MIDI_CHANNEL = MIDI_CHANNEL_DEFAULT;
    if(Threshold_ON < 0 || Threshold_ON > 20000) Threshold_ON = THRESHOLD_ON_DEFAULT;
    if(Threshold_OFF < 0 || Threshold_OFF > Threshold_ON) Threshold_OFF = 0.5 * Threshold_ON;
    if(Detection_Time < 0) Detection_Time = DETECTION_TIME_DEFAULT;

    
    for (int i = 0; i < NUM_SENSORES; i++)
    {
      CONTROL[i] = (int)doc["CONTROL"][i];
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
  }
  doc["MIDI_CHANNEL"] = MIDI_CHANNEL;
  doc["THRESHOLD_ON"] = Threshold_ON;
  doc["THRESHOLD_OFF"] = Threshold_OFF;
  doc["MAX_VELOCITY"] = Max_Velocity;
  doc["DETECTION_TIME"] = Detection_Time;
  TEST_MIDI == true? doc["TEST_MIDI"] = 1 : doc["TEST_MIDI"] = 0;
  
  
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
     (movingAvVelocityk[sensor] < Threshold_ON))
     {
        // Nothing to do here
     }
  // 2. NOTE OFF - NOTE ON   
  else if ((noteState[sensor] == NOTE_OFF) && 
           (movingAvVelocityk[sensor] > Threshold_ON))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    if (DEBUG && !PLOT_SIGNALS) Serial.printf("%lu Note:  %s, OFF -> ON, Amplitud: %.2f \n",millis() / 1000, NOTAS[CONTROL[sensor]], movingAvVelocityk[sensor]);
    
    sendMIDI(NOTE_ON,CONTROL[sensor],127);
  }
  // 3. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] < Threshold_OFF))
  {
    noteState[sensor] = NOTE_OFF;
    if (DEBUG && !PLOT_SIGNALS) Serial.printf("%lu  Note:  %s, ON -> OFF, Amplitud: %.2f \n",millis() / 1000, NOTAS[CONTROL[sensor]], movingAvVelocityk[sensor]);
    sendMIDI(NOTE_OFF,CONTROL[sensor],0);
  }
  // 4. NOTE ON - NEW NOTE
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] > Threshold_ON) &&
           (currentTime - detectionTime[sensor] >= Detection_Time))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    if (DEBUG && !PLOT_SIGNALS) Serial.printf("%lu  Note:  %s, ON -> NEW NOTE, Amplitud: %.2f \n",millis() / 1000, NOTAS[CONTROL[sensor]], movingAvVelocityk[sensor]); 
    
    sendMIDI(NOTE_ON,CONTROL[sensor],127);
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];

  // Plot Sensor 2 and 3.
  if (PLOT_SIGNALS)
  {
    plotSignalAndThresholds(movingAvVelocityk[plotSensor],noteState[plotSensor],Threshold_ON,Threshold_OFF);
  }
  
  
  if (sensor == 0) digitalWrite(PIN_SPEED,LOW);
}

void plotSignalAndThresholds(float Velocity, int noteState, float Threshold_ON, float Threshold_OFF)
{
    Serial << Velocity << COMMA << 160 * noteState << COMMA << Threshold_ON << COMMA << Threshold_OFF << CR;
    
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
     (movingAvVelocityk[sensor] < Threshold_ON))
     {

     }
  // 2. NOTE OFF - NOTE ON   
  else if ((noteState[sensor] == NOTE_OFF) && 
           (movingAvVelocityk[sensor] > Threshold_ON))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    notaEnviada[sensor] = false;
    //Serial.printf("%d Nota:  %s, OFF -> ON \n",millis(), NOTAS[sensor]);
    //sendMIDI(NOTE_ON,CONTROL[sensor],127);
  }
  // 3. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] < Threshold_OFF))
  {
    noteState[sensor] = NOTE_OFF;
    Serial.printf("%lu  Nota:  %s, ON -> OFF \n",millis(), NOTAS[CONTROL[sensor]]);
    //sendMIDI(NOTE_OFF,CONTROL[sensor],0);
  }
  // 4. NOTE ON - NEW NOTE
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] > Threshold_ON) &&
           (currentTime - detectionTime[sensor] >= Detection_Time))
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
      maxVelocity[sensor] = constrain(127 * maxVelocity[sensor] / Max_Velocity, 0, 127);
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
    maxVelocity[sensor] = constrain(127 * maxVelocity[sensor] / Max_Velocity, 0, 127);
    //Serial.printf("%d Nota: %s, Vel: %.2f (np) \n",millis(), NOTAS[sensor],maxVelocity[sensor]);
    sendMIDI(NOTE_ON,CONTROL[sensor],(int)maxVelocity[sensor]);
    notaEnviada[sensor] = true;
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];

  if (PLOT_SIGNALS)
  {
    plotSignalAndThresholds(movingAvVelocityk[plotSensor],noteState[plotSensor],Threshold_ON,Threshold_OFF);
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
  
  Serial.println(F("Testing MIDI Channel by sending notes"));
  int i = 0;
  int j = 0;
  while(i < repeticiones)
  {
    sendMIDI(NOTE_ON,CONTROL[j],127);
    
    sendMIDI(NOTE_ON,CONTROL[j],0);
    j <  3? j++: j=0; 
    Serial.printf("Sending Note N° %d, MIDI code %d, Note %s\n",i, CONTROL[j],NOTAS[CONTROL[j]]);
    delay(100);
    i++;
  }
  Serial.println("Midi Test Finished");
}

void GPIOSetup()
{
  // Led PIN
  pinMode(PIN_LED,OUTPUT);
  
  // Sustain Pedal
  pinMode(PIN_SUSTAIN_PEDAL,INPUT_PULLUP);
  // Setup PIN
  pinMode(PIN_SETUP, INPUT_PULLUP);
  // OTA PIN
  pinMode(PIN_OTA, INPUT_PULLUP);
  // Speed measurement pin
  pinMode(PIN_SPEED,OUTPUT);

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



