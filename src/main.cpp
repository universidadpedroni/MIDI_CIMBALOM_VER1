#include <Arduino.h>
#include <config.h>
#include <blink.h>
#include <findNotes3Options.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "debugConstants.h"
#include <auxFunc.h>

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


void findNotes3(int sensor);
void findNotes3NoVelocity(int sensor);
void plotSignalAndThresholds(float Velocity, int noteState, float Threshold_ON, float Threshold_OFF);
//void plotSignalsAndSetup();


const char* host = "esp32";
const char* ssid = "MyEspUpdater";

void OverTheAirUpdate()
{
  AsyncWebServer server(80);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP("MidiESP");
  IPAddress myIP = WiFi.softAPIP();
  Serial.print(F("To upload, go to:"));
  Serial.print(WiFi.softAPIP());
  Serial.println(F("/update\n"));
  //Serial << F("To upload, go to: ") << WiFi.softAPIP() << F("/update\n");
  
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

bool GuardarConfiguracionEnSPIFF()
{
  // Nombre del archivo: config.json
  if(!SPIFFS.begin(true))
  {
    Serial.println(F("SPIFFS error"));
    return false; 
  }
  File config = SPIFFS.open(JSON_FILE_NAME,"w");
  if(!config)
  {
    Serial.println(F("Error opening config file."));
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
  doc["GAIN_VELOCITY"] = Gain_Velocity;
  doc["DETECTION_TIME"] = Detection_Time;
  doc["DURATION_VELOCITY"] = Duration_Velocity;
  TEST_MIDI == true? doc["TEST_MIDI"] = 1 : doc["TEST_MIDI"] = 0;
 
  // Serialize JSON to file
  serializeJson(doc,config);
  config.close();
  return true;
}

void serialCommands()
{
  int midiTest = 0;
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
        case 'D':   // Detection time
          Detection_Time = (unsigned long) Serial.parseInt();
          Detection_Time = constrain(Detection_Time,0,1000);
          break;
        case 'F':   // ThresHold OFF
          Threshold_OFF = Serial.parseFloat();
          Threshold_OFF = constrain(Threshold_OFF, 0.0, 0.8 * Threshold_ON);
          break;
        case 'G': // Velocity Gain
          Gain_Velocity = Serial.parseFloat();
          Gain_Velocity = constrain(Gain_Velocity, 1, 10);
        case 'M':   // Test midi
          midiTest = Serial.parseInt();
          midiTest = constrain(midiTest,0,1);
          midiTest == 1? TEST_MIDI = true : TEST_MIDI = false;
          break;
        case 'T':   // ThresHold ON
          Threshold_ON = Serial.parseFloat();
          constrain(Threshold_ON, 0, 200);
          break;
        default:
          break;
      }
      GuardarConfiguracionEnSPIFF();
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
      sendMIDI(SerialMidi, CONTROL_CHANGE,SUSTAIN,127);;
    }
    else
    {
      sendMIDI(SerialMidi, CONTROL_CHANGE,SUSTAIN,0);
    }
    sustainlastState = sustainCurrentState;
  }
}

bool CargarConfiguracionDesdeSPIFF()
{
  // Nombre del archivo: config.json
  if(!SPIFFS.begin(true))
  {
    Serial.println(F("SPIFFS error"));
    return false; 
  }
  File config = SPIFFS.open(JSON_FILE_NAME,"r");
  if(!config)
  {
    Serial.println(F("Error opening config file."));
    return false;
  }
  else{
    Serial.printf("Reading config file: %s \n", JSON_FILE_NAME);
  }
  
  // Create
  DynamicJsonDocument doc(2048);
  // Deserialize the Json Doc.
  DeserializationError error = deserializeJson(doc, config);
  if (error)
  {
    Serial.println(F("Error opening config file."));
    return false;
  }
  else
  {
    Serial.println(F("Loading config from json File"));
    MIDI_CHANNEL = (int)doc["MIDI_CHANNEL"];
    Threshold_ON = (float)doc["THRESHOLD_ON"];
    Threshold_OFF = (float)doc["THRESHOLD_OFF"];
    Max_Velocity = (float)doc["MAX_VELOCITY"];
    Gain_Velocity = (float)doc["GAIN_VELOCITY"];
    Duration_Velocity = (int)doc["DURATION_VELOCITY"];
    Detection_Time = (unsigned long) doc["DETECTION_TIME"];
    TEST_MIDI = (bool) doc["TEST_MIDI"];
    
    if(MIDI_CHANNEL < 1 || MIDI_CHANNEL > 16 ) MIDI_CHANNEL = MIDI_CHANNEL_DEFAULT;
    if(Threshold_ON < 0 || Threshold_ON > 200) Threshold_ON = THRESHOLD_ON_DEFAULT;
    if(Threshold_OFF < 0 || Threshold_OFF > Threshold_ON) Threshold_OFF = 0.5 * Threshold_ON;
    if(Detection_Time < 0) Detection_Time = DETECTION_TIME_DEFAULT;
    if(Gain_Velocity < 1) Gain_Velocity = GAIN_VELOCITY_DEFAULT;
    if(Duration_Velocity < 1 || Duration_Velocity > 100) Duration_Velocity = DURATION_VELOCITY_DEFAULT;
    
    
    for (int i = 0; i < NUM_SENSORES; i++)
    {
      CONTROL[i] = (int)doc["CONTROL"][i];
    }
  
  }

  config.close();

  return true;

}



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
  

  SerialPresentation(Serial);
  SerialComandosDisponibles(Serial);
  SerialShowConfigSensores(Serial, NUM_SENSORES, CONTROL);
  // OTA
  if (!digitalRead(PIN_OTA))
  {
    Serial.println(F("Starting OTA Update"));
    OverTheAirUpdate();
  }

  // Load Config from SPIFF
  bool ConfigCargadaDesdeSPIFF = CargarConfiguracionDesdeSPIFF();
  if (ConfigCargadaDesdeSPIFF)
  {
    Serial.println(F("Config loaded from json file"));
  }
  else
  {
    Serial.println(F("Loading Default Config"));
    // TODO : Cargar config por defecto
    
  }

  //TODO: Enter Setup mode to adjust threshold and release time when button is pressed
  
    // TODO Setup Function
  SerialShowConfig(Serial, MIDI_CHANNEL,
                  Threshold_ON, Threshold_OFF,
                  Detection_Time, Gain_Velocity,
                  Duration_Velocity,
                  TEST_MIDI);
  
  
  if (TEST_MIDI) TestMIDI(Serial, SerialMidi, 4);
  SerialSetupFinished(Serial);
  /*
  float medicion = 0.0;
  int ajuste = 0.0;
  while(1){
    medicion = (float)analogRead(GPIO_NUM_36) - 6000;
    ajuste = (int) (Gain_Velocity * medicion / Max_Velocity); 
    Serial.printf("%.2f,  %d\n", medicion, ajuste);
    delay(500);

  }
  */
  
}

void loop()
{
  ledBlink.update(LED_INTERVAL);
  sustainPedalUpdate();
  findNotes3(0);
  serialCommands();
  /*
  //for (int i = 0; i < NUM_SENSORES; i++)
  for (int i = 0; i < 5; i++)
  {
    findNotes3NoVelocity(i);

  }
  */
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
    if (PLOT_SIGNALS) Serial.printf("%lu Note:  %s, OFF -> ON, Amplitud: %.2f \n",millis() / 1000, NOTAS[CONTROL[sensor]], movingAvVelocityk[sensor]);
    
    sendMIDI(SerialMidi,NOTE_ON,CONTROL[sensor],127);
  }
  // 3. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] < Threshold_OFF))
  {
    noteState[sensor] = NOTE_OFF;
    if (PLOT_SIGNALS) Serial.printf("%lu  Note:  %s, ON -> OFF, Amplitud: %.2f \n",millis() / 1000, NOTAS[CONTROL[sensor]], movingAvVelocityk[sensor]);
    sendMIDI(SerialMidi,NOTE_OFF,CONTROL[sensor],0);
  }
  // 4. NOTE ON - NEW NOTE
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] > Threshold_ON) &&
           (currentTime - detectionTime[sensor] >= Detection_Time))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    if (PLOT_SIGNALS) Serial.printf("%lu  Note:  %s, ON -> NEW NOTE, Amplitud: %.2f \n",millis() / 1000, NOTAS[CONTROL[sensor]], movingAvVelocityk[sensor]); 
    
    sendMIDI(SerialMidi,NOTE_ON,CONTROL[sensor],127);
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];

   
  
  if (sensor == 0) digitalWrite(PIN_SPEED,LOW);
}


void findNotes3(int sensor)
{
  if (sensor == 0) digitalWrite(PIN_SPEED,HIGH);
  unsigned long currentTime = millis();
  
  // Reading the amplitude of the signal and going through moving average filter
  amplik[sensor][0] =  Gain_Velocity * (float)analogRead(CANALES_ADC[sensor]) / Max_Velocity;
  movingAvVelocityk[sensor] = (amplik[sensor][0] + amplik[sensor][1] + amplik[sensor][2]) / 3;
  //movingAvVelocityk[sensor] = amplik[sensor][0];

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
    maxVelocity[sensor] = movingAvVelocityk[sensor];
    notaEnviada[sensor] = false;
    contadorMax[sensor] = 0;
    if(PLOT_SIGNALS) SerialShowSignalsLabels(Serial);
    
  }
  // 3. NOTE ON - NOTE ON, No Max
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] > Threshold_ON) &&
           (currentTime - detectionTime[sensor] <= Detection_Time))
  {
    if (maxVelocity[sensor] < movingAvVelocityk[sensor])
    {
      maxVelocity[sensor] = movingAvVelocityk[sensor];
    }
    else
    {
      contadorMax[sensor]++;
    } 
    detectionTime[sensor] = currentTime;
    if (contadorMax[sensor] >= Duration_Velocity && notaEnviada[sensor] == false)
    {
      //sendMIDI(SerialMidi,NOTE_ON,CONTROL[sensor],constrain(maxVelocity[sensor],  0, 256));
      sendMIDI(SerialMidi,NOTE_ON,CONTROL[sensor],(int)maxVelocity[sensor]);
      if(SHOW_MIDI_MESSAGE_SENT) ShowMidiMessage(Serial, NOTE_ON, CONTROL[sensor],(int)constrain(maxVelocity[sensor],  0.0, 256.0) );
      notaEnviada[sensor] = true;
      if(PLOT_SIGNALS) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],100); 
    }
    else
    {
      if(PLOT_SIGNALS) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],0); 
    }
    
 
  }
  
  // 5. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           ((movingAvVelocityk[sensor] < Threshold_OFF) ||
           (currentTime - detectionTime[sensor] >= Detection_Time)))
  {
    noteState[sensor] = NOTE_OFF;
    maxVelocity[sensor] = 0;
    sendMIDI(SerialMidi, NOTE_OFF,CONTROL[sensor],0);
    if(SHOW_MIDI_MESSAGE_SENT) ShowMidiMessage(Serial, NOTE_ON, CONTROL[sensor],0);
    if(PLOT_SIGNALS) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],0);  
       
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];
  
  
  if (sensor == 0) digitalWrite(PIN_SPEED,LOW);

}















