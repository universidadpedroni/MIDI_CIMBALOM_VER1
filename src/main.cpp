#include <Arduino.h>
#include <config.h>
#include <findNotes3Options.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "debugConstants.h"
#include <auxFunc.h>

// OTA con Access Point
#include <wifiFunctions.h>
//#include <WiFi.h>
//#include <ESPAsyncWebServer.h>
//#include <AsyncElegantOTA.h>
//#include "wifiConfig.h"

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// TODO: ESP32 as slave
// Wire Master Reader
// by Gutierrez PS <https://github.com/gutierrezps>
// ESP32 I2C slave library: <https://github.com/gutierrezps/ESP32_I2C_Slave>
// based on the example by Nicholas Zambetti <http://www.zambetti.com>

// WIFI
// https://www.upesy.com/blogs/tutorials/how-create-a-wifi-acces-point-with-esp32

// https://forum.arduino.cc/t/reading-piezo-for-midi-note-velocity/69277
// https://iq.opengenus.org/bitwise-division/
// https://ccrma.stanford.edu/~craig/articles/linuxmidi/misc/essenmidi.html
// Lo mejor está en un PDF que se llama Arduino-midi.pdf
// CABLE MIDI. AMARILL=,VCC. BLANCO, DATO
// https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/
// https://randomnerdtutorials.com/decoding-and-encoding-json-with-arduino-or-esp8266/



void findNotes3(int sensor);
void findNotes3NoVelocity(int sensor);
//void plotSignalAndThresholds(float Velocity, int noteState, float Threshold_ON, float Threshold_OFF);

TaskHandle_t presentacionPorSerieTaskHandle;


void presentacionPorSerieTask(void *parameter){
  SerialPresentation(Serial, softwareVersion, jsonVersion);
  SerialShowActiveSensors(Serial, sensoresActivos);
  SerialComandosDisponibles(Serial);
  SerialShowConfig(Serial, MIDI_CHANNEL,
                  Threshold_ON, Threshold_OFF,
                  Detection_Time, Gain_Velocity,
                  Duration_Velocity,
                  TEST_MIDI);
  //Serial.printf("Tarea ejecutándose en el núcleo %d\n", xPortGetCoreID());
  SerialShowConfigSensores(Serial, NUM_SENSORES, CONTROL);
  SerialSetupFinished(Serial);
  vTaskDelete(NULL);
}

void crearTareaDePresentacionPorSerie(){
  xTaskCreatePinnedToCore(presentacionPorSerieTask,             // Función de la tarea
                          "Presentacion Por Serie",             // Nombre de la tarea (opcional)
                            10000,                              // Tamaño de la pila de la tarea (ajustar según necesidad)
                            NULL,                               // Parámetro de la tarea (puede ser NULL)
                            1,                                  // Prioridad de la tarea (mayor valor = mayor prioridad)
                            &presentacionPorSerieTaskHandle,    // Manejador de la tarea (para detenerla o suspenderla si es necesario)
                            1);                                 // Núcleo en el que se ejecutará la tarea (0 para el primer núcleo, 1 para el segundo)

}

void GPIOSetup()
{
  // Sustain Pedal
  pinMode(PIN_SUSTAIN_PEDAL,INPUT_PULLUP);
  // OTA PIN
  pinMode(PIN_OTA, INPUT_PULLUP);
  // Pin para determinar qué función se usa para detectar las notas.
  pinMode(PIN_NO_DETECTAR_VELOCIDAD, INPUT_PULLUP);
  // Pines para la selección de la octava.
  pinMode(PIN_OCT_SEL_0, INPUT_PULLUP);
  pinMode(PIN_OCT_SEL_1, INPUT_PULLUP);
  pinMode(PIN_OCT_SEL_2, INPUT_PULLUP);
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
  doc["JSON_VERSION"] = jsonVersion;

 
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
      Serial.printf("Comando recibido: %c\n", comando);
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
          break;
        case 'H': // show help
          SerialComandosDisponibles(Serial);
          break;
        case 'M':   // Test midi
          midiTest = Serial.parseInt();
          midiTest = constrain(midiTest,0,1);
          midiTest == 1? TEST_MIDI = true : TEST_MIDI = false;
          break;
        case 'P':   // Plot Signals
          plotSignals = !plotSignals;
          plotSignals? Serial.println("Plot Signals ON") : Serial.println("Plot Signals OFF");
          break;
        case 'T':   // ThresHold ON
          Threshold_ON = Serial.parseFloat();
          Threshold_ON = constrain(Threshold_ON, 0, 200);
          break;
        case 'S': // Working sensors. Usada para determinar con cuantos sensores se trabajará
          sensoresActivos = Serial.parseInt();
          sensoresActivos = constrain(sensoresActivos, 1, NUM_SENSORES);
          SerialShowActiveSensors(Serial, sensoresActivos);
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
    jsonVersion = (int)doc["JSON_VERSION"];
    MIDI_CHANNEL = (int)doc["MIDI_CHANNEL"];
    Threshold_ON = (float)doc["THRESHOLD_ON"];
    Threshold_OFF = (float)doc["THRESHOLD_OFF"];
    Max_Velocity = (float)doc["MAX_VELOCITY"];
    Gain_Velocity = (float)doc["GAIN_VELOCITY"];
    Duration_Velocity = (int)doc["DURATION_VELOCITY"];
    Detection_Time = (unsigned long) doc["DETECTION_TIME"];
    TEST_MIDI = (bool) doc["TEST_MIDI"];
    
    if(MIDI_CHANNEL < 1 || MIDI_CHANNEL > 16 ) MIDI_CHANNEL = MIDI_CHANNEL_DEFAULT;
    if(Threshold_ON < 0.0 || Threshold_ON > 200.0) Threshold_ON = THRESHOLD_ON_DEFAULT;
    if(Threshold_OFF < 0.0 || Threshold_OFF > Threshold_ON) Threshold_OFF = 0.5 * Threshold_ON;
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
  // Serial ports setup
  Serial.begin(BAUDRATE);
  Serial.flush();
  Serial.println(F("Starting Init..."));
  SerialMidi.begin(MIDI_BAUDRATE);
  SerialMidi.flush();
  // ADC Setup
  analogReadResolution(16);
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
  
  
  // La presentación por el puerto serie se realiza en el segundo núcleo.
  crearTareaDePresentacionPorSerie();
  if (TEST_MIDI) TestMIDI(Serial, SerialMidi, 4);

}

void loop()
{
  static int octavaOld = -99;
  int octava = findOctave(PIN_OCT_SEL_0, PIN_OCT_SEL_1, PIN_OCT_SEL_2);
  if(octava != octavaOld){
    Serial.printf("Octava Seleccionada: %d\n", octava);
    octavaOld = octava;
  }
  sustainPedalUpdate();
  serialCommands();
  
  //for (int i = 0; i < NUM_SENSORES; i++)
  for (int i = 0; i < sensoresActivos; i++)
  {
    //if (i == 0) digitalWrite(PIN_SPEED,HIGH);
    digitalRead(PIN_NO_DETECTAR_VELOCIDAD) == true? findNotes3NoVelocity(i):findNotes3(i);
    //if (i == 0) digitalWrite(PIN_SPEED,LOW);
    
  }
    
}


// FUNCIONANDO, SIN VELOCIDAD
void findNotes3NoVelocity(int sensor)
{
  
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
    if (plotSignals) Serial.printf("%lu Note:  %s, OFF -> ON, Amplitud: %.2f \n",millis() / 1000, NOTAS[CONTROL[sensor]], movingAvVelocityk[sensor]);
    
    sendMIDI(SerialMidi,NOTE_ON,CONTROL[sensor],127);
  }
  // 3. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] < Threshold_OFF))
  {
    noteState[sensor] = NOTE_OFF;
    if (plotSignals) Serial.printf("%lu  Note:  %s, ON -> OFF, Amplitud: %.2f \n",millis() / 1000, NOTAS[CONTROL[sensor]], movingAvVelocityk[sensor]);
    sendMIDI(SerialMidi,NOTE_OFF,CONTROL[sensor],0);
  }
  // 4. NOTE ON - NEW NOTE
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] > Threshold_ON) &&
           (currentTime - detectionTime[sensor] >= Detection_Time))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    if (plotSignals) Serial.printf("%lu  Note:  %s, ON -> NEW NOTE, Amplitud: %.2f \n",millis() / 1000, NOTAS[CONTROL[sensor]], movingAvVelocityk[sensor]); 
    
    sendMIDI(SerialMidi,NOTE_ON,CONTROL[sensor],127);
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];

   
  
  
}


void findNotes3(int sensor)
{
  
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
    if(plotSignals) SerialShowSignalsLabels(Serial);
    
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
      if(plotSignals) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],100); 
    }
    else
    {
      if(plotSignals) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],0); 
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
    if(plotSignals) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],0);  
       
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];
  
  
  

}















