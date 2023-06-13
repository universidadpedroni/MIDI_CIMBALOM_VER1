#include <Arduino.h>

#include <config.h>
#include <findNotes3Options.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "debugConstants.h"
#include <auxFunc.h>

// OTA con Access Point
#include <wifiFunctions.h>

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

TaskHandle_t presentacionPorSerieTaskHandle;

// FUNCIONANDO, SIN VELOCIDAD
void findNotes3(int sensor)
{
  
  unsigned long currentTime = millis();
  
  // Reading the amplitude of the signal and going through moving average filter
  amplik[sensor][0] =  Gain_Velocity * (float)analogRead(CANALES_ADC[sensor]) / Max_Velocity;
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
    maxVelocity[sensor] = movingAvVelocityk[sensor];
    notaEnviada[sensor] = false;
    contadorMax[sensor] = 0;
    if(serialPlotSignals) SerialShowSignalsLabels(Serial);
    
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
      // Se actualizan los valores de cmd, data1 y data2 para los esclavos.
      
      sendMIDI(SerialMidi,NOTE_ON,CONTROL[sensor],(int)maxVelocity[sensor]);
      if(SHOW_MIDI_MESSAGE_SENT) ShowMidiMessage(Serial, NOTE_ON, CONTROL[sensor],(int)constrain(maxVelocity[sensor],  0.0, 256.0) );
      notaEnviada[sensor] = true;
      if(serialPlotSignals) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],100); 
    }
    else
    {
      if(serialPlotSignals) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],0); 
    }
    
 
  }
  
  // 5. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           ((movingAvVelocityk[sensor] < Threshold_OFF) ||
           (currentTime - detectionTime[sensor] >= Detection_Time)))
  {
    noteState[sensor] = NOTE_OFF;
    maxVelocity[sensor] = 0;
    // Se actualizan los valores de cmd, data1 y data2 para los esclavos.
    
    sendMIDI(SerialMidi, NOTE_OFF,CONTROL[sensor],0);
    if(SHOW_MIDI_MESSAGE_SENT) ShowMidiMessage(Serial, NOTE_ON, CONTROL[sensor],0);
    if(serialPlotSignals) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],0);  
       
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];
}

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
    if (serialDebugSignals) SerialDebugSignals(millis(), sensor, movingAvVelocityk[sensor], ESTADO_OFF_ON);
    if(serialPlotSignals) SerialShowSignalsLabels(Serial);
    
    sendMIDI(SerialMidi,NOTE_ON,CONTROL[sensor],MIDI_VEL_TEST);
  }
  // 3. NOTE ON - NOTE OFF
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] < Threshold_OFF))
  {
    noteState[sensor] = NOTE_OFF;
    if (serialDebugSignals) SerialDebugSignals(millis(), sensor, movingAvVelocityk[sensor], ESTADO_ON_OFF);
    
    sendMIDI(SerialMidi,NOTE_OFF,CONTROL[sensor],0);
    if(serialPlotSignals) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],0);  

  }
  // 4. NOTE ON - NEW NOTE
  else if ((noteState[sensor] == NOTE_ON) &&
           (movingAvVelocityk[sensor] > Threshold_ON) &&
           (currentTime - detectionTime[sensor] >= Detection_Time))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    if (serialDebugSignals) SerialDebugSignals(millis(), sensor, movingAvVelocityk[sensor], ESTADO_ON_NEW_NOTE);
    if(serialPlotSignals) SerialShowSignals(Serial, Threshold_ON, Threshold_OFF, movingAvVelocityk[sensor], maxVelocity[sensor],0);  

    
    sendMIDI(SerialMidi,NOTE_ON,CONTROL[sensor],MIDI_VEL_TEST);
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];

  
}


void presentacionPorSerieTask(void *parameter){
  SerialPresentation(Serial, SOFTWARE_VERSION, jsonVersion, HARDWARE_VERSION);
  SerialShowActiveSensors(Serial, sensoresActivos);
  SerialComandosDisponibles(Serial);
  SerialShowConfig(Serial, MIDI_CHANNEL,
                  Threshold_ON, Threshold_OFF,
                  Detection_Time, Gain_Velocity,
                  Duration_Velocity,
                  TEST_MIDI,
                  boardNumber);   
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
  // Pin para ejecutar el midi test
  pinMode(PIN_MIDI_TEST, INPUT_PULLUP);
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
  
  doc["MIDI_CHANNEL"] = MIDI_CHANNEL;
  doc["THRESHOLD_ON"] = Threshold_ON;
  doc["THRESHOLD_OFF"] = Threshold_OFF;
  doc["MAX_VELOCITY"] = Max_Velocity;
  doc["GAIN_VELOCITY"] = Gain_Velocity;
  doc["DETECTION_TIME"] = Detection_Time;
  doc["DURATION_VELOCITY"] = Duration_Velocity;
  TEST_MIDI == true? doc["TEST_MIDI"] = 1 : doc["TEST_MIDI"] = 0;
  //doc["JSON_VERSION"] = jsonVersion;

 
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
        case 'B':   // Serial Debug Signals. Muestra información por el puerto Serie
          serialDebugSignals = !serialDebugSignals;
          serialDebugSignals? Serial.println("Debug Signals ON") : Serial.println("Debug Signals OFF");
          break;
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
        case 'M':  // Midi comming from other boards - Plan B
          if (Serial.available() >= 3) {
            int cmd = Serial.read();
            int data1 = Serial.read();
            int data2 = Serial.read();
            sendMIDI(SerialMidi, cmd, data1, data2);
            //Serial.printf("C %x, D1 %x, D2 %x\n", cmd, data1, data2);
          }
          break;

        case 'N':   // Test midi
          midiTest = Serial.parseInt();
          midiTest = constrain(midiTest,0,1);
          midiTest == 1? TEST_MIDI = true : TEST_MIDI = false;
          break;
        case 'P':   // Plot Signals
          serialPlotSignals = !serialPlotSignals;
          serialPlotSignals? Serial.println("Plot Signals ON") : Serial.println("Plot Signals OFF");
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
          Serial.printf("Comando recibido: %c desconocido\n", comando);
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
    // serializeJsonPretty(doc, Serial); // Muestra todo el contenido del json
    Serial.println(F("Loading config from json File"));
    jsonVersion = doc["JSON_VERSION"].as<int>();
    Serial.printf("Json Version: %d\n", jsonVersion);
    MIDI_CHANNEL = doc["MIDI_CHANNEL"].as<int>();
    Threshold_ON = doc["THRESHOLD_ON"].as<float>();
    Threshold_OFF = doc["THRESHOLD_OFF"].as<float>();
    Max_Velocity =  doc["MAX_VELOCITY"].as<float>();
    Gain_Velocity = doc["GAIN_VELOCITY"].as<float>();
    Duration_Velocity = doc["DURATION_VELOCITY"].as<int>();
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
      CONTROL[i] = doc["CONTROL"][String(boardNumber)][i].as<int>();
      Serial.println(CONTROL[i]);
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
  
  // Selección de Octava y dirección de I2C
  boardNumber = findBoard(PIN_OCT_SEL_0, PIN_OCT_SEL_1, PIN_OCT_SEL_2);
  
  boardNumber == 0? SerialMidi.begin(MIDI_BAUDRATE): SerialMidi.begin(BAUDRATE);
  // Plan B. Como no puedo usar el I2C, voy a comunicar las ESPs por los puertos series.
  // Cada esclava sale desde el puerto serialMidi y entra al Serial de la board previa.
  // El mensaje midi comienza con una letra M. Al recibir la letra M, automaticamente la board retransmite por el puerto Serial 1 
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
       
  }
  
  // La presentación por el puerto serie se realiza en el segundo núcleo.
  crearTareaDePresentacionPorSerie();
}

void loop()
{
 
  static bool noDetectarVelocidadOld = false;
  bool noDetectarVelocidad = digitalRead(PIN_NO_DETECTAR_VELOCIDAD);
  if(noDetectarVelocidad != noDetectarVelocidadOld){
    noDetectarVelocidad == true? Serial.println("No detectar velocidad") : Serial.println("Detectar Velocidad");
    noDetectarVelocidadOld = noDetectarVelocidad;
  }

  if(boardNumber == 0) sustainPedalUpdate();
  
  serialCommands();
  
  
  for (int i = 0; i < sensoresActivos; i++)
  {
    noDetectarVelocidad == true? findNotes3NoVelocity(i):findNotes3(i);
       
  }
  
  if (!digitalRead(PIN_MIDI_TEST)){
    TestMIDI(Serial, SerialMidi, 1);
  }
    
}