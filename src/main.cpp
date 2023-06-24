#include <Arduino.h>

#include <config.h>
#include <findNotes3Options.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "debugConstants.h"
#include <auxFunc.h>
#include <dipSwitchFunc.h>

// OTA con Access Point
#include <wifiFunctions.h>




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





// FUNCIONANDO, SIN VELOCIDAD
void findNotes3(int sensor)
{
  
  unsigned long currentTime = millis();
  
  // Reading the amplitude of the signal and going through moving average filter
  amplik[sensor][0] =  (float)analogRead(CANALES_ADC[sensor]) / Attenuation;
  movingAvVelocityk[sensor] = (amplik[sensor][0] + amplik[sensor][1] + amplik[sensor][2]) / 3;
  
  
  // *** State Machine ***
  // 1. NOTE OFF -> NOTE OFF ( No necesito pasar por este estado)
  //if((noteState[sensor] == NOTE_OFF) && 
  //   (movingAvVelocityk[sensor] < Threshold_ON))
  //   {
  //      //Serial.printf("NOTE OFF - NOTE OFF, sensor %d\n", sensor);
  //      // Nothing to do here
  //   }
  // 2. NOTE OFF - NOTE ON   
  if ((noteState[sensor] == NOTE_OFF) && 
           (movingAvVelocityk[sensor] > Threshold_ON))
  {
    noteState[sensor] = NOTE_ON;
    detectionTime[sensor] = currentTime;
    maxVelocity[sensor] = movingAvVelocityk[sensor];
    notaEnviada[sensor] = false;
    contadorMax[sensor] = 0;
    if(serialPlotSignals) SerialShowSignalsLabels(Serial);
    //Serial.printf("NOTE OFF - NOTE ON, sensor %d\n", sensor);
    
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
    //Serial.printf("NOTE ON - NOTE OFF, sensor %d\n", sensor);   
  }
  
  // Updates
  amplik[sensor][2] = amplik[sensor][1];
  amplik[sensor][1] = amplik[sensor][0];
}

void findNotes3NoVelocity(int sensor)
{
  
  unsigned long currentTime = millis();
  
  // Reading the amplitude of the signal and going through moving average filter
  amplik[sensor][0] =  (float)analogRead(CANALES_ADC[sensor]) / Attenuation;
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


void presentacionPorSerie(){
  SerialPresentation(Serial, SOFTWARE_VERSION, jsonVersion, HARDWARE_VERSION);
  SerialComandosDisponibles(Serial);
  SerialShowConfig(Serial, MIDI_CHANNEL,
                  Threshold_ON, Threshold_OFF,
                  Detection_Time, Attenuation,
                  Duration_Velocity,
                  boardNumber);   
  //Serial.printf("Tarea ejecutándose en el núcleo %d\n", xPortGetCoreID());
  SerialShowConfigSensores(Serial, NUM_SENSORES, CONTROL);
  SerialSetupFinished(Serial);
  
}


void GPIOSetup()
{
  // Sustain Pedal
  pinMode(PIN_SUSTAIN_PEDAL,INPUT_PULLUP);
  // OTA PIN
  //pinMode(PIN_OTA, INPUT_PULLUP);
  // Pin para determinar qué función se usa para detectar las notas.
  pinMode(PIN_OPTIONS_0, INPUT_PULLUP);
  // Pin para ejecutar el midi test
  pinMode(PIN_OPTIONS_1, INPUT_PULLUP);
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
  doc["ATTENUATION"] = Attenuation;
  doc["DETECTION_TIME"] = Detection_Time;
  doc["DURATION_VELOCITY"] = Duration_Velocity;
  

 
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
        case 'G': // Attenuation
          Attenuation = Serial.parseFloat();
          Attenuation = constrain(Attenuation, 1, 1000);
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

        //case 'N':   // Test midi
        //  midiTest = Serial.parseInt();
        //  midiTest = constrain(midiTest,0,1);
        //  midiTest == 1? TEST_MIDI = true : TEST_MIDI = false;
        //  break;
        case 'P':   // Plot Signals
          serialPlotSignals = !serialPlotSignals;
          serialPlotSignals? Serial.println("Plot Signals ON") : Serial.println("Plot Signals OFF");
          break;
        case 'T':   // ThresHold ON
          Threshold_ON = Serial.parseFloat();
          Threshold_ON = constrain(Threshold_ON, 0, 200);
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
    MIDI_CHANNEL = doc["MIDI_CHANNEL"].as<int>();
    Threshold_ON = doc["THRESHOLD_ON"].as<float>();
    Threshold_OFF = doc["THRESHOLD_OFF"].as<float>();
    Attenuation =  doc["ATTENUATION"].as<float>();
    Duration_Velocity = doc["DURATION_VELOCITY"].as<int>();
    Detection_Time = doc["DETECTION_TIME"].as<unsigned long>();
    
    
    if(MIDI_CHANNEL < 1 || MIDI_CHANNEL > 16 ) MIDI_CHANNEL = MIDI_CHANNEL_DEFAULT;
    if(Threshold_ON < 0.0 || Threshold_ON > 200.0) Threshold_ON = THRESHOLD_ON_DEFAULT;
    if(Threshold_OFF < 0.0 || Threshold_OFF > Threshold_ON) Threshold_OFF = 0.5 * Threshold_ON;
    if(Detection_Time < 0) Detection_Time = DETECTION_TIME_DEFAULT;
    if(Duration_Velocity < 1 || Duration_Velocity > 100) Duration_Velocity = DURATION_VELOCITY_DEFAULT;
    
    
    
    for (int i = 0; i < NUM_SENSORES; i++)
    {
      CONTROL[i] = doc["CONTROL"][String(boardNumber)][i].as<int>();
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
  int opciones = determinarOpcionesDeDip4(PIN_OPTIONS_0, PIN_OPTIONS_1);
  
  // OTA
  if (opciones == OPTIONS_OTA)
  {
    Serial.println(F("Starting OTA Update"));
    OverTheAirUpdate();
  }
  // Load Config from SPIFF
  bool ConfigCargadaDesdeSPIFF = CargarConfiguracionDesdeSPIFF();
  ConfigCargadaDesdeSPIFF == true? Serial.println(F("Config loaded from json file")):Serial.println(F("Loading Default Config"));
    
  // La presentación por el puerto serie se realiza en el segundo núcleo.
  presentacionPorSerie();
}

void loop()
{
 
  static bool DetectarVelocidadOld = true;
  bool DetectarVelocidad = false;
  int opciones = determinarOpcionesDeDip4(PIN_OPTIONS_0, PIN_OPTIONS_1);
  if (opciones == OPTIONS_DETECTAR_VELOCIDAD) DetectarVelocidad = true;
  if( opciones == OPTIONS_NO_DETECTAR_VELOCIDAD) DetectarVelocidad = false;
  if(DetectarVelocidad != DetectarVelocidadOld){
    DetectarVelocidad == true? Serial.println("Detectar velocidad") : Serial.println("No Detectar Velocidad");
    DetectarVelocidadOld = DetectarVelocidad;
  }
  


  //if(boardNumber == 0) sustainPedalUpdate();
  
  serialCommands();
  
  
  for (int i = 0; i < NUM_SENSORES; i++)
  {
    DetectarVelocidad == true? findNotes3(i): findNotes3NoVelocity(i);
       
  }
  
  if(opciones == OPTIONS_TEST_MIDI) TestMIDI(Serial, SerialMidi, 500);
      
}