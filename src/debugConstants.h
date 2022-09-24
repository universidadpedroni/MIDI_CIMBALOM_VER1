#include <Arduino.h>


// *** CONFIGURATION OPTIONS *** //
#define DEBUG true
const bool  PLOT_SIGNALS =  false;
 int plotSensor = 4;
const bool TEST_MIDI = false;
// **************************** //


#if DEBUG
    #define Debug(x)    Serial.print(x)
    #define Debugln(x)  Serial.println(x)
    #define Debugf(x, args)   Serial.printf(x,args)
    
#else
    #define Debug(x) 
    #define Debugln(x)
    #define Debugf(x)
#endif 