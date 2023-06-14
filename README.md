====================================================================
DIPSWITCH CONFIG
DipSwitch 4 contacts.
Switch 1: TBD. Do not use. Wired to LED_BUILTIN. INPUT_PULLUP doesn't work.
Switch 2: PIN OPTIONS 1
Switch 3: TBD. Do not use. It is wired to RX2
Switch 4: PIN OPTIONS 0

/* Tabla de verdad de los pines
PIN OPTIONS 1   PIN OPTIONS 0
0               0                   Detectar Velocidad
0               1                   No detectar Velocidad
1               0                   Test Midi
1               1                   OTA
*/

DipSwitch 3 contacts.
Used to set the Octave. See the board to configure. Midi signals will be available in board 0 only.

====================================================================
List of available commands
B - Debug Signals on Serial  Port
C,n - Midi Channel [1-16]
D,n - Detection Time [0, 1000]
F,n - Threshold OFF [0, 0.8 * Threshold on]
G,n - Velocity Gain [1 10]
N,n - Test Midi [0,1]
P - Plot Signals ON / OFF
T,n - Threshold ON [0, 200]
S, n - enabled Sensors [1 - 5]
====================================================================
OTA Update.
1. Set Switch 4 to .
2. Power up the board. It will  be configured as an access point.
3. Search for MidiESP OTA
4. Connect to the board using WiFi.
5. go to 192.168.5.1/update