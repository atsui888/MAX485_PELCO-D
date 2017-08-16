/*
 MAX485 Master
 ============
 RS485  / PELCO-D protocol (9600, 8N1)
 Mast Address given by manufacturer is 0x01
 Note:
 If using Mega2560, only the following pins can be used for software Serial (RX)
  10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69 
 */

#include <SoftwareSerial.h>

/*
The Circuit
===========
  MAX485 D1                 <-> arduino SSerialTx
  MAX485 R0                 <-> arduino SSerialRx
  MAX485 De and Re (jumper) <-> pin 3 Arduino
*/

// Protocol PELCO-D
// need to understand PELCO-D in detail if robot is autonomous
// if teleop, human will be the limiter when they let go of the joystick
uint8_t ptzUp[]     = {0xFF, 0x01, 0x00, 0x08, 0x00, 0xFF, 0x08};
uint8_t ptzDown[]   = {0xFF, 0x01, 0x00, 0x10, 0x00, 0xFF, 0x10};
uint8_t ptzLeft[]   = {0xFF, 0x01, 0x00, 0x04, 0xFF, 0x00, 0x04};
uint8_t ptzRight[]  = {0xFF, 0x01, 0x00, 0x02, 0xFF, 0x00, 0x02};
uint8_t mastUp[]     = {0xFF, 0x01, 0x01, 0x46, 0x00, 0x01, 0x49};  // Camera Mast goes from 'rest' position to Full Height
uint8_t mastDown[]   = {0xFF, 0x01, 0x01, 0x46, 0x00, 0x02, 0x4A};  // PTZ goes to default position and Camera Mast (Full Height) closes to 'rest' position.
uint8_t testBuf[]    = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36};

#define SSerialTx 11
#define SSerialRx 10
#define SSerialTxControl 3   //RS485 Direction control, HIGH->enable transmit, LOW->disable transmit
#define RS485Transmit    HIGH
#define RS485Receive     LOW

SoftwareSerial sSerialMax485(SSerialRx, SSerialTx); // RX, TX

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  pinMode(SSerialTxControl, OUTPUT);    
  Serial.println("Serial Monitor ready...");
  // set the data rate for the SoftwareSerial port
  sSerialMax485.begin(9600);
  delay(1000);    // delay 1s to allow other device to init (can be changed after testing)  
}

void loop() {
  // Enable RS485 Transmit   
  digitalWrite(SSerialTxControl, RS485Transmit);  
  
  Serial.println();
  Serial.println("Sending Pelco-D command...");
  sSerialMax485.write(mastDown,sizeof(mastUp));
  delay(5000);
  /*
  Serial.println();
  Serial.println("Sending Pelco-D command to lower Mast");
  sSerialMax485.write(poleDown,sizeof(mastDown));
  delay(5000);
  */

  // Disable RS485 Transmit       
  digitalWrite(SSerialTxControl, RS485Receive);  
  delay(1000);
}
