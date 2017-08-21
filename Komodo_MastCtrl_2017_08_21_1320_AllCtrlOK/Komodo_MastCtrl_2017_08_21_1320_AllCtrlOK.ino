//#define DEBUG_ON

//#define DEBUG_RC_RECEIVER
//#define DEBUG_PELCO_D
//#define DEBUG_CONTROL_INTEGRATION

/*
 MAX485 Master
 ============ 
 RS485  / PELCO-D protocol (9600, 8N1)
 Mast Address given by manufacturer is 0x01
 Note:
 If using Mega2560, only the following pins can be used for software Serial (RX)
  10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69 
 */

/*
  AT9: Model-001 Aeroplane gfx 
*/

#include <SoftwareSerial.h>

/*
The Circuit
===========
  MAX485 D1                 <-> arduino SSerialTx
  MAX485 R0                 <-> arduino SSerialRx
  MAX485 De and Re (jumper) <-> pin 3 Arduino
*/

// Protocol PELCO-D - chksum is sum of bytes 2 to 6
uint8_t mastUp[]        = {0xFF, 0x01, 0x01, 0x46, 0x00, 0x01, 0x49};  
// PTZ goes to default position and Camera Mast (Full Height) closes to 'rest' position.
uint8_t mastDown[]      = {0xFF, 0x01, 0x01, 0x46, 0x00, 0x02, 0x4A};  

uint8_t ptzUp[]         = {0xFF, 0x01, 0x00, 0x08, 0x00, 0xFF, 0x08};
uint8_t ptzDown[]       = {0xFF, 0x01, 0x00, 0x10, 0x00, 0xFF, 0x10};
uint8_t ptzLeft[]       = {0xFF, 0x01, 0x00, 0x04, 0xFF, 0x00, 0x04};
uint8_t ptzLeft_Fast[]  = {0xFF, 0x01, 0x00, 0x04, 0x3F, 0x00, 0x44};
uint8_t ptzRight[]      = {0xFF, 0x01, 0x00, 0x02, 0xFF, 0x00, 0x02};
// STOP Pan / Tilt / Zoom / Iris etc. 
uint8_t ptzStop[]       = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};

/*
  Pan Right at medium speed: FF 01 00 02 20 00 23
  Tilt Up at high speed: A0 00 00 08 00 20 AF 27
  Tilt Down at medium speed: FF 01 00 10 20 00 31
*/

uint8_t testBuf[]    = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36};

#define SSerialTx 11
#define SSerialRx 10
#define SSerialTxControl 3   //RS485 Direction control, HIGH->enable transmit, LOW->disable transmit
#define RS485Transmit    HIGH
#define RS485Receive     LOW

SoftwareSerial sSerialMax485(SSerialRx, SSerialTx); // RX, TX


// RC Channel Pins
// Used by Komodo 
// **************
// Receiver Ch2 i.e. RJS_UD --> used by Komodo Forwards/Backwards
const byte ArdPin_ReceiverCh2 = 3;
// Receiver Ch4 i.e. RJS_LR --> used by Komodo A_Left/A_Right
const byte ArdPin_ReceiverCh4 = 5;

/*
  Cam pole PTZ controls
  =====================
  ch3 (2): LJS_UD -> PTZ UD (enable/disable cam ctrl)
  ch7 (4): RearL_switch -> toggle MastUp/PoleUp
  ch5 (5): RearR_Switch -> All down to REST pos
  ch6 (6): Left Side Turning knob (PTZ L/R)
  ch8 (7): Right Side Turn knob (PTZ U/D)
*/


// Enable / Disable Mast_Pole
// Receiver Ch 3: LJS_UD --> used for PTZ Up and Down
// Failsafe: Dn Pos
// if stick up, all Camera controls work
// If stick down, all camera controls do NOT work
// if stick down, Camera does not fold back to original position as customer may want to leave it as a deployed camera.
const byte ArdPin_ReceiverCh3 = 2;

// Used by Mast and Pole
// *********************
// Receiver Ch 7: Rear_Left_Switch --> Toggle Mast up, toggle Pole up.
const byte ArdPin_ReceiverCh7 = 4;

// Receiver Ch 5: Rear_Right_Switch --> Toggle both Mast and Pole down to default REST pos
const byte ArdPin_ReceiverCh5 = 5;

// Used by PTZ
// ************
// Receiver 6: Left Side turning Knob
// PTZ Left n Right
const byte ArdPin_ReceiverCh6 = 6;

// Receiver 8: Right Side turning Knob
// PTZ Up and Down
const byte ArdPin_ReceiverCh8 = 7;

// Init RC Channel Values 
int receiverCh3=0;  // LJS_UD
int receiverCh7=0;  // RL switch
int receiverCh5=0;  // RR switch
int receiverCh6=0;  // L Turning Knob
int receiverCh8=0;  // R Turning Knob

byte camControls_Status = 0;          // 0 is inactive
int receiver_Limit_Low = 1200;        // <1200 is low
int receiver_Limit_High = 1800;       // >1800 is high
// between 1450 and 1550 is neutral pos
int receiver_Limit_Neutral_Low = 1450;
int receiver_Limit_Neutral_High = 1550;

// for Rear Left and Right Switches, center pos is !used, only UD is used
// Rear Left Switch
byte receiverCh7_CurrentState = 0;  // 0: down, 1: neutral, 2: up 
byte receiverCh7_TempState = 0;
// Rear Right Switch
byte receiverCh5_CurrentState = 0;  // 0: down, 1: neutral, 2: up
byte receiverCh5_TempState = 0;

byte receiverCh6_CurrentState = 2; // 1: Left, 2: Center, 3: Right
byte receiverCh6_TempState = 2;
byte receiverCh8_CurrentState = 2; // 1: Left, 2: Center, 3: Right
byte receiverCh8_TempState = 2;

void setup() {
  // For RC Receiver input
  pinMode(ArdPin_ReceiverCh3, INPUT); 
  pinMode(ArdPin_ReceiverCh7, INPUT); 
  pinMode(ArdPin_ReceiverCh5, INPUT); 
  pinMode(ArdPin_ReceiverCh6, INPUT);
  pinMode(ArdPin_ReceiverCh8, INPUT);

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // for Pelco-D
  pinMode(SSerialTxControl, OUTPUT); 
  // set the data rate for the SoftwareSerial port
  sSerialMax485.begin(9600);
  delay(1000);    // delay 1s to allow other device to init (can be changed after testing)
  receiverCh7 = pulseIn(ArdPin_ReceiverCh7,HIGH,25000); 
  receiverCh5 = pulseIn(ArdPin_ReceiverCh5,HIGH,25000); 

  // get initial state, if states changes later via user toggle, action takes place
  getReceiverCh7State();
  receiverCh7_CurrentState=receiverCh7_TempState; 
  getReceiverCh5State();
  receiverCh5_CurrentState=receiverCh5_TempState;  
}

void loop() {
  receiverCh3 = pulseIn(ArdPin_ReceiverCh3,HIGH,25000); // CamCtrl Status: LJS - UpDown
  receiverCh7 = pulseIn(ArdPin_ReceiverCh7,HIGH,25000); // MastUp: R_Left_Switch
  receiverCh5 = pulseIn(ArdPin_ReceiverCh5,HIGH,25000); // MastDn: R_Right_Switch
  receiverCh6 = pulseIn(ArdPin_ReceiverCh6,HIGH,25000); // PTZ_LR: Left Turning Knob
  receiverCh8 = pulseIn(ArdPin_ReceiverCh8,HIGH,25000); // PTZ_UD: Right Turning Knob
  
  receiverCh3>receiver_Limit_High?camControls_Status=0:camControls_Status=1;  

  #ifdef DEBUG_RC_RECEIVER
    debug_rc_receiver_fn();  
  #endif

  #ifdef DEBUG_PELCO_D
     //sSerialMax485.write(mastUp,sizeof(mastUp));
    sSerialMax485.write(mastDown,sizeof(mastDown));
    for(;;){}
  #endif

  if(!camControls_Status) {
    #ifdef DEBUG_CONTROL_INTEGRATION
      Serial.println();
      Serial.println("Cam control is disabled");
    #endif        
  } else {
    #ifdef DEBUG_CONTROL_INTEGRATION
      //Serial.println();
      //Serial.println("Cam control Enabled");
    #endif
   
    getReceiverCh7State();    
    if(receiverCh7_CurrentState!=receiverCh7_TempState) {
      // send command to raise mast_pole
      #ifdef DEBUG_CONTROL_INTEGRATION
        Serial.println();
        Serial.println("Raise the mast/pole");
      #endif
      digitalWrite(SSerialTxControl, RS485Transmit);
      sSerialMax485.write(mastUp,sizeof(mastUp));            
      receiverCh7_CurrentState=receiverCh7_TempState;
      digitalWrite(SSerialTxControl, RS485Receive);      
    }
    
    getReceiverCh5State();
    if(receiverCh5_CurrentState!=receiverCh5_TempState) {
      // send command to lower mast/pole
      #ifdef DEBUG_CONTROL_INTEGRATION
        Serial.println();
        Serial.println("Lower mast/pole to Resting Pos");
      #endif
      digitalWrite(SSerialTxControl, RS485Transmit);
      sSerialMax485.write(mastDown,sizeof(mastDown));      
      receiverCh5_CurrentState=receiverCh5_TempState;
      digitalWrite(SSerialTxControl, RS485Receive);     
    }

    // RC notes:
    /*
        if Right knob is in center pos and if left knob is in leftmost pos,
        PTZ will !move at all or it will stutter, cause left knob cmds a pan 
        but Right knob (which is executed just after the left knob) commands a stop.

        seems like the best way to control the PTZ UD/LR is via LJS because stop is via Center
        spring back pos which is where but UD and LR agree.
   */
    
    // Ch6 Left side Turning Knob - Get state
    if(receiverCh6 > 1600) {
      // knob is to the right
      receiverCh6_TempState = 3;           
    } else if(receiverCh6 < 1400) {
      // knob is to the left
      receiverCh6_TempState = 1;      
    } else {
      receiverCh6_TempState = 2;      
    }
    #ifdef DEBUG_CONTROL_INTEGRATION
      
      Serial.println("");
      Serial.print("receiverCh6_TempState is: ");
      Serial.print(receiverCh6_TempState);
      Serial.print("  receiverCh6_CurrentState is: ");
      Serial.print(receiverCh6_CurrentState);
      Serial.print("  receiverCh8_CurrentState is: ");
      Serial.print(receiverCh8_CurrentState);
      
    #endif

      
    // If ch6 state changed AND if ch8 is at Centre Pos
    // between PTZ_LR and PTA_UD, only one can be active
    if((receiverCh6_CurrentState!=receiverCh6_TempState)&&(receiverCh8_CurrentState==2)) {
      //digitalWrite(SSerialTxControl, RS485Transmit);
      digitalWrite(SSerialTxControl, RS485Transmit);
      switch(receiverCh6_TempState) {
        case 1:
                sSerialMax485.write(ptzStop,sizeof(ptzStop));
                sSerialMax485.write(ptzLeft,sizeof(ptzLeft));
                receiverCh6_CurrentState=receiverCh6_TempState;
                break;
        case 2:
                sSerialMax485.write(ptzStop,sizeof(ptzStop));
                receiverCh6_CurrentState=receiverCh6_TempState;
                break;
        case 3: 
                sSerialMax485.write(ptzStop,sizeof(ptzStop));
                sSerialMax485.write(ptzRight,sizeof(ptzRight));                
                receiverCh6_CurrentState=receiverCh6_TempState;
                break;        
        default:
                break;
      }
      digitalWrite(SSerialTxControl, RS485Receive);
      //digitalWrite(SSerialTxControl, RS485Receive);  
    }    

    
    // Ch8 Right side Turning Knob - Get state
    if(receiverCh8 > 1600) {
      // knob is to the right
      receiverCh8_TempState = 3;           
    } else if(receiverCh8 < 1400) {
      // knob is to the left
      receiverCh8_TempState = 1;      
    } else {
      receiverCh8_TempState = 2;      
    }
    // If ch8 state changed AND if ch6 is at Centre Pos
    // PTZ LR and UD, only one can be active
    if((receiverCh8_CurrentState!=receiverCh8_TempState)&&(receiverCh6_CurrentState==2)) {
      digitalWrite(SSerialTxControl, RS485Transmit);
      switch(receiverCh8_TempState) {
        case 1:
                sSerialMax485.write(ptzStop,sizeof(ptzStop));
                sSerialMax485.write(ptzDown,sizeof(ptzDown));
                receiverCh8_CurrentState=receiverCh8_TempState;
                break;
        case 2:
                sSerialMax485.write(ptzStop,sizeof(ptzStop));
                receiverCh8_CurrentState=receiverCh8_TempState;
                break;
        case 3: 
                sSerialMax485.write(ptzStop,sizeof(ptzStop));
                sSerialMax485.write(ptzUp,sizeof(ptzUp));
                receiverCh8_CurrentState=receiverCh8_TempState;
                break;
        default:
                break;        
      }
      digitalWrite(SSerialTxControl, RS485Receive);
    }  
    
              
  }  

  /*
  PelcoD_Cmd(mastUp); // something wrong with this fn
  for(;;) {}
  */ 

  
  // *****************************************************************
  //          All PELCO-D cmds executed
  // *****************************************************************
  // Disable RS485 TX mode and delay to avoid buffer overflow
  //digitalWrite(SSerialTxControl, RS485Receive);  
  
  delay(50);
}

void getReceiverCh7State() {
  if(receiverCh7 > receiver_Limit_High) {
    // switch is in Down state
    receiverCh7_TempState = 0;
  } else if(receiverCh7 < receiver_Limit_Low) {
    // switch is in Up state
    receiverCh7_TempState = 2;  
  }  
}

void getReceiverCh5State() {  
  if(receiverCh5 > receiver_Limit_High) {
    // switch is in Down state
    receiverCh5_TempState = 0;
  } else if(receiverCh7 < receiver_Limit_Low) {
    // switch is in Up state
    receiverCh5_TempState = 2;  
  } 
}


/*
void PelcoD_Cmd(uint8_t *cmd) {
  // Enable RS485 Transmit   
  digitalWrite(SSerialTxControl, RS485Transmit);    
  
  //sSerialMax485.write(mastDown,sizeof(mastUp)); 
  sSerialMax485.write(cmd,sizeof(cmd)); 
  
  // Disable RS485 Transmit       
  digitalWrite(SSerialTxControl, RS485Receive);  
  delay(1000);
}
*/

/*
 
  Radio Link AT9 Transmitter & R9D Receiver Pulse Width
  
  Channels in use (original configuration):
  
  Channel 1: Only VCC & GND Pins are connected. 
  
  Channel 2 (Right joystick Up & down): 
  Only DATA PIN is connected. Forward & Reverse control.
  [1491 ~ 1497µs neutral]
  [1371µs full forward]
  [1614 ~ 1620µs full reverse]
  
  Channel 4 (Right joystick Left & Right): 
  Only DATA PIN is connected. Axial turn control 
  [1504 ~ 1511µs neutral] 
  [1370 ~ 1376µs axial turn left] 
  [1614 ~ 1621µs axial turn right] 
  
  Extra channels not in use:
  
  Channel 1 (Left joystick left & right):
  [1081µs full left position]
  [1486 ~ 1493µs neutral position]
  [1899 ~ 1906µs full right position]
  
  Channel 3 (Left joystick Up & Down):
  [1904 ~ 1911µs full down position]
  [1486 ~ 1492µs center position]
  [1081µs full up position]
  
  Channel 5 (Rear right side sliding switch):
  [1906 ~ 1913µs full down position]
  [1544 ~ 1551µs center position]
  [1073 ~ 1080µs full up position]
  
  Channel 6 (Left side turning knob):
  [1365 ~ 1371µs full anti-clockwise position]
  [1499 ~ 1506µs center position]
  [1614 ~ 1621µs full clockwise position]
  
  Channel 7 (Rear left side sliding switch):
  [1908 ~ 1915µs full down position]
  [1543 ~ 1549µs center position]
  [1081 ~ 1082µs full up position]
  
  Channel 8 (Right side turning knob):
  [1076 ~ 1082µs full anti-clockwise position]
  [1510 ~ 1516µs center position]
  [1908 ~ 1915µs full clockwise position]
  
  Channel 9 (Left side flap switch B):
  [1909 ~ 1915 down position]
  [1076 ~ 1083 up position]
 
 */

/*
   Testing required to confirm what these cmds do
    MASK POLE UP
    FF 01 01 46 00 01 49   < -- > send code 1st time pole moving from default to stand position
    FF 01 01 46 00 01 49   < -- > send code 2nd time pole retract up high
    
    MASK POLE DOWN DEFAULT
    FF 01 01 46 00 02 4A  < -- > move the mask to default stage 
    
    ROTATE LEFT  
    FF 01 00 02 FF 00 02 < -- > camera move continuously until reaches max to stop 
    
    ROTATE LEFT STOP 
    FF 01 00 00 00 00 01
    
    ROTATE LEFT in STEP
    FF 01 00 02 FF 00 02 FF 01 00 00 00 00 01 < -- > send code camera move in small steps 
    
    ROTATE RIGHT
    FF 01 00 04 FF 00 04 < -- > camera move continuously until reaches max to stop 
              NOTE: move to left first
    
    ROTATE RIGHT STOP
    FF 01 00 00 00 00 01
    
    ROTATE RIGHT in STEP
    FF 01 00 04 FF 00 04 FF 01 00 00 00 00 01 < -- > send code camera move in small steps 
    
    CAMERA UP
    FF 01 00 08 00 FF 08 < -- > camera move continuously until reaches max to stop
    
    CAMERA UP STOP
    FF 01 00 00 00 00 01 
    
    CAMERA UP STEP
    FF 01 00 08 00 FF 08 FF 01 00 00 00 00 01 < -- > send code camera move in small steps 
    
    CAMERA DOWN
    FF 01 00 10 00 FF 10 < -- > camera move continuously until reaches max to stop
    
    CAMERA DOWN STOP
    FF 01 00 00 00 00 01 < -- > camera stop
    
    CAMERA DOWN STEP
    FF 01 00 10 00 FF 10 FF 01 00 00 00 00 01 < -- > send code camera move in small steps 

*/

/*
  https://www.commfront.com/pages/pelco-d-protocol-tutorial

  http://info.kmtronic.com/pelco-d-commands.html
  
*/

void debug_rc_receiver_fn() {
  Serial.println();

    // LJS_LR... (L)1075~1080 -- 1488~1495 -- (R)1893~1895
    // Failsafe: Neutral position
    //Serial.print("Rx_Ch1 PTZ LR = "); 
    //Serial.println(receiverCh1);

    // LJS_UD... Up(1074~1081) -- 1513~1520 -- Dn(1903~1909)
    // Failsafe: Dn pos
    //Serial.print("Rx_Ch3 PTZ UD = "); 
    //Serial.println(receiverCh3);

    // Rear_Left_Switch: Up(1050~1081) -- Dn(1907~1914)
    // Failsafe: Last known position
    //Serial.print("Rx_Ch7 Mast/Pole UD = "); 
    //Serial.println(receiverCh7);

    // Rear_Right_Switch: Up(1078~1079) -- Dn(1905~1906)
    // Failsafe: Last Know position
    //Serial.print("Rx_Ch5 Mast/Pole UD = "); 
    //Serial.println(receiverCh5);

    // Left Side: Switch B (up/down: Up(1076~1082) -- Dn(1908~1915)
    // Failsafe: Last known position
    //Serial.print("Rx_Ch9 mast/pole/ptz enable/disable = "); 
    //Serial.println(receiverCh9);

    // Left Side: Turning Knob 
    // Failsafe:
    Serial.print("Rx_Ch6 mast/pole/ptz enable/disable = "); 
    Serial.println(receiverCh6);
}

