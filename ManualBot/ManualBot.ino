#include <PS2X_lib.h>  //for v1.6

 // create PS2 Controller Class
PS2X ps2x;
/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        13  //14    
#define PS2_CMD        11  //15
#define PS2_SEL        10  //16
#define PS2_CLK        12  //17

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   false
#define rumble      false

// Motor
const int MotorPinA[2] = {7,4};
const int MotorPinB[2] = {8,9};
const int MotorPWM[2] = {5,6};

#define MOTORL 0
#define MOTORR 1

#define CW 1
#define CCW 2
#define BREAKVCC 3
#define BREAKGND 4

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;
int limL = 128; int limR = 128;
int fspeedl,fspeedr,sspeed;
bool dir,steer;

void setup()
{
 
  Serial.begin(57600);
  delay(500);  //added delay to give wireless ps2 module some time to startup, before configuring it
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  

  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(MotorPinA[i], OUTPUT);
    pinMode(MotorPinB[i], OUTPUT);
    pinMode(MotorPWM[i], OUTPUT);
  }
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
	if (pressures)
	  Serial.println("true ");
	else
	  Serial.println("false");
	Serial.print("rumble = ");
	if (rumble)
	  Serial.println("true)");
	else
	  Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
//  Serial.print(ps2x.Analog(1), HEX);
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
	case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }
}
void Motor_Control(int motor, int direct, int pwm )
{
  if (direct == CW)
  {
    digitalWrite(MotorPinA[motor],LOW);
    digitalWrite(MotorPinB[motor],HIGH);
  }
  else if (direct == CCW)
  {
    digitalWrite(MotorPinA[motor],HIGH);
    digitalWrite(MotorPinB[motor],LOW);
  }
  else if (direct == BREAKGND)
  {
    digitalWrite(MotorPinA[motor],LOW);
    digitalWrite(MotorPinB[motor],LOW);
  }
  else 
  {
    digitalWrite(MotorPinA[motor],HIGH);
    digitalWrite(MotorPinB[motor],HIGH);
  }
  analogWrite(MotorPWM[motor],pwm);
}
void loop() 
{
  //Motor_Control(0,CW,255);
  if(error == 1)
    return; 

  ps2x.read_gamepad(false, vibrate);  
  Serial.println(ps2x.Analog(PSS_LY));
  
  if(ps2x.Button(PSB_L1))
    limL = 255;
  else limL = 128;

  if (ps2x.Button(PSB_R1))
    limR = 255;
  else limR = 128;
    
  if(ps2x.Button(PSB_L2))
  {
    Serial.println('d');
    Motor_Control(MOTORL,CCW,limL-50);
    Motor_Control(MOTORR,CW,limR -50);
  }
  else if (ps2x.Button(PSB_R2))
  {
    Motor_Control(MOTORL,CW,limL-50);
    Motor_Control(MOTORR,CCW,limR -50);
  }
  else
  {
    if(ps2x.Analog(PSS_LY) == 127)
    {
      if (ps2x.Analog(PSS_RX) == 128)
      {
        Motor_Control(MOTORL,BREAKVCC,0);
        Motor_Control(MOTORR,BREAKVCC,0);
      }
      else if (ps2x.Analog(PSS_RX) > 128)
      {
      //  Serial.println('a');
        sspeed = map(ps2x.Analog(PSS_RX),0,127,limR,0);
        Motor_Control(MOTORL,CW,0);
        Motor_Control(MOTORR,CW,sspeed);
      }
      else
      {
        sspeed = map(ps2x.Analog(PSS_RX),129,255,0,limL);
        Motor_Control(MOTORL,CW,sspeed);
        Motor_Control(MOTORR,CW,0);
      } 
    } 

    if(ps2x.Analog(PSS_LY) < 127)
    {
      fspeedl = map(ps2x.Analog(PSS_LY), 128, 255, 0, limL);
      fspeedr = map(ps2x.Analog(PSS_LY), 128, 255, 0, limR);
      if (ps2x.Analog(PSS_RX) == 128)
      {
        Motor_Control(MOTORL,CW,fspeedl);
        Motor_Control(MOTORR,CW,fspeedr);
      }
      else if (ps2x.Analog(PSS_RX) > 128)
      {
        sspeed = map(ps2x.Analog(PSS_RX),0,127,limR,0);
        Motor_Control(MOTORL,CW,fspeedl-sspeed);
        Motor_Control(MOTORR,CW,fspeedr);
      }
      else
      {
        sspeed = map(ps2x.Analog(PSS_RX),129,255,0,limL);
        Motor_Control(MOTORL,CW,fspeedl);
        Motor_Control(MOTORR,CW,fspeedr - sspeed);
      }  
    }

    if(ps2x.Analog(PSS_LY) > 127)
    {
      fspeedl = map(ps2x.Analog(PSS_LY), 0, 126, limL, 0);
      fspeedr = map(ps2x.Analog(PSS_LY), 0, 126, limR, 0);
      if (ps2x.Analog(PSS_RX) == 128)
      {
        Motor_Control(MOTORL,CCW,fspeedl);
        Motor_Control(MOTORR,CCW,fspeedr);
      }
      else if (ps2x.Analog(PSS_RX) > 128)
      {
        sspeed = map(ps2x.Analog(PSS_RX),0,127,limR/2,0);
        Motor_Control(MOTORL,CCW,fspeedl-sspeed);
        Motor_Control(MOTORR,CCW,fspeedr);
      }
      else
      {
        sspeed = map(ps2x.Analog(PSS_RX),129,255,0,limL/2);
        Motor_Control(MOTORL,CCW,fspeedl);
        Motor_Control(MOTORR,CCW,fspeedr-sspeed);
      }  
    }
  }
  delay(50);
}