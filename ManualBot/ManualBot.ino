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
#define LMotor_1      2
#define LMotor_2      3
#define LMotor_PWM    5
#define RMotor_1      4
#define RMotor_2      7
#define RMotor_PWM    6


//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;
int fspeed;
bool dir,steer;

void setup()
{
 
  Serial.begin(57600);
  delay(100);  //added delay to give wireless ps2 module some time to startup, before configuring it
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
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
void Motor_Control(bool LM1, bool LM2, int LP, bool RM1, bool RM2, int RP )
{
  digitalWrite(LMotor_1, LM1);
  digitalWrite(LMotor_2, LM2);
  analogWrite(LMotor_PWM, LP);
  digitalWrite(RMotor_1, RM1);
  digitalWrite(RMotor_2, RM2);
  analogWrite(LMotor_PWM, RP);
}
void loop() 
{
  if(error == 1) //skip loop if no controller found
    return; 
  ps2x.read_gamepad();    //read controller 
  Serial.println(ps2x.Analog(PSS_LY));
  steer = 0;
 // if(ps2x.Analog(PSS_RX) >  128)
 //   steer = map(ps2x.Analog(PSS_LY), 0, 127, 255, 0);
  if(ps2x.Analog(PSS_LY) == 127) 
    Motor_Control(0,1,0,0,1,0);
  if(ps2x.Analog(PSS_LY) > 127)
  {
    fspeed = map(ps2x.Analog(PSS_LY), 128, 255, 0, 255);
    Motor_Control(0,1,fspeed,0,1,fspeed);
  }
  if(ps2x.Analog(PSS_LY) < 127)
  {
    fspeed = map(ps2x.Analog(PSS_LY), 0, 126, 255, 0);
    Motor_Control(1,0,fspeed,1,0,fspeed);
  }
  delay(40);
}