
const int MotorPinA[2] = {7,4};
const int MotorPinB[2] = {8,9};
const int MotorPWM[2] = {5,6};

#define MOTORL 0
#define MOTORR 1

#define CW 1
#define CCW 2
#define BREAKVCC 3
#define BREAKGND 4

int speedd = 0;

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};
int newFlashInterval = 0;
float servoFraction = 0.0; // fraction of servo range to move
int pwmR = 0; int dirR = 1;
int pwmL = 0; int dirL = 1;

unsigned long curMillis;

unsigned long prevReplyToPCmillis = 0;
unsigned long replyToPCinterval = 1000;

//=============

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino is ready");
  for (int i=0; i<2; i++)
  {
    pinMode(MotorPinA[i], OUTPUT);
    pinMode(MotorPinB[i], OUTPUT);
    pinMode(MotorPWM[i], OUTPUT);
  }
    pinMode(13, OUTPUT);
}

//=============

void loop() {
 //getDataFromPC();
 Motor_Control(0,1,255);
 Motor_Control(1,1,255);
 // replyToPC();
}

//=============

void getDataFromPC() {

    // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {

    char x = Serial.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) { 
      bytesRecvd = 0; 
      readInProgress = true;
    }
  }
}

//=============
 
void parseData() {

    // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,",");      // get the first part - the string
  dirL = atoi(strtokIndx); // copy it to messageFromPC
  
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  pwmL = atoi(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  dirR = atof(strtokIndx);     // convert this part to a float

  
  strtokIndx = strtok(NULL, ","); 
  pwmR = atof(strtokIndx);  

}

//=============

void replyToPC() 
{

  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<DirL ");
    Serial.print(dirL);
    Serial.print(" PwmL ");
    Serial.print(pwmL);
    Serial.print(" DirR ");
    Serial.print(dirR);
    Serial.print(" PwmR ");
    Serial.print(pwmR);
    Serial.print(" Time ");
    Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
    Serial.println(">");
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
