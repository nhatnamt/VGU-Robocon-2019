#include <SoftwareSerial.h>
SoftwareSerial EEBlue(10, 11); // RX | TX
void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  EEBlue.begin(115200);  //Default Baud for comm, it may be different for your Module. 
  Serial.println("The bluetooth gates are open.\n Connect to HC-05 from any other bluetooth device with 4001 as pairing key!.");
 
}
int state = 0;
void loop()
{
 
  // Feed any data from bluetooth to Terminal.
  if (EEBlue.available())
  {
    state = EEBlue.read(); // Reads the data from the serial port
    Serial.write(EEBlue.read());
  }
  // Feed all data from termial to bluetooth
  if (state == '0') 
  {
    digitalWrite(13, LOW); // Turn LED OFF
    Serial.println("LED: OFF"); // Send back, to the phone, the String "LED: ON"
    state = 0;
  }
  else if (state == '1') {
  digitalWrite(13, HIGH);
  Serial.println("LED: ON");;
  state = 0;
 } 
  if (Serial.available())
    EEBlue.write(Serial.read());
}
