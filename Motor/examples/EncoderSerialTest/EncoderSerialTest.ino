#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3);

void setup()  
{
  Serial.begin(57600);
  Serial.println("Goodnight moon!");
  
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);

  // set the data rate for the SoftwareSerial port
  mySerial.begin(4800);
  mySerial.listen();
  mySerial.flush();
  mySerial.println("Hello, world?");
}

void loop() // run over and over
{
  if (mySerial.available())
    Serial.write(mySerial.read());
  /* if (Serial.available())
    mySerial.write(Serial.read()); */
}
