/* Kay Hutchinson 10/18/19
 * Test code to communicate with Arduino using Python and parse commands.
 * Written while drinking a pumpkin spice latte, please excuse typos.
*/


//int incomingByte = 0;
//String myString;
char receivedChar;
byte receivedByte;
boolean newData = false;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
}

void loop() {

  Serial.println("data");
  
  recvOneChar();

  if (newData == true) {
    react(receivedChar);
  }
  
  showNewData();
  delay(500);
  
  
  
  /*
  // send data only when you receive data:
  // check if data has been sent from the computer:
  if (Serial.available() > 0) {
  
    // read the incoming byte:
    incomingByte = Serial.read();
  
    // say what you got:
    // ECHO the value that was read, back to the serial port.
    //Serial.write(byteRead);
    Serial.print("I received: ");
    Serial.println(incomingByte);
   
   
  }
  delay(1000);

  */
}



void recvOneChar() {
  if (Serial.available() > 0) {
    receivedChar = Serial.read();
    newData = true;
  }
}

void showNewData() {
  if (newData == true) {
    Serial.print("ECHO: ");
    Serial.println(receivedChar);
    newData = false;
    //react(receivedChar);
  }
}




void react(char receivedChar) {
  switch (receivedChar) {
    case 'I':
      Serial.println("xIn");
      blink();
      break;
    case 'O':
      Serial.println("xOut");
      break;
    case 'U':
      Serial.println("yUp");
      break;
    case 'D':
      Serial.println("yDown");
      break;
    case 'L':
      Serial.println("zLeft");
      break;
    case 'R':
      Serial.println("zRight");
      break;   
  }
}


void blink(){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second

}
