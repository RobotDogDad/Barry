#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define joy1button 36
#define led 32
#define led_comms_ON 33
#define led_comms_OFF 34
#define joy1X A15
#define joy1Y A14

RF24 radio(9, 10, 4000000); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

boolean buttonStateOut = 0;
boolean buttonStateIn = 0;
boolean commState = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;

unsigned long ControllerMillis;



typedef struct
{
  float corrected_angle;
}
displayDef;
displayDef displayPak;

typedef struct
{
  int joy1XValue;
  int joy1YValue;
  boolean joy1buttonValue;
}
controlDef;
controlDef controlPak;

void setup() {
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(joy1button, INPUT_PULLUP);

  Serial.begin(115200);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(100);
}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event

    previousMillis = currentMillis;

  delay(5);
  radio.startListening();

      // check for radio data
  if (radio.available()) {
      radio.read(&displayPak, sizeof(displayPak));
      ControllerMillis = currentMillis;
      Serial.println("Incoming Data");
      Serial.println(displayPak.corrected_angle);
    }



    // is the remote disconnected for too long ?
  if (currentMillis - ControllerMillis > 500) {
      commState = 0;
      Serial.println("no data");
  }
  else {
    commState = 1;      
  }

  if (buttonStateIn == HIGH) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }

  if (commState == 1) {
    digitalWrite(led_comms_ON, HIGH);
    digitalWrite(led_comms_OFF, LOW);
  }
  else {
    digitalWrite(led_comms_OFF, HIGH);
    digitalWrite(led_comms_ON, LOW);
  }
  delay(5);

  radio.stopListening();
  }

  controlPak.joy1buttonValue = digitalRead(joy1button);
  controlPak.joy1XValue = analogRead(joy1X);
  controlPak.joy1YValue = analogRead(joy1Y);
  
  radio.write(&controlPak, sizeof(controlPak));
    //end of timed radio loop
  
}