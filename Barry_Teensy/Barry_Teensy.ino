#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define buttonOut 39
#define led 32
#define led_comms_ON 33
#define led_comms_OFF 34

RF24 radio(9, 10, 4000000); // CE, CSN
const byte addresses[][6] = {"00002", "00001"};

boolean buttonStateOut = 0;
boolean buttonStateIn = 0;
boolean commState = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;

unsigned long remoteMillis;



void setup() {
  pinMode(buttonOut, INPUT_PULLUP);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(34, OUTPUT);

  Serial.begin(115200);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event

    previousMillis = currentMillis;

  delay(5);
  radio.startListening();

      // check for radio data
  if (radio.available()) {
      radio.read(&buttonStateIn, sizeof(buttonStateIn));
      remoteMillis = currentMillis;
    }

    // is the remote disconnected for too long ?
  if (currentMillis - remoteMillis > 500) {
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
  
  buttonStateOut = 1-digitalRead(buttonOut);
  radio.write(&buttonStateOut, sizeof(buttonStateOut));
    //end of timed radio loop
  }
}
