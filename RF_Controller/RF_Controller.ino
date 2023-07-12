#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define buttonOut 39
#define led 32

RF24 radio(9, 10, 4000000); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

boolean buttonStateOut = 0;
boolean buttonStateIn = 0;

void setup() {
  pinMode(buttonOut, INPUT_PULLUP);
  pinMode(32, OUTPUT);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
  delay(5);
  radio.startListening();

  radio.read(&buttonStateIn, sizeof(buttonStateIn));
  if (buttonStateIn == HIGH) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
  delay(5);

  radio.stopListening();
  
  buttonStateOut = 1-digitalRead(buttonOut);
  radio.write(&buttonStateOut, sizeof(buttonStateOut));
}