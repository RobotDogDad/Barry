#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(8, 8, NEO_GRB + NEO_KHZ800);

#define StickLX A15
#define StickLY A14
#define StickLButton 37
#define StickRX A12
#define StickRY A13
#define StickRButton 28

#define BumperL 1
#define BumperR 3

#define SelectL 0
#define SelectR 2

#define ButtonD 7
#define ButtonL 6
#define ButtonR 4
#define ButtonU 5


RF24 radio(9, 10, 4000000); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;

unsigned long ControllerMillis;
unsigned long RadioTimeDelta;

typedef struct
{
  int PingBack;
}
displayDef;
displayDef displayPak;

typedef struct
{
  int StickLXValue;
  int StickLYValue;
  boolean StickLButtonValue;
  int StickRXValue;
  int StickRYValue;
  boolean StickRButtonValue;
  boolean BumperLValue;
  boolean BumperRValue;
  boolean SelectLValue;
  boolean SelectRValue;
  boolean ButtonDValue;
  boolean ButtonLValue;
  boolean ButtonRValue;
  boolean ButtonUValue;
}
controlDef;
controlDef controlPak;

void setup() {

  Serial.begin(115200);
  pinMode(StickLButton, INPUT_PULLUP);
  pinMode(StickRButton, INPUT_PULLUP);
  pinMode(BumperL, INPUT_PULLUP);
  pinMode(BumperR, INPUT_PULLUP);
  pinMode(SelectL, INPUT_PULLUP);
  pinMode(SelectR, INPUT_PULLUP);
  pinMode(ButtonD, INPUT_PULLUP);
  pinMode(ButtonL, INPUT_PULLUP);
  pinMode(ButtonR, INPUT_PULLUP);
  pinMode(ButtonU, INPUT_PULLUP);

  pixels.begin();


  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(100);
}

void loop() {

  currentMillis = millis();
  RadioTimeDelta = (currentMillis - ControllerMillis);

  if (currentMillis - previousMillis >= 10) {  // start timed event

    previousMillis = currentMillis;

  delay(5);
  radio.startListening();


      // check for radio data
  if (radio.available()) {
      radio.read(&displayPak, sizeof(displayPak));
      ControllerMillis = currentMillis;
      Serial.println("Good Connection");
      Serial.println(displayPak.PingBack);
    }



    // is the remote disconnected for too long ?
  if (currentMillis - ControllerMillis > 500) {
      Serial.println("No Connection");
  }
  delay(5);

  radio.stopListening();
  }

  controlPak.StickLXValue = analogRead(StickLX);
  controlPak.StickLYValue = analogRead(StickLY);
  controlPak.StickLButtonValue = digitalRead(StickLButton);
  controlPak.StickRXValue = analogRead(StickRX);
  controlPak.StickRYValue = analogRead(StickRY);
  controlPak.StickRButtonValue = digitalRead(StickRButton);

  controlPak.BumperLValue = digitalRead(BumperL);
  controlPak.BumperRValue = digitalRead(BumperR);

  controlPak.SelectLValue = digitalRead(SelectL);
  controlPak.SelectRValue = digitalRead(SelectR);

  controlPak.ButtonDValue = digitalRead(ButtonD);
  controlPak.ButtonLValue = digitalRead(ButtonL);
  controlPak.ButtonRValue = digitalRead(ButtonR);
  controlPak.ButtonUValue = digitalRead(ButtonU);
  
  radio.write(&controlPak, sizeof(controlPak));
    //end of timed radio loop

  if (RadioTimeDelta < 63) {
  pixels.setPixelColor(0, pixels.Color(0,5,0));
  pixels.setPixelColor(1, pixels.Color(0,5,0));
  pixels.setPixelColor(2, pixels.Color(0,5,0));
  pixels.setPixelColor(3, pixels.Color(0,5,0));
  pixels.setPixelColor(4, pixels.Color(0,5,0));
  pixels.setPixelColor(5, pixels.Color(0,5,0));
  pixels.setPixelColor(6, pixels.Color(0,5,0));
  pixels.setPixelColor(7, pixels.Color(0,5,0));
  }

  else if (RadioTimeDelta < 125) {
  pixels.setPixelColor(0, pixels.Color(0,5,0));
  pixels.setPixelColor(1, pixels.Color(0,5,0));
  pixels.setPixelColor(2, pixels.Color(0,5,0));
  pixels.setPixelColor(3, pixels.Color(0,5,0));
  pixels.setPixelColor(4, pixels.Color(0,5,0));
  pixels.setPixelColor(5, pixels.Color(0,5,0));
  pixels.setPixelColor(6, pixels.Color(0,5,0));
  pixels.setPixelColor(7, pixels.Color(5,0,0));
  }

  else if (RadioTimeDelta < 188) {
  pixels.setPixelColor(0, pixels.Color(0,5,0));
  pixels.setPixelColor(1, pixels.Color(0,5,0));
  pixels.setPixelColor(2, pixels.Color(0,5,0));
  pixels.setPixelColor(3, pixels.Color(0,5,0));
  pixels.setPixelColor(4, pixels.Color(0,5,0));
  pixels.setPixelColor(5, pixels.Color(0,5,0));
  pixels.setPixelColor(6, pixels.Color(5,0,0));
  pixels.setPixelColor(7, pixels.Color(5,0,0));
  }

  else if (RadioTimeDelta < 250) {
  pixels.setPixelColor(0, pixels.Color(0,5,0));
  pixels.setPixelColor(1, pixels.Color(0,5,0));
  pixels.setPixelColor(2, pixels.Color(0,5,0));
  pixels.setPixelColor(3, pixels.Color(0,5,0));
  pixels.setPixelColor(4, pixels.Color(0,5,0));
  pixels.setPixelColor(5, pixels.Color(5,0,0));
  pixels.setPixelColor(6, pixels.Color(5,0,0));
  pixels.setPixelColor(7, pixels.Color(5,0,0));
  }  

  else if (RadioTimeDelta < 313) {
  pixels.setPixelColor(0, pixels.Color(0,5,0));
  pixels.setPixelColor(1, pixels.Color(0,5,0));
  pixels.setPixelColor(2, pixels.Color(0,5,0));
  pixels.setPixelColor(3, pixels.Color(0,5,0));
  pixels.setPixelColor(4, pixels.Color(5,0,0));
  pixels.setPixelColor(5, pixels.Color(5,0,0));
  pixels.setPixelColor(6, pixels.Color(5,0,0));
  pixels.setPixelColor(7, pixels.Color(5,0,0));
  }  

  else if (RadioTimeDelta < 375) {
  pixels.setPixelColor(0, pixels.Color(0,5,0));
  pixels.setPixelColor(1, pixels.Color(0,5,0));
  pixels.setPixelColor(2, pixels.Color(0,5,0));
  pixels.setPixelColor(3, pixels.Color(5,0,0));
  pixels.setPixelColor(4, pixels.Color(5,0,0));
  pixels.setPixelColor(5, pixels.Color(5,0,0));
  pixels.setPixelColor(6, pixels.Color(5,0,0));
  pixels.setPixelColor(7, pixels.Color(5,0,0));
  }  

  else if (RadioTimeDelta < 438) {
  pixels.setPixelColor(0, pixels.Color(0,5,0));
  pixels.setPixelColor(1, pixels.Color(0,5,0));
  pixels.setPixelColor(2, pixels.Color(5,0,0));
  pixels.setPixelColor(3, pixels.Color(5,0,0));
  pixels.setPixelColor(4, pixels.Color(5,0,0));
  pixels.setPixelColor(5, pixels.Color(5,0,0));
  pixels.setPixelColor(6, pixels.Color(5,0,0));
  pixels.setPixelColor(7, pixels.Color(5,0,0));
  }  

  else if (RadioTimeDelta < 500) {
  pixels.setPixelColor(0, pixels.Color(0,5,0));
  pixels.setPixelColor(1, pixels.Color(5,0,0));
  pixels.setPixelColor(2, pixels.Color(5,0,0));
  pixels.setPixelColor(3, pixels.Color(5,0,0));
  pixels.setPixelColor(4, pixels.Color(5,0,0));
  pixels.setPixelColor(5, pixels.Color(5,0,0));
  pixels.setPixelColor(6, pixels.Color(5,0,0));
  pixels.setPixelColor(7, pixels.Color(5,0,0));
  }  

  else {
  pixels.setPixelColor(0, pixels.Color(5,0,0));
  pixels.setPixelColor(1, pixels.Color(5,0,0));
  pixels.setPixelColor(2, pixels.Color(5,0,0));
  pixels.setPixelColor(3, pixels.Color(5,0,0));
  pixels.setPixelColor(4, pixels.Color(5,0,0));
  pixels.setPixelColor(5, pixels.Color(5,0,0));
  pixels.setPixelColor(6, pixels.Color(5,0,0));
  pixels.setPixelColor(7, pixels.Color(5,0,0));
  }  


  pixels.show();
 
}