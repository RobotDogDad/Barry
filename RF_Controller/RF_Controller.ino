//// Multifunction RF Controller

// Teensy 4.1 Microcontroller, 4.5V input
// All Vin to secondary components must be 3.3V

//// Pinout
// Pin 0: Digital Input, Left Red Button
// Pin 1: Digital Input, Left Yellow Button
// Pin 2: Digital Input, Left Blue Button
// Pin 3: Digital Input, Left White Button
// Pin 4: Digital Input, Right White Button
// Pin 5: Digital Input, Right Red Button
// Pin 6: Digital Input, Right Blue Button
// Pin 7: Digital Input, Right Yellow Button
// Pin 9: Digital Output, Radio CE
// Pin 10: Digital Output, Radio CS
// Pin 11: SPI, Radio MOSI
// Pin 12: SPI, Radio MISO
// Pin 13: SPI, Radio SCK
// Pin 18: I2C, Screen SDA
// Pin 19: I2C, Screen SCK
// Pin 26: Analog In, Right Stick X
// Pin 27: Analog In, Right Stick Y
// Pin 28: Digital Input, Right Stick Button
// Pin 37: Digital Input, Left Stick Button
// Pin 38: Analog In, Left Stick Y
// Pin 39: Analog In, Left Stick X



// Include Libraries and Definitions

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <SerLCD.h>
#include <Bounce.h>

#define joy1X A14
#define joy1Y A15
#define joy2X A12
#define joy2Y A13
#define CE 9
#define CSN 10
#define DisplayArrayLength 10


// Setup button debouncing
Bounce joy1button = Bounce(37, 10);
Bounce joy2button = Bounce(28, 10);

Bounce lbYellow = Bounce(1, 10);
Bounce lbRed = Bounce(0, 10);
Bounce lbWhite = Bounce(3, 10);
Bounce lbBlue = Bounce(2, 10);

Bounce rbYellow = Bounce(7, 10);
Bounce rbRed = Bounce(5, 10);
Bounce rbWhite = Bounce(4, 10);
Bounce rbBlue = Bounce(6, 10);

// RF_Controller Radio Setup
RF24 radio(CE, CSN, 4000000); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

// LCD Setup
SerLCD lcd; // Initialize with default address 0x72

// Timers
unsigned long currentMillis;
unsigned long ControllerMillis = 0;
unsigned long DisplayMillis = 0;

// Variable definitions
bool commState = 0;

// Data structure definitions
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
  int joy1buttonValue;
  int joy2XValue;
  int joy2YValue;
  int joy2buttonValue;
  int lbYellowValue;
  int lbRedValue;
  int lbWhiteValue;
  int lbBlueValue;
  int rbYellowValue;
  int rbRedValue;
  int rbWhiteValue;
  int rbBlueValue;
}
controlDef;
controlDef controlPak;

void setup() {



// Set Pins  
  pinMode(37, INPUT_PULLUP); //joy1button
  pinMode(28, INPUT_PULLUP); //joy2button
  pinMode(7, INPUT_PULLUP); //rbYellow
  pinMode(5, INPUT_PULLUP); //rbRed
  pinMode(4, INPUT_PULLUP); //rbWhite
  pinMode(6, INPUT_PULLUP); //rbBlue
  pinMode(1, INPUT_PULLUP); //lbYellow
  pinMode(0, INPUT_PULLUP); //lbRed
  pinMode(3, INPUT_PULLUP); //lbWhite
  pinMode(2, INPUT_PULLUP); //lbBlue


// Initialize I2C to Screen @ 400kHz
  Wire.begin();
  lcd.begin(Wire);
  Wire.setClock(400000);
  delay(2000);
  lcd.clear();
  lcd.print("I2C Online");
  delay(1000);
//  lcd.autoscroll();

// Initialize Teensy-to-PC Serial
  Serial.begin(115200);
  lcd.clear();  
  lcd.print("Serial Online");
  delay(1000);

// Initialize Radio
  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00001
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(100);
  lcd.clear();  
  lcd.println("Radio Online");
  delay(1000);
  lcd.clear();
}



void loop() {

// Timer Start
currentMillis = millis();


// Retrieve displayPak Data  
  radio.startListening();

      // Check for Radio Data
  if (radio.available()) {
      radio.read(&displayPak, sizeof(displayPak));
      ControllerMillis = currentMillis;
      Serial.println("Incoming Data");
      Serial.println(displayPak.corrected_angle);
    }

    // Radio Timeout Check
  if ((currentMillis - ControllerMillis) > 500) {
      commState = 0;
  }

  else if((currentMillis - ControllerMillis) < 500) {
    commState = 1;     
  }

// Switch Radio Modes
  radio.stopListening();
  delay(5);

// Update Buttons
  joy1button.update();
  joy2button.update();
  lbYellow.update();
  lbRed.update();
  lbWhite.update();
  lbBlue.update();
  rbYellow.update();
  rbRed.update();
  rbWhite.update();
  rbBlue.update();


// Update controlPak Data
  controlPak.joy1XValue = analogRead(joy1X);
  controlPak.joy1YValue = analogRead(joy1Y);
  controlPak.joy1buttonValue = joy1button.read();
  controlPak.joy2XValue = analogRead(joy2X);
  controlPak.joy2YValue = analogRead(joy2Y);
  controlPak.joy2buttonValue = joy2button.read();
  controlPak.lbYellowValue = lbYellow.read();
  controlPak.lbRedValue = lbRed.read();
  controlPak.lbWhiteValue = lbWhite.read();
  controlPak.lbBlueValue = lbBlue.read();
  controlPak.rbYellowValue = rbYellow.read();
  controlPak.rbRedValue = rbRed.read();
  controlPak.rbWhiteValue = rbWhite.read();
  controlPak.rbBlueValue = rbBlue.read();

// Send controlPak Data
  radio.write(&controlPak, sizeof(controlPak));

// Update Display
  if ((commState = 0)){
    lcd.println("No Comms");
    Serial.println("No Comms");
  }
  else if ((commState = 1)){
    lcd.println("Radio Connected");
    Serial.println("Radio Connected");
  }
 // else {  
 //   lcd.println(" ");
 // }

  int displayOutputs[DisplayArrayLength] = {controlPak.joy1buttonValue, controlPak.joy2buttonValue, controlPak.lbYellowValue, controlPak.lbRedValue, controlPak.lbWhiteValue, controlPak.lbBlueValue, controlPak.rbYellowValue, controlPak.rbRedValue, controlPak.rbWhiteValue, controlPak.rbBlueValue};
  //for (int x = 0; x < DisplayArrayLength; x++){
  //lcd.print(displayOutputs[x]);
 // }
  //lcd.println();
  Serial.println(currentMillis);
  Serial.println(ControllerMillis);

}