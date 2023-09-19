//// Barry the Ringbearer

// Teensy 4.1 Microcontroller, 5V input
// Vin to logic components 3.3v, Vin to LEDs 5V

//// Pinout
// Pin 0: RX, ODrive 1
// Pin 1: TX, ODrive 1
// Pin 2: PWM, Servo 1
// Pin 3: PWM, Servo 2
// Pin 4: PWM, Servo 3
// Pin 5: PWM, Servo 4
// Pin 7: RX, ODrive 2
// Pin 8: TX, ODrive 2
// Pin 9: Digital Output, Radio CE
// Pin 10: Digital Output, Radio CS
// Pin 11: SPI, Radio MOSI
// Pin 12: SPI, Radio MISO
// Pin 13: SPI, Radio SCK
// Pin 14: TX, RESERVED
// Pin 15: RX, RESERVED
// Pin 16, TX, RESERVED
// Pin 17, TX, RESERVED
// Pin 18: I2C SDA, RESERVED
// Pin 19: I2C SCK, RESERVED
// Pin 20: TX, RESERVED
// Pin 21: RX, RESERVED
// Pin 24: TX, RESERVED
// Pin 25: RX, RESERVED
// Pin 28: RX, RESERVED
// Pin 29: TX, RESERVED
// Pin 33: Digital Output, NeoPixels
// Pin 34: RX, RESERVED
// Pin 35: TX, RESERVED
// Pin 39: Analog In, RESERVED
// Pin 40: Analog In, RESERVED
// Pin 41: Analog In, RESERVED

// Library Imports
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include <PWMServo.h>
#include <DifferentialSteering.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


//Variable Declarations
//// Integers
int Servo1Value;
int Servo2Value;
int Servo3Value;
int Servo4Value;
int fPivYLimit = 32;
int LeftMotor;
int RightMotor;
int motornum = 0;
int requested_state;

//// Boolean


//// Floats
float StickLXScaled = 0;
float StickLYScaled = 0;
float StickRXScaled = 0;
float StickRYScaled = 0;
float DriveVel = 0;
float DriveRot = 0;

////  Counters
const unsigned long overflowThreshold = 1000000000UL;
unsigned long previousTime = 0;
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;
unsigned long remoteMillis;


// Instantiate Peripherals
////Radio
RF24 radio(9, 10, 4000000); // CE, CSN
const byte addresses[][6] = {"00002", "00001"};

////LEDs
Adafruit_NeoPixel ring = Adafruit_NeoPixel(24, 33, NEO_GRB + NEO_KHZ800);

////Servos
PWMServo servo1; // Right Elbow
PWMServo servo2; // Right Shoulder
PWMServo servo3; // Left Elbow
PWMServo servo4; // Left Shoulder

////ODrives
DifferentialSteering DiffSteer;
HardwareSerial& odrive_serial1 = Serial1;
HardwareSerial& odrive_serial2 = Serial2;

ODriveArduino odrive1(odrive_serial1);
ODriveArduino odrive2(odrive_serial2);

// Data Package
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

// Initialize Serial, Radio, Servo, LEDs & Drive
  Serial.begin(115200);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(100);
 
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);

  servo1.write(65);
  servo2.write(60);
  servo3.write(170);
  servo4.write(175);  

  ring.begin();

  displayPak.PingBack = 1;

  DiffSteer.begin(fPivYLimit);

  // ODrive uses 115200 baud
  odrive_serial1.begin(115200);
  odrive_serial2.begin(115200);

odrive_serial1 << "w axis0.controller.config.vel_limit " << 10.0f << '\n';
odrive_serial1 << "w axis0.motor.config.current_lim " << 10.0f << '\n';
odrive_serial2 << "w axis0.controller.config.vel_limit " << 10.0f << '\n';
odrive_serial2 << "w axis0.motor.config.current_lim " << 10.0f << '\n';

      odrive1.run_state(0, 8, false);
      requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      //Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive2.run_state(motornum, requested_state, false)) return;

      odrive2.run_state(0, 8, false);
      requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
      //Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive2.run_state(motornum, requested_state, false)) return;
    

}

void loop() {


  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event

    previousMillis = currentMillis;

    delay(5);
    radio.startListening();

        // Check for radio data
    if (radio.available()) {
        radio.read(&controlPak, sizeof(controlPak)); // Read data and record to memory
        remoteMillis = currentMillis; // Reset Watchdog

/*
        // Send Data to Serial 
        Serial.print("Left Stick X value is: ");
        Serial.println(controlPak.StickLXValue);
        Serial.print("Left Stick Y value is: ");
        Serial.println(controlPak.StickLYValue);
        Serial.print("Right Stick X value is: ");
        Serial.println(controlPak.StickRXValue);
        Serial.print("Right Stick Y value is: ");
        Serial.println(controlPak.StickRYValue);

        Serial.print("Left Stick Button state is: ");
        Serial.println(controlPak.StickLButtonValue);
        Serial.print("Right Stick Button state is: ");
        Serial.println(controlPak.StickRButtonValue);

        Serial.print("Left Bumper state is: ");
        Serial.println(controlPak.BumperLValue);
        Serial.print("Right Bumper state is: ");
        Serial.println(controlPak.BumperRValue);

        Serial.print("Left Select state is: ");
        Serial.println(controlPak.SelectLValue);
        Serial.print("Right Select state is: ");
        Serial.println(controlPak.SelectRValue);

        Serial.print("Down Button state is: ");
        Serial.println(controlPak.ButtonDValue);
        Serial.print("Left Button state is: ");
        Serial.println(controlPak.ButtonLValue);
        Serial.print("Right Button state is: ");
        Serial.println(controlPak.ButtonRValue);
        Serial.print("Up Button state is: ");
        Serial.println(controlPak.ButtonUValue);
*/
      }

      // Is the remote disconnected for too long ?
    if (currentMillis - remoteMillis > 500) { // Trigger watchdog if data hasn't been received for (500) milliseconds
        Serial.println("No Connection"); // Watchdog code goes here
    }

      // Switch radio from read to write (leave some time here to change modes)
      delay(5);
      radio.stopListening();
      delay(5);

      radio.write( &displayPak, sizeof(displayPak) ); // Send small packet to transmitter for its own internal watchdog

    //End of timed radio loop
  }



  ring.setPixelColor(0, ring.Color(5,25,5));
  ring.setPixelColor(1, ring.Color(5,25,5));
  ring.setPixelColor(2, ring.Color(5,25,5));
  ring.setPixelColor(3, ring.Color(5,25,5));
  ring.setPixelColor(4, ring.Color(5,25,5));
  ring.setPixelColor(5, ring.Color(5,25,5));
  ring.setPixelColor(6, ring.Color(5,25,5));
  ring.setPixelColor(7, ring.Color(5,25,5));
  ring.setPixelColor(8, ring.Color(5,25,5));
  ring.setPixelColor(9, ring.Color(5,25,5));
  ring.setPixelColor(10, ring.Color(5,25,5));
  ring.setPixelColor(11, ring.Color(5,25,5));
  ring.setPixelColor(12, ring.Color(5,25,5));
  ring.setPixelColor(13, ring.Color(5,25,5));
  ring.setPixelColor(14, ring.Color(5,25,5));
  ring.setPixelColor(15, ring.Color(5,25,5));
  ring.setPixelColor(16, ring.Color(5,25,5));    
  ring.setPixelColor(17, ring.Color(5,25,5));
  ring.setPixelColor(18, ring.Color(5,25,5));
  ring.setPixelColor(19, ring.Color(5,25,5));
  ring.setPixelColor(20, ring.Color(5,25,5));
  ring.setPixelColor(21, ring.Color(5,25,5));
  ring.setPixelColor(22, ring.Color(5,25,5));
  ring.setPixelColor(23, ring.Color(5,25,5));    

 
  ring.show();

// Stick Value Conversion
  StickLXScaled = ((controlPak.StickLXValue - 512.0)/512.0);
  StickLYScaled = ((controlPak.StickLYValue - 512.0)/512.0);
  StickRXScaled = ((controlPak.StickRXValue - 512.0)/512.0);
  StickRYScaled = ((controlPak.StickRYValue - 512.0)/512.0);

  if ((StickLXScaled < 0.1) && (StickLXScaled > -0.1)) {
    StickLXScaled = 0;
  }

  if ((StickLYScaled < 0.1) && (StickLYScaled > -0.1)) {
    StickLYScaled = 0;
  }

  if ((StickRXScaled < 0.1) && (StickRXScaled > -0.1)) {
    StickRXScaled = 0;
  }

  if ((StickRYScaled < 0.1) && (StickRYScaled > -0.1)) {
    StickRYScaled = 0;
  }
/*
    // Send Data to Serial 
    Serial.print("Left Stick X value is: ");
    Serial.println(StickLXScaled);
    Serial.print("Left Stick Y value is: ");
    Serial.println(StickLYScaled);
    Serial.print("Right Stick X value is: ");
    Serial.println(StickRXScaled);
    Serial.print("Right Stick Y value is: ");
    Serial.println(StickRYScaled);

    Serial.print("Left Stick Button state is: ");
    Serial.println(controlPak.StickLButtonValue);
    Serial.print("Right Stick Button state is: ");
    Serial.println(controlPak.StickRButtonValue);

    Serial.print("Left Bumper state is: ");
    Serial.println(controlPak.BumperLValue);
    Serial.print("Right Bumper state is: ");
    Serial.println(controlPak.BumperRValue);

    Serial.print("Left Select state is: ");
    Serial.println(controlPak.SelectLValue);
    Serial.print("Right Select state is: ");
    Serial.println(controlPak.SelectRValue);

    Serial.print("Down Button state is: ");
    Serial.println(controlPak.ButtonDValue);
    Serial.print("Left Button state is: ");
    Serial.println(controlPak.ButtonLValue);
    Serial.print("Right Button state is: ");
    Serial.println(controlPak.ButtonRValue);
    Serial.print("Up Button state is: ");
    Serial.println(controlPak.ButtonUValue);
*/

 // Drive Commands
  DriveVel = map(StickLYScaled, -1.00, 1.00, -127.00, 127.00);
  DriveRot = map(StickLXScaled, -1.00, 1.00, -127.00, 127.00);
  DiffSteer.computeMotors(DriveVel, DriveRot);
  LeftMotor = DiffSteer.computedLeftMotor();
  RightMotor = DiffSteer.computedRightMotor();

    Serial.print("Left Motor value is: ");
    Serial.println(LeftMotor);
    Serial.print("Right Motor value is: ");
    Serial.println(RightMotor);

 // Servo Commands
 
 if ((controlPak.ButtonDValue == LOW)) {
   servo1.write(65);
   servo2.write(60);
   servo3.write(170);
   servo4.write(175);  
 }

 else if ((controlPak.ButtonLValue == LOW)) {
   servo1.write(180);
   servo2.write(180);
   servo3.write(170);
   servo4.write(175);
 }

 else if ((controlPak.ButtonRValue == LOW)) {
   servo1.write(65);
   servo2.write(60);
   servo3.write(55);
   servo4.write(55);
 }
 
 else if ((controlPak.ButtonUValue == LOW)) {
   servo1.write(180);
   servo2.write(180);
   servo3.write(55);
   servo4.write(55);  
 }


 

 /* 
  
  Servo1Value = map(controlPak.StickRXValue, 0, 1024, 0, 180);
  servo1.write(Servo1Value);

  Servo2Value = map(controlPak.StickRXValue, 0, 1024, 0, 180);
  servo2.write(Servo2Value);

  Servo3Value = map(controlPak.StickRXValue, 0, 1024, 0, 180);
  servo3.write(180 - Servo3Value);

  Servo4Value = map(controlPak.StickRXValue, 0, 1024, 0, 180);
  servo4.write(180 - Servo4Value);

*/

}  
