// Library Imports
#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include "math.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// I/O Definitions
#define led 32
#define led_comms_ON 33
#define led_comms_OFF 34

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B


// Instantiate Radio
RF24 radio(9, 10, 4000000); // CE, CSN, SPEED
const byte addresses[][6] = {"00002", "00001"};

//Variable Declarations
////  Sensor values
float AccelY;
float AccelZ;
float AccelAngleX;
float GyroX;
float fusedPitch = 0;

////  Booleans
boolean buttonStateIn = 0;
boolean commState = 0;

////  Counters
const unsigned long overflowThreshold = 1000000000UL;
unsigned long previousTime = 0;


unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;

unsigned long remoteMillis;


// Data Package
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

// Initialize Serial, I2C and Radio
  Serial.begin(115200);
  delay(500); 
  
  Wire.begin();
  delay(10);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(100);

  // Check for IMU State
  if( myIMU.begin() ){
    Serial.println("Ready.");
  }
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) ){
    Serial.println("Loaded Settings.");
  }

}

void loop() {

    // Sensor Data Input
  AccelY = myIMU.readFloatAccelY();
  AccelZ = myIMU.readFloatAccelZ();
  GyroX = myIMU.readFloatGyroY();

  // Math
  //// Compute the time delta (in seconds)
  unsigned long currentTime = millis();
  if (currentTime < previousTime) { // Check for overflow
      currentTime += overflowThreshold;
    }

  //// Compute Accelerometer Angle
  AccelAngleX = -atan2(AccelY, AccelZ)*RAD_TO_DEG;
    //if(isnan(AccelAngleY)) {
  //  }

  float dt = (currentTime - previousTime) / 1000.0;
  fusedPitch = 0.98 * (fusedPitch += GyroX * dt) + 0.02 * AccelAngleX;
  previousTime = currentTime; // Save the current time for the next loop iteration

  //// Wrap around the pitch to keep it within the range of -180 to 180 degrees
  if (fusedPitch > 180) {
      fusedPitch -= 360;
    } else if (fusedPitch < -180) {
      fusedPitch += 360;
    }
  
    // Save to displayPak
    displayPak.corrected_angle = (float)fusedPitch;

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event

    previousMillis = currentMillis;

  delay(5);
  radio.startListening();

      // check for radio data
  if (radio.available()) {
      radio.read(&controlPak, sizeof(controlPak));
      remoteMillis = currentMillis;
    }

    // is the remote disconnected for too long ?
  if (currentMillis - remoteMillis > 500) {
      commState = 0;
      Serial.println("no data");
  }
  else {
    commState = 1;
    Serial.print("Joystick 1 X Value is: ");
    Serial.println(controlPak.joy1XValue);
    Serial.print("Joystick 1 Y Value is: ");
    Serial.println(controlPak.joy1YValue);
    Serial.print("Joystick 1 button state is: ");
    Serial.println(controlPak.joy1buttonValue);
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
  

    //end of timed radio loop
  }

  radio.write( &displayPak, sizeof(displayPak) );

}
