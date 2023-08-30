


// ------------ Library Inclusions ------------ // 
#include <math.h>
#include <Wire.h>
#include <SCServo.h> 
#include "SparkFun_VCNL4040_Arduino_Library.h" 
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include <movingAvg.h> 


// ------------ Definitions ------------ // 

// IR SENSOR
#define EPS 0.005                               // Point below which a value should be considered zero 

// Actuator 
#define MAX_ID 6                                // Number of motors 
#define SCS_2_RAD 0.00511327                    // Convert SCS Format to Radians

#define MOTOR_OFFSET_RADIANS 0

#define MOTOR_SPEED_SLOW_RADIANS_PER_SECOND 1.0
#define MOTOR_SPEED_FAST_RADIANS_PER_SECOND 2.0

#define MOTOR_ACCELERATION_RADIONS_PER_SECOND_SQUARED -0.1

#define YAW_RANGE_LOW_DEGREES 60
#define YAW_RANGE_HIGH_DEGREES 75

#define Y_MAP_SCALE 1.5

#define PROXIMITY_CALIBRATION_SAMPLES 5
#define PROXIMITY_DIFFERENCE_THRESHOLD 50
#define DROP_TIME_LIMIT 200
#define MOVING_AVG_SAMPLES 15
#define ANGLE_AVG_SAMPLES 5


// Drops per Bottle | Threshold | Max Suspension Time | Pitch Min Limit | Pitch Max Limit | Roll Limit

#define DROPS_MAX_INDEX 0
#define DROPS_THRESHOLD_INDEX 1
#define MAX_SUSPENSION_TIME_INDEX 2
#define PITCH_MIN_LIMIT_INDEX 3
#define PITCH_MAX_LIMIT_INDEX 4
#define ROLL_MAX_LIMIT_INDEX 5



#define SECOND_TO_MILLISECONDS 1000
#define MILLISECOND_TO_SECONDS 0.001
#define DEG2RAD 0.0174533

#define TIMESTEP_MILLISECONDS 20

#define SERIAL_TIMEOUT 4000

// ------------ Global Variables ------------ // 

// Actuator 
SCSCL sc; 

long int motorBaudRate = 38400;                 // BaudRate used between Arduino and Motors 
long int usbBaudRate = 115200;                  // BaudRate used between MATLAB and Arduino 

double q[1]={0}; 
u8 ID[1]={1}; 
int numID = 1; 
const int motorOffset[MAX_ID] = {0,-512,-512,-512,-512,-512};  

// IR SENSOR 
VCNL4040 proximitySensor;
unsigned int proximity = 0; 
unsigned int baseline = 0; 
movingAvg pMovingAvg(MOVING_AVG_SAMPLES);
movingAvg xMovingAvg(ANGLE_AVG_SAMPLES);
movingAvg yMovingAvg(ANGLE_AVG_SAMPLES);


// Button 
int buttonState = 0;         // variable for reading the pushbutton status 

// Pins 
const int buttonPin = 3;     // the number of the pushbutton pin 
const int ledPin = 4; 
const int powerLedPin = 5;
const int batteryMonitorPin = A0; 

// Battery Monitor
const float minVoltage = 6.4;

// Timers 
unsigned long tick = 0, tock = 0, timer = 0, ticDrop = 0, tocDrop = 0;

// Accelerometer
float x, y, z;
int degreesX = 0, yMap = 0;

// BLE
BLEService ocumateService("20B10010-E8F2-537E-4F6C-D104768A1214"); // create service
BLEByteCharacteristic dropsCharacteristic("20B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);


// ------------ CONTROLS ------------ // 

// Variables 
double startPos = 0, endPos = (15/8)*PI, runPos = 0;  // Actuator positions.
double startSpeed = MOTOR_SPEED_FAST_RADIANS_PER_SECOND, endSpeed = MOTOR_SPEED_SLOW_RADIANS_PER_SECOND, runSpeed = 0; 
int dropCount = 0;
const int angleUpper = 90, angleLower = 65;

bool validAngle = false;

bool reset = false; // Reset LED display.
bool usbConnected = true;

int tmpProxDiffTresh = PROXIMITY_DIFFERENCE_THRESHOLD;

// ------------ MEDICATION ------------ // 

int medSelect = 0;
char medName[3][5] = {"CMB0", "XLC0", "LTN0"};

// Drops per Bottle | Threshold | Max Suspension Time | Pitch Min Limit | Pitch Max Limit | Roll Limit
int medCal[3][6] = {
                  {145,10,500,70,90,10},  // Combigan
                  {87,50,0,0,0,0},  // Xalacom
                  {83,50,0,0,0,0}   // Latanoprost
                  };



// States 
static enum
{ 
  IDLE, 
  DISPENSING, 
  SUSPENDED
} state = IDLE;


// ------------ SETUP ------------ // 
void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(powerLedPin, OUTPUT);

  // Ensure LEDs are off to start
  digitalWrite(ledPin, LOW);
  digitalWrite(powerLedPin, LOW);

  // Setup USB Serial 
  Serial.begin(usbBaudRate);    // Set serial baud rate for USB 
  tick = millis();
  int tmp = 0;
  while(!Serial) { // Wait For Serial To Connect 
    // tmp = millis() - tick;
    if ((millis() - tick) > SERIAL_TIMEOUT) {
      usbConnected = false;
      break;
    }
  } 
  // Serial.println(tmp);              
  
  if (usbConnected) {
    Serial.println("USB serial initialized.");
  }
  
  // Serial.println("Test 0");
  // Initialise BLE
  if (!BLE.begin()) {
    Serial.println("BLE Failed");
    if (usbConnected) {
      Serial.println("starting Bluetooth® Low Energy module failed!");
    }
    Serial.println("Entering LED Error State");
    errorLED();
  }

  
  BLE.setLocalName("Anna's Ocumate");   // set the local name peripheral advertises
  BLE.setAdvertisedService(ocumateService); // set the UUID for the service this peripheral advertises:
  ocumateService.addCharacteristic(dropsCharacteristic); //add the characteristics to the service
  BLE.addService(ocumateService);  // add the service
  dropsCharacteristic.setEventHandler(BLEWritten, dropsCharacteristicWritten);
  BLE.advertise(); // start advertising
 

  if (usbConnected) {
    Serial.println("Bluetooth® device active, waiting for connections...");
  }

  // Setup Motor Serial 
  Serial1.begin(motorBaudRate); // Set serial baud rate for motors 
  while(!Serial1);              // Wait For Serial To Connect 
  sc.pSerial = &Serial1;        // Assign serial port to motor 

  Serial.println("Test 3");
  if (usbConnected) {
    Serial.println("Motor serial initialized.");
  }

  // Initialise IMU
  if (!IMU.begin()) {
    if (usbConnected) {
      Serial.println("Failed to initialize IMU!");
    }
    errorLED();
  }
  if (usbConnected) {
    Serial.println("IMU initialized.");
  }

  //Join i2c bus 
  Wire.begin();

  if (usbConnected) {
    Serial.println("I2C bus initialized.");
  }

  // Return Motor To Start Position 
  q[0] = startPos; 
  driveMotorPos(q);

  if (usbConnected) {
    Serial.println("Motor set to start position.");
  }

  // Initialise Proximity Sensor
  proximitySensor.begin();
  proximitySensor.powerOnProximity(); //Turn on proximity sensor

  if (usbConnected) {
    Serial.println("Proximity sensor initialized.");
  }

  if (usbConnected) {
    Serial.println("Calibrating proximity baseline...");
  }
  for(int i = 0; i < PROXIMITY_CALIBRATION_SAMPLES; i++) // Average over five readings taken one second apart.
  {
    baseline += proximitySensor.getProximity();    // Read baseline proximity value.
    delay(0.25*SECOND_TO_MILLISECONDS); // Wait for one second.
  }

  baseline /= PROXIMITY_CALIBRATION_SAMPLES;
  if (usbConnected) {
    Serial.println("Baseline proximity value is " + String(baseline) + ".");
  }

  // DEBUG - Force Baseline
  baseline = 92;

  pMovingAvg.begin();
  xMovingAvg.begin();
  yMovingAvg.begin();

  delay(50);
  runPos = startPos;
  if (usbConnected) {
    Serial.println("Ready");
  }


}


// ------------ LOOP ------------ // 
void loop() {
  tick = millis();

  //Check device battery level
  checkBatteryLevel();

  BLE.poll(); // poll for Bluetooth® Low Energy events
  dropsCharacteristic.writeValue(dropCount);  // Broadcast number of detected drops

  proximity = proximitySensor.getProximity();    // Read IR sensor value.
  
  // Serial.println("Baseline proximity value is " + String(baseline) + ".");

  unsigned int difference = abs((int) (proximity - baseline));
  double avgDiff = pMovingAvg.reading(difference);
  
  

  // if (usbConnected) {
  //   Serial.println(avgDiff);
  // }

  // Read gyroscope & turn on LED
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    

    // X - Tilt
    // Y - Rotation (while fully tilted)

    // Serial.print(x);
    // Serial.print(", ");
    // Serial.print(y);
    // Serial.print(", ");
    // Serial.println(y*cos(x));
    // Serial.println(z);


    if (x > 0) {
      x = 100 * x;
      degreesX = map(x, 0, 97, 0, 90);
    }
    else {
      x = 100 * x;
      degreesX = map(x, 0, -100, 0, 90);
    }

    if (y > 0) {
      yMap = 100 * y * (1 + Y_MAP_SCALE*sin(degreesX*DEG2RAD));
    }
    else {
      yMap = -100 * y * (1 + Y_MAP_SCALE*sin(degreesX*DEG2RAD));
    }

    double degreesXAvg = xMovingAvg.reading(degreesX);
    double yMapAvg = yMovingAvg.reading(yMap);

    // Serial.print(degreesXAvg);
    // Serial.print(", ");
    // // // Serial.println(y*sin(degreesX*DEG2RAD));
    // Serial.println(yMapAvg);

    // Serial.println(degreesX);
    if (degreesXAvg < (90-medCal[medSelect][PITCH_MIN_LIMIT_INDEX]) && degreesXAvg > (90-medCal[medSelect][PITCH_MAX_LIMIT_INDEX]) && yMapAvg < medCal[medSelect][ROLL_MAX_LIMIT_INDEX]) {
      // Serial.println("DROP NOW!");
      validAngle = true;
      digitalWrite(ledPin, HIGH);
    }
    else {
      validAngle = false;
      digitalWrite(ledPin, LOW);
    }

  }

  // VALID ANGLE OVERIDE
  // validAngle = true;

  // DEBUG
  // int tmpProxDiffTresh = pow(cos((degreesX) * PI / 180), 2) * PROXIMITY_DIFFERENCE_THRESHOLD;
  // if (usbConnected) {
  //   Serial.println(tmpProxDiffTresh);
  // }

  // Drive motor.
  q[0] = runPos; 
  driveMotorPos(q);

  // STATE MACHINE
  switch (state) { 
    case IDLE: 
      runPos = 0;
      runSpeed = 0;
      buttonState = digitalRead(buttonPin); // Read button state. 
      // Serial.println(buttonState);
      if (buttonState == HIGH && validAngle) {
        // tmpProxDiffTresh = pow(cos((degreesX) * PI / 180), 3) * PROXIMITY_DIFFERENCE_THRESHOLD; // Set Treshold based on angle
        state = DISPENSING;
        runPos = MOTOR_OFFSET_RADIANS;
        runSpeed = startSpeed;
        if (usbConnected) {
          Serial.println("Button pressed!");
        }
      }
      break;

    case DISPENSING:
      if (avgDiff > medCal[medSelect][DROPS_THRESHOLD_INDEX]) { 
        state = SUSPENDED;
        ticDrop = millis();
        if (usbConnected) {
          Serial.println("Suspension detected!");
          // Serial.println(tmpProxDiffTresh);
          
        }
      }
      else 
      {
        state = DISPENSING;
      }

      break;
    
    case SUSPENDED: 

      tocDrop = millis(); 

      if ((tocDrop-ticDrop) >= medCal[medSelect][MAX_SUSPENSION_TIME_INDEX]) {
        state = IDLE;
        if (usbConnected) {
          Serial.println("Warning: Max drop suspension time exceed!"); 
        }
      }

      if (difference < medCal[medSelect][DROPS_THRESHOLD_INDEX]) { 
        state = IDLE;
        dropCount++;
        tocDrop = millis();
        if (usbConnected) {
          Serial.print("Drop detected. Drop time: "); 
          Serial.print(tocDrop-ticDrop);
          Serial.print("ms. Tilt angle: "); 
          Serial.print(90-degreesX);
          Serial.println(" degrees"); 

          
          // Serial.println(tmpProxDiffTresh);
          
        }
      } 
      else 
      {  
        state = SUSPENDED;
      }

      break;

    default:  
      state = IDLE; 
      break; 
  }


  // Adjust motor position.
  if (state != IDLE)
  {
    runSpeed += MOTOR_ACCELERATION_RADIONS_PER_SECOND_SQUARED * TIMESTEP_MILLISECONDS * MILLISECOND_TO_SECONDS;
    runPos += runSpeed * TIMESTEP_MILLISECONDS * MILLISECOND_TO_SECONDS;

    if (runSpeed <= endSpeed)
    {
      runSpeed = endSpeed;
    }

    if (runPos >= endPos) 
    { 
      state = IDLE;
      if (usbConnected) {
        Serial.print("Warning: Max extension reached! Tilt angle: "); 
        Serial.print(90-degreesX);
        Serial.println(" degrees"); 
      }
    }
  }

  // Wait until the full time-step unless running late.
  tock = millis();
  timer = tock - tick;

  if (timer > TIMESTEP_MILLISECONDS)
  {
    if (usbConnected) {
      Serial.println("Warning: Running late by " + String(timer - TIMESTEP_MILLISECONDS) + " milliseconds!");
    }
    return; // Exit loop early.
  }

  delay(TIMESTEP_MILLISECONDS - timer);

}

// ------------------------------------------------------------------------ //
// --                             FUNCTIONS                              -- //
// ------------------------------------------------------------------------ //



// ------------ MOTOR FUNCTIONS ------------ //
void driveMotorPos(double *q) {
  
  // Drive motors to desired position by converting input to motor command

  // External Variables
  // @ q            - Input joint-space position in rad

  // Internal Variables
  // @ scsVel         - Array of mapped velocity values to be 
  //          sent to motors
  // @ sc             - Motor object allowing access to library functions

  // Internal Functions
  // @ rad2scs()      - Convert rad/s to motor command
  // @ SyncWriteVel() - Send motor commands to all motors simultaniously
  //    Takes:  ID[]        - array of motor ID numbers
  //            IDN         - number of motors being addressed
  //            Positions[] - array of desired positions
  //            Time        - how long it should take to move. Default: 0
  //            Speed       - how fast a move should be performed. Default: 0


  u16 scsPos[numID];
  
  
  for (int i = 0; i < numID; i++) {
    scsPos[i] = (int) (q[i] / SCS_2_RAD);   // Convert radian input to SCS input format
    scsPos[i] -= motorOffset[ID[i]-1];            // Compensate for motor offset
    
    if (scsPos[i] < 0) {
      scsPos[i] = 0;
    }
    else if (scsPos[i] > 1024) {
      scsPos[i] = 1024;
    }
  }

  // Send position commands to all motors simultaniously
  sc.SyncWritePos2(ID, numID, scsPos, 10, 0);
}

void dropsCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (usbConnected) {
    Serial.print("Characteristic event, written: ");
  }
  if (!dropsCharacteristic.value()) {
    if (usbConnected) {
      Serial.println("Reset");
    }
    dropCount = 0;
    // digitalWrite(ledPin, HIGH);
  }
}

void errorLED() {

  if (usbConnected) {
    Serial.println("ERROR");
  }

  while(1) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

void lowPowerLED(float voltage = -1) {

  if (usbConnected) {
    Serial.print("LOW POWER: ");
    Serial.print(voltage);
    Serial.println("V");
  }

  // while(1) {
  digitalWrite(powerLedPin, HIGH);
  // }
}

void checkBatteryLevel() {

  int rawBattVoltage = analogRead(batteryMonitorPin);
  float voltage = rawBattVoltage * (3.30 / 1023.00) * 3; //convert the value to a true voltage.

  // Serial.print("Raw Voltage: ");
  // Serial.print(rawBattVoltage);
  // Serial.println("V");

  // Serial.print("Voltage: ");
  // Serial.print(voltage);
  // Serial.println("V");

  if (voltage < minVoltage) {
    lowPowerLED(voltage);  // Enter Low Power Alert State
  }

}
