

// TO-DO
// * Have an over run after max extension is reached as IR value increases with pressure post-max
// * Check why not registering the correct value
// * Extend the max range of the motor and check that it is adequitely compressing Xalacom
// - If not, try velocity mode to go beyond max limit.  

// 66
// 82

// * Add press and hold functionality to go into a medication selection mode (hardware only)
// * Impliment double-press button to change (manually)
// * OR use baseline value to detect removal (or press and hold button and go into recalibrate mode) choose medication & validate with green light flashes (1,2 & 3)
// - Need to check registered value when not plugged in and have check to adapt calibration values if Serial is detected




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


// Drops per Bottle | Low Voltage Baseline |Baseline | Threshold | Max Suspension Time | Pitch Min Limit | Pitch Max Limit | Roll Limit
#define NUM_MEDS 3
#define EPS_MED_SELECT 4
#define DROPS_MAX_INDEX 0
#define DROPS_LV_BASELINE_INDEX 1
#define DROPS_BASELINE_INDEX 2
#define DROPS_THRESHOLD_INDEX 3
#define MAX_SUSPENSION_TIME_INDEX 4
#define PITCH_MIN_LIMIT_INDEX 5
#define PITCH_MAX_LIMIT_INDEX 6
#define ROLL_MAX_LIMIT_INDEX 7

#define DROP_MODE 0
#define CALIBRATE_MODE 1
#define BUTTON_MODE_RESET -1
#define MODE_BUTTON_THRESHOLD 1000

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
int baseline = 0; 
int newBaseline = 0;
movingAvg pMovingAvg(MOVING_AVG_SAMPLES);
movingAvg xMovingAvg(ANGLE_AVG_SAMPLES);
movingAvg yMovingAvg(ANGLE_AVG_SAMPLES);


// Button 
int buttonState = 0;         // variable for reading the pushbutton status 
int buttonStateOld = 0;
int buttonMode = BUTTON_MODE_RESET;

// Pins 
const int buttonPin = 3;     // the number of the pushbutton pin 
const int ledPin = 4; 
const int powerLedPin = 5;
const int batteryMonitorPin = A0; 

// Battery Monitor
const float minVoltage = 6.4;

// Timers 
unsigned long tick = 0, tock = 0, timer = 0, ticDrop = 0, tocDrop = 0, ticButton = 0, buttonTimer = 0, ticOverRun = 0;

// Accelerometer
float x, y, z;
int degreesX = 0, yMap = 0;

// BLE
BLEService ocumateService("20B10010-E8F2-537E-4F6C-D104768A1214"); // create service
BLEByteCharacteristic dropsCharacteristic("20B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);


// ------------ CONTROLS ------------ // 

int medSelect = 2;
bool disableAngleBounds = false;
bool printIR = true;


// Variables 
// Combigan -> (15/8)*PI
double startPos = 0, endPos = (2.2)*PI, runPos = 0;  // Actuator positions.
double startSpeed = MOTOR_SPEED_FAST_RADIANS_PER_SECOND, endSpeed = MOTOR_SPEED_SLOW_RADIANS_PER_SECOND, runSpeed = 0; 
int dropCount = 0;
const int angleUpper = 90, angleLower = 65;

bool validAngle = false;

bool reset = false; // Reset LED display.
bool usbConnected = true;

int tmpProxDiffTresh = PROXIMITY_DIFFERENCE_THRESHOLD;

// ------------ MEDICATION ------------ // 


char medName[NUM_MEDS][5] = {"CMB0", "XLC0", "LTN0"};

// Drops per Bottle | Low Voltage Baseline | Baseline | Threshold | Max Suspension Time | Pitch Min Limit | Pitch Max Limit | Roll Limit
int medCal[NUM_MEDS][8] = {
                  {145,74,92,10,500,70,90,10},  // Combigan
                  {87,64,82,90,500,82,90,7},  // Xalacom - 45
                  {145,59,75,80,500,80,90,8}   // Latanoprost
                  };



// States 
static enum
{ 
  IDLE,
  CALIBRATING,
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

  medSelect = -1;
  // for (int i = 0; i < NUM_MEDS; i++) {
  //   // if (abs(baseline - medCal[i][DROPS_BASELINE_INDEX]) < EPS_MED_SELECT) {
  //   if (((abs(baseline - medCal[i][DROPS_BASELINE_INDEX]) < EPS_MED_SELECT) && usbConnected) || ((abs(baseline - medCal[i][DROPS_LV_BASELINE_INDEX]) < EPS_MED_SELECT) && !usbConnected)) {
  //     medSelect = i;
  //     break;
  //   }
  //   else {
  //     medSelect = -1;
  //   }
  // }

  if (medSelect != -1) {
    Serial.print("Selected Medication: ");
    Serial.println(medName[medSelect]);
  }
  else {
    Serial.println("No Matching Medication Found!");
  }

  medSelectLED();

  // DEBUG - Force Baseline
  // baseline = 92;

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

  unsigned int difference = abs((int) (proximity - medCal[medSelect][DROPS_BASELINE_INDEX]));
  double avgDiff = pMovingAvg.reading(difference);
  
  
  if (printIR) {
    // Serial.println(avgDiff);
    Serial.println(proximity);
    
  }

  // Read gyroscope & turn on LED
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    

    // X - Tilt
    // Y - Rotation (while fully tilted)

    // Serial.print(x);
    // Serial.print(", ");
    // Serial.print(y);
    // Serial.print(", ");
    // // Serial.println(y*cos(x));
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
    if (degreesXAvg < (90-medCal[medSelect][PITCH_MIN_LIMIT_INDEX]) && degreesXAvg > (90-medCal[medSelect][PITCH_MAX_LIMIT_INDEX]) && yMapAvg < medCal[medSelect][ROLL_MAX_LIMIT_INDEX] && z < 0) {
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
  if (disableAngleBounds) {
    validAngle = true;
  }

  // Indicate if no valid medication is selected
  noMedLED();


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

      if (buttonState == HIGH && buttonStateOld == LOW) {
        ticButton = millis();
      }
      else if (buttonState == LOW && buttonStateOld == HIGH) {
        buttonTimer = millis() - ticButton;
        if (buttonTimer > MODE_BUTTON_THRESHOLD) {
          buttonMode = CALIBRATE_MODE;
        }
        else {
          buttonMode = DROP_MODE;
        }

        Serial.println(buttonMode);

      }

      buttonStateOld = buttonState;

      

      

      // Serial.println(buttonState);
      if (buttonMode == DROP_MODE && validAngle) {
        // tmpProxDiffTresh = pow(cos((degreesX) * PI / 180), 3) * PROXIMITY_DIFFERENCE_THRESHOLD; // Set Treshold based on angle
        state = DISPENSING;
        runPos = MOTOR_OFFSET_RADIANS;
        runSpeed = startSpeed;
        if (usbConnected) {
          Serial.println("Button pressed!");
        }
      }

      else if(buttonMode == CALIBRATE_MODE) {
        state = CALIBRATING;
        
      }

      buttonMode = BUTTON_MODE_RESET;
      break;



    case CALIBRATING:
     
        // if (usbConnected) {
        //   Serial.println("Calibrating proximity baseline...");
        // }
        // for(int i = 0; i < PROXIMITY_CALIBRATION_SAMPLES; i++) // Average over five readings taken one second apart.
        // {
        //   newBaseline += proximitySensor.getProximity();    // Read baseline proximity value.
        //   delay(0.25*SECOND_TO_MILLISECONDS); // Wait for one second.
        // }

        // newBaseline /= PROXIMITY_CALIBRATION_SAMPLES;

        // // baseline = newBaseline;
        // if (usbConnected) {
        //   Serial.println("New baseline proximity value is " + String(newBaseline) + ".");
        // }

        // for (int i = 0; i < NUM_MEDS; i++) {
        //   // DEBUG
        //   Serial.print(newBaseline);
        //   Serial.print(", ");
        //   Serial.print(medCal[i][DROPS_BASELINE_INDEX]);
        //   Serial.print(", ");
        //   Serial.println(abs(newBaseline - medCal[i][DROPS_BASELINE_INDEX]));


          

        //   if (abs((baseline+medCal[i][DROPS_BASELINE_INDEX])-newBaseline) < EPS_MED_SELECT) {
        //     medSelect = i;
        //     break;
        //   }
        //   else {
        //     medSelect = -1;
        //   }
        // }


        medSelect++;

        if (medSelect >= NUM_MEDS) {
          medSelect = 0;
        }

        if (medSelect != -1) {
          Serial.print("Selected Medication: ");
          Serial.println(medName[medSelect]);
        }
        else {
          Serial.println("No Matching Medication Found!");
        }

        medSelectLED();

        newBaseline = 0;


      state = IDLE;

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

      // if ((tocDrop-ticDrop) >= medCal[medSelect][MAX_SUSPENSION_TIME_INDEX]) {
      //   state = IDLE;
      //   if (usbConnected) {
      //     Serial.println("Warning: Max drop suspension time exceed!"); 
      //   }
      // }

      if (difference < medCal[medSelect][DROPS_THRESHOLD_INDEX]) { 
      // if (avgDiff < medCal[medSelect][DROPS_THRESHOLD_INDEX]) { 
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
    if (runPos < endPos) {
      ticOverRun = millis();
    }

    if (runPos >= endPos) 
    { 

      if (usbConnected) {
        Serial.print("Warning: Max extension reached! Tilt angle: "); 
        Serial.print(90-degreesX);
        Serial.println(" degrees"); 
      }
      if((millis() - ticOverRun) >= medCal[medSelect][MAX_SUSPENSION_TIME_INDEX]) {
        state = IDLE;
        Serial.println("Warning: Max extension overrun time exceed!"); 
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

void noMedLED() {
  if (medSelect == -1) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}

void medSelectLED() {

  if (medSelect != -1) {
    for (int i = 0; i <= medSelect; i++) {
      digitalWrite(ledPin, HIGH);
      delay(150);
      digitalWrite(ledPin, LOW);
      delay(150);
    }
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
