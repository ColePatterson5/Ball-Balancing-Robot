#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <PID_v1.h>
#include <EEPROM.h>

/////////////////
// BNO055 Init //
/////////////////

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Setup I2C Line
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

///////////////
// ELRS Init //
///////////////

// Set up a new Serial object
#define crsfSerial Serial5 //Serial5
AlfredoCRSF crsf;

////////////////
// Motor Init //
////////////////
int i = 0;
unsigned long previousTime = 0;
bool flag = HIGH;

// Initialize variables

float z;
float y;
float trim;
float yaw;
float YAW;
float Z_set;
float Y_set;
float max_z;
float max_y;
float max_yaw_speed;
float M1_y;
float M3_y;
float calib;
int Motor1_Sum;
int Motor2_Sum;
int Motor3_Sum;
bool Motor1_Dir;
bool Motor2_Dir;
bool Motor3_Dir;
float PID_tune_enable;
float PID_mult;
float PID_select;
float PID_modify;

///////////////
// Setup PID //
///////////////
double Pk = 23;  //around 8
double Ik = 18; //
double Dk = 0.9; //

double Setpoint, Input, Output;    // PID variables
PID PID(&Input, &Output, &Setpoint, Pk, Ik , Dk, DIRECT);    // PID Setup


////////////////////
// More Variables //
////////////////////

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timers
int loopTime;
unsigned long previousSafetyMillis;
int IMUdataReady = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
int arm;

////////////////////
// Setup Function //
////////////////////
void setup() {
  Serial.begin(115200); // Open serial port
  delay(3000);          // give the serial port time to open

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  /* TO CLEAR EEPROM
  for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
  }
  */

  EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }

    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();
    /* Optional: Display current status */
    displaySensorStatus();

    sensors_event_t event;
    bno.getEvent(&event);
    /* always recal the mag as It goes out of calibration very often */  
     if (foundCalib){
        /*Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }*/
    }

    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    delay(500);
    


  // ELRS
  crsfSerial.begin(115200, SERIAL_8N1);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");
  crsf.begin(crsfSerial);
  delay(1000);


  // Motors
  pinMode(1, OUTPUT); //direction control PIN 10 with direction wire 
  pinMode(0, OUTPUT); //PWM PIN 11  with PWM wire

  pinMode(7, OUTPUT); //direction control PIN 10 with direction wire 
  pinMode(6, OUTPUT); //PWM PIN 11  with PWM wire

  PID.SetMode(AUTOMATIC);              
  PID.SetOutputLimits(-1000, 1000);
  PID.SetSampleTime(10);
      
  calib = 0;
  delay(1000);
}



void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= 5) {  // start timed event
          
    previousMillis = currentMillis;
    //printChannels();
    // Get and print the Euler angles and IMU calibration
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);

    // Scale RC Inputs to degrees
    max_y = 2; //Degrees
    max_yaw_speed = 200; //Sort of dimensionless
    crsf.update();
    y = crsf.getChannel(2);
    yaw = crsf.getChannel(1);
    arm = crsf.getChannel(8); //on/off motor switch
    //trim = 0.05*(crsf.getChannel(10)-1500);
    Y_set = -1 * (((float)(y - 988) * (max_y - (-max_y)) / (2011 - 988)) - max_y);
    YAW = -1 * (((float)(yaw - 988) * (max_yaw_speed - (-max_yaw_speed)) / (2011 - 988)) - max_yaw_speed);


// do PID calcs
    // Setpoint is 0 with no input
    // Is changed through RC
    Setpoint = Y_set;

    // Print Setpoints
    Serial.print("Y Set: ");
    Serial.print(Setpoint);
    Serial.println(", Yaw: ");
    //Serial.println(YAW);

    // Get Orientation data to feed into PID
    // Taking 80% to slow it down
    Input = euler.y();

    if (crsf.getChannel(9) > 1811){
      calib = -Input;
    }
    
    Input += calib;


    // Deadzone for "close enough" to center
    if (abs(Input) < 0.05){
      Input = 0;
    }

    // Print Inputs to PID
    Serial.print("PID Input Y: ");
    Serial.println(Input);


    PID.Compute(); //This gives output variable Output1

    Serial.print("PID Y: ");
    Serial.print(Output);


    // Wheel Math

    M1_y = (Output); 
    M3_y = (-Output);
        
    Motor1_Sum = (M1_y + YAW);
    Motor2_Sum = (M3_y + YAW);
    
    Motor1_Dir = sign(Motor1_Sum);
    Motor2_Dir = sign(Motor2_Sum);

    Serial.print(", Motor 1 PWM: ");
    Serial.print(Motor1_Sum);
    Serial.print(", Motor 2 PWM: ");
    Serial.print(Motor2_Sum);
    Serial.print("Motor1_Dir: ");
    Serial.print(Motor1_Dir);

    if (arm == 2000) {        // drive the motors
      digitalWrite(1, Motor1_Dir);
      digitalWrite(7, Motor2_Dir);
      analogWrite(0, abs(Motor1_Sum));
      analogWrite(6, abs(Motor2_Sum));
    }

    else {     // stop the motors when disarmed
      Serial.println("Motors halted");
      analogWrite(0, 0);
      analogWrite(6, 0);
    }
    if (Input > 45 || Input < -45){
      Serial.println("Motors halted");
      analogWrite(0, 0);

      analogWrite(6, 0);
    }
    Serial.println("--");
  }
}

// Loop End

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for cleaner loop code //////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Use crsf.getChannel(x) to get us channel values (1-16).
void printChannels() {
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++)
  {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
  delay(250);
}

// Filter Function
float filter(float lengthOrig, float currentValue, int filter) {
  float lengthFiltered =  (lengthOrig + (currentValue * filter)) / (filter + 1);
  return lengthFiltered;  
}

bool sign(int sum){
  bool sign;
  if (sum > 0){
    return sign = HIGH;
  }
  else{
    return sign = LOW;
  }
}

void displaySensorDetails(void) {
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) {
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void displaySensorStatus(void) {
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

void displayCalStatus(void) {
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}
