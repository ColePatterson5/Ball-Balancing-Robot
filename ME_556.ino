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

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

///////////////
// ELRS Init //
///////////////

// Set up a new Serial object
#define crsfSerial Serial5
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
float yaw;
float YAW;
float Z_set;
float Y_set;
float max_z;
float max_y;
float max_yaw_speed;
float M1_z;
float M2_z;
float M3_z;
float M1_y;
float M3_y;
float caliby;
float calibz;
int Motor1_Sum;
int Motor2_Sum;
int Motor3_Sum;
bool Motor1_Dir;
bool Motor2_Dir;
bool Motor3_Dir;

///////////////
// Setup PID //
///////////////
double Pk = 500;  
double Ik = 3500;
double Dk = 15;

double Setpointy, Inputy, Outputy;    // PID variables
PID PIDy(&Inputy, &Outputy, &Setpointy, Pk, Ik , Dk, DIRECT);    // PID Setup

double Setpointz, Inputz, Outputz;    // PID variables
PID PIDz(&Inputz, &Outputz, &Setpointz, Pk, Ik, Dk, DIRECT);    // PID Setup


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


/*************************************************************/
/*    Display the raw calibration offset and radius data     */
/*************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
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

/**************************************************************************/
/*Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)   */
/**************************************************************************/
void displaySensorDetails(void)
{
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

/*****************************************************/
/* Display some basic info about the sensor status   */
/*****************************************************/
void displaySensorStatus(void)
{
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

/******************************************/
/* Display sensor calibration status      */
/******************************************/
void displayCalStatus(void)
{
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
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
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
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");
  crsf.begin(crsfSerial);
  delay(1000);


  // Motors
  pinMode(1, OUTPUT); //direction control PIN 10 with direction wire 
  pinMode(0, OUTPUT); //PWM PIN 11  with PWM wire
  pinMode(4, OUTPUT); //direction control PIN 10 with direction wire 
  pinMode(3, OUTPUT); //PWM PIN 11  with PWM wire
  pinMode(7, OUTPUT); //direction control PIN 10 with direction wire 
  pinMode(6, OUTPUT); //PWM PIN 11  with PWM wire

  PIDy.SetMode(AUTOMATIC);              
  PIDy.SetOutputLimits(-1000, 1000);
  PIDy.SetSampleTime(10);

  PIDz.SetMode(AUTOMATIC);              
  PIDz.SetOutputLimits(-1000, 1000);
  PIDz.SetSampleTime(10);

  caliby = 0;
  calibz = 0;
  delay(1000);
}



void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= 5) {  // start timed event
          
    previousMillis = currentMillis;

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
    max_z = 1.5; //Degrees
    max_y = 1.5; //Degrees
    max_yaw_speed = 400; //Sort of dimensionless
    crsf.update();
    z = crsf.getChannel(1);
    y = crsf.getChannel(2);
    yaw = crsf.getChannel(4);
    arm = crsf.getChannel(5); //on/off motor switch
    Z_set = -1 * (((float)(z - 988) * (max_z - (-max_z)) / (2011 - 988)) - max_z);
    Y_set = -1 * (((float)(y - 988) * (max_y - (-max_y)) / (2011 - 988)) - max_y);
    YAW = -1 * (((float)(yaw - 988) * (max_yaw_speed - (-max_yaw_speed)) / (2011 - 988)) - max_yaw_speed);


// do PID calcs
    // Setpoint is 0 with no input
    // Is changed through RC
    Setpointy = Y_set;
    Setpointz = Z_set;

    // Print Setpoints
    Serial.print("Y Set: ");
    Serial.print(Setpointy);
    Serial.print(", Z Set: ");
    Serial.print(Setpointz);
    Serial.print(", Yaw: ");
    Serial.println(YAW);

    // Get Orientation data to feed into PID
    // Taking 80% to slow it down
    Inputy = -euler.y() * 0.8;
    Inputz = -euler.z() * 0.8;

    if (crsf.getChannel(9) == 2011){
      caliby = -Inputy;
      calibz = -Inputz;
    }
    
    Inputy += caliby;
    Inputz += calibz;

    // Deadzone for "close enough" to center
    if (abs(Inputy) < 0.05){
      Inputy = 0;
    }
    if (abs(Inputz) < 0.05){
      Inputz = 0;
    }    

    // Print Inputs to PID
    Serial.print("PID Input Y: ");
    Serial.print(Inputy);
    Serial.print(", PID Input Z: ");
    Serial.println(Inputz);

    PIDy.Compute(); //This gives output variable Output1
    PIDz.Compute();

    Serial.print("PID Y: ");
    Serial.print(Outputy);
    Serial.print(", PID Z: ");
    Serial.println(Outputz);

    // Wheel Math
    M1_z = -(Outputz * 0.5);
    M3_z = -(Outputz * 0.5);
    M2_z = Outputz;

    M1_y = (Outputy * sin(3.141592653/3)); 
    M3_y = -(Outputy * sin(3.141592653/3));
        
    Motor1_Sum = (M1_z + M1_y + YAW);
    Motor2_Sum = (M2_z + YAW);
    Motor3_Sum = (M3_z + M3_y + YAW); 
    
    Motor1_Dir = sign(Motor1_Sum);
    Motor2_Dir = sign(Motor2_Sum);
    Motor3_Dir = sign(Motor3_Sum);

    Motor1_Sum = -0.1 * abs(Motor1_Sum) + 240;
    Motor2_Sum = -0.1 * abs(Motor2_Sum) + 240;
    Motor3_Sum = -0.1 * abs(Motor3_Sum) + 240;

    Serial.print("Motor 1 PWM: ");
    Serial.print(Motor1_Sum);
    Serial.print(", Motor 2 PWM: ");
    Serial.print(Motor2_Sum);
    Serial.print(", Motor 3 PWM: ");
    Serial.print(Motor3_Sum);
    Serial.println(" ");

    if (arm == 1000) {        // drive the motors
      //Serial.print(Motor1_Dir," ", Motor2_Dir , " " , Motor3_Dir , " ")
      //Serial.println(Motor1_Sum , " " , Motor2_Sum , " " , Motor3_Sum , " ")
      digitalWrite(1, Motor1_Dir);
      digitalWrite(7, Motor2_Dir);
      digitalWrite(4, Motor3_Dir);
      analogWrite(0, abs(Motor1_Sum));
      analogWrite(6, abs(Motor2_Sum));
      analogWrite(3, abs(Motor2_Sum));
    }

    else {     // stop the motors when disarmed
      Serial.println("Motors halted");
      analogWrite(0, 255);
      analogWrite(3, 255);
      analogWrite(6, 255);
    }
    if (Inputy > 45 || Inputy < -45 || Inputz > 45 || Inputz < -45 ){
      analogWrite(0, 255);
      analogWrite(3, 255);
      analogWrite(6, 255);
    }
    Serial.println("--");
  }
}





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

