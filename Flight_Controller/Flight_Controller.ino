#include "Servo.h"
#include "FastPID.h"
#include <ReefwingAHRS.h>
#include <MyBoschSensor.h>
#include "ArduinoBLE.h"
#include <DroneCustomDataTypes.h>
#include "Common.h"

#define ESC_CRITIAL_PITCH_CHECK(actAngle,maxAngle,minAngle) (actAngle < maxAngle && actAngle > minAngle)
#define BLE_SENSE_UUID(val) ("F000" val "-0451-4000-B000-000000000000")
#define EULER_ANGLES_SIZE 32
#define ATTITUDE_SET_POINT_SIZE 12
#define DRONE_CONTROL_CONFIG_SIZE 16
#define DRONE_DEFAULT_DEVICE_NAME   "BluePirateDrone"
#define PID_HZ 50
#define LOOP_PERIOD 20
#define MAX_PITCH 40.0
#define MIN_PITCH -40.0
#define SAMPLESIZE 1000

typedef union
{
  AttituteSetPoint values;
  uint8_t bytes[ ATTITUDE_SET_POINT_SIZE ];
} AttitudeSetPointData_ut;

typedef union
{
  DroneControllConfig values;
  uint8_t bytes[ DRONE_CONTROL_CONFIG_SIZE ];
} DroneControllConfig_ut;

//function prototypes
void InitialiseIMU();
void InitialiseAHRS();
bool InitialiseBLE();

//IMU class obj configuration
MyBoschSensor myIMU(Wire1);
//Sensor fusion and Sensor filter used to get attitude and heading 
ReefwingAHRS ahrs;
//sensor data obj
SensorData data;
//Servo obj -- used to controll ESC
Servo leftProp;
Servo rigthProp;
//PID obj Initialise
FastPID pitchPID;
AttitudeSetPointData_ut ActualAttitudeData;
AttitudeSetPointData_ut AttitudeSetPointData;
DroneControllConfig_ut DronePIDData;

//BLE variable declaration
BLEService ahrsService(BLE_SENSE_UUID("AB30"));
BLECharacteristic ahrsDataCharacteristic( BLE_SENSE_UUID("AB31"), BLERead | BLENotify, sizeof ActualAttitudeData.bytes );
BLECharacteristic ahrsSetPointDataCharacteristic( BLE_SENSE_UUID("AB32"), BLERead | BLEWriteWithoutResponse, sizeof AttitudeSetPointData.bytes );
BLECharacteristic PitchPidConfigCharacteristic( BLE_SENSE_UUID("AB33"), BLERead | BLEWriteWithoutResponse, sizeof DronePIDData.bytes );
BLEByteCharacteristic armEscCharacteristic( BLE_SENSE_UUID("AB34"), BLERead | BLEWriteWithoutResponse);

//PID variable declaration
int output_bits = 16;
bool output_signed = true;
int output;
double setPoint = 1;

int outputLeftProp;
int outputRightProp;
int outputleftPropTest;
double throttle=1300;//init val of prop 
bool withinAllowedAngle = false; // true when pitch angle
bool escArmed = false;

//  Display and Loop Frequency
unsigned long lastTime = 0;
unsigned long mainLoopPeriod = 0;
unsigned long startTime ;
unsigned long waitForFlagTime ;

//accelerometer cal value
float cal_ax,cal_ay,cal_az;

//Gyro cal value 
float cal_gx,cal_gy,cal_gz;
VectorData rawGyroOffset;
VectorData rawAccOffset;



//Magnetometer Hard and soft Iron calibration values Acquired from Motion Sensor Calibration App
float magData[3];  
float gyroData[3];
float accData[3];
float hardIron[3] = {-20.17, -9.72, 21.14};
float softIron[3][3] = 
{
  {1.039, -0.0038, 0.008},
  {-0.0038, 0.984, -0.01},
  {0.008, -0.01, 1.030}
};

uint8_t value[32];

/*Summary
  -Starts serial Connection
  -Initializes modules (BLE,AHRS,IMU)
  -Confihure Attitude PID (float kp, float ki, float kd, float hz, int bits, bool sign)
  -Configure PWM module with correct (Pin, min pulse width, max Pulse width)
*/
void setup() {
    pinMode(LEDB, OUTPUT);
    pinMode(LEDR, OUTPUT);

    pinMode(9, OUTPUT);
    pinMode(8, OUTPUT);

    digitalWrite( LEDB, LOW );
    Serial.begin(115200);

    InitialiseAHRS();
    pitchPID.configure(3,0.7,1.0, PID_HZ, output_bits, output_signed);
    //pitchPID.configure(2.75,0.25,1.25, PID_HZ, output_bits, output_signed);

    pitchPID.setOutputRange(-1000,1000);

    Serial.println("esc stared");
    InitialiseIMU();
    InitialiseBLE();
    digitalWrite( LEDB, LOW );
    digitalWrite( LEDR, LOW );
    digitalWrite( LEDR, HIGH );
    //delay(5000);
    leftProp.attach(9); // (pin, min pulse width, max pulse width in microseconds)
    rigthProp.attach(8); // (pin, min pulse width, max pulse width in microseconds)
    leftProp.writeMicroseconds(1000);
    rigthProp.writeMicroseconds(1000);
    delay(2000);
    // leftProp.writeMicroseconds(2000);
    // rigthProp.writeMicroseconds(2000);
    // delay(7000);
    // leftProp.writeMicroseconds(1000);
    // rigthProp.writeMicroseconds(1000);
    // delay(7000);
    digitalWrite( LEDR, LOW);

}

/*Summary
  -Takes a time stamp at the start of Main loop
  -Reads IMU data performing calibration on raw data
  -update predictive filter with raw data to get angles
  -take a step with the PID controller
  -constrain left and right propellor output to lie within a range
  -Perform a check to see if Motors are armed (not armed if outside allowed range)
  -Write To PWM to control ESC and in turn motor
  -Poll Ble (rebroadcast Service)
  -Write To BLE Actual Device Attitude
  -Wait for main loop period to be the desired period(set in define)
*/
void loop() {
  startTime = millis();

  if(myIMU.magneticFieldAvailable()){myIMU.readMagneticField(magData[0], magData[1], magData[2]); calibrateMagRawData();}
  myIMU.readGyroscope(data.gx, data.gy, data.gz);  data.gx -= cal_gx; data.gy -= cal_gy; data.gz -= cal_gz;
  myIMU.readAcceleration(data.ax, data.ay, data.az);data.ax -= cal_ax; data.ay -= cal_ay; data.az -= cal_az;

  ahrs.setData(data,true);
  ahrs.update();

  output = pitchPID.step(AttitudeSetPointData.values.pitch, ahrs.angles.pitch);
  outputLeftProp = constrain(throttle+output,1100,2000); 
  outputRightProp = constrain((throttle*0.90)-(output),1100,2000);

  withinAllowedAngle = ESC_CRITIAL_PITCH_CHECK(ahrs.angles.pitch,MAX_PITCH,MIN_PITCH);

  leftProp.writeMicroseconds(outputLeftProp * (escArmed ) );
  rigthProp.writeMicroseconds(outputRightProp * (escArmed));

  //PrintDroneDataToSerial();

  BLE.poll();
  ActualAttitudeData.values.roll = ahrs.angles.roll;
  ActualAttitudeData.values.pitch = ahrs.angles.pitch;
  ActualAttitudeData.values.yaw = ahrs.angles.yaw;

  //write to ble
  ahrsDataCharacteristic.writeValue( ActualAttitudeData.bytes, sizeof ActualAttitudeData.bytes );
  
  //wait for loop to take 20 ms
  while((millis()-startTime) < LOOP_PERIOD);
  // lastTime = millis();
  // Serial.println(lastTime-startTime);

}

/*Summary
  This Function Initializes the Ble
  -it Begings the BLe
  -Configures the Device Name
  -Adds the Service to the device
  -Adds The Characteristics to the service
  -Sets the Characteristics callback functions
  -Starts Advertising*/
bool InitialiseBLE()
{
    if ( !BLE.begin() )
    {
        return false;
    }
    //BLE.setBinaryConfirmPairing(true);
    // set advertised local name and service UUID:
    BLE.setDeviceName( DRONE_DEFAULT_DEVICE_NAME );
    BLE.setLocalName( DRONE_DEFAULT_DEVICE_NAME );
    BLE.setAdvertisedService( ahrsService );
    BLE.setPairable(true);

    ahrsSetPointDataCharacteristic.setEventHandler(BLEWritten,CallBackSetPointWritten);
    PitchPidConfigCharacteristic.setEventHandler(BLEWritten,CallBackControlConfigWritten);
    armEscCharacteristic.setEventHandler(BLEWritten,CallBackArmEscWritten);
    // BLE add characteristics
    ahrsService.addCharacteristic(ahrsDataCharacteristic);
    ahrsService.addCharacteristic(ahrsSetPointDataCharacteristic);
    ahrsService.addCharacteristic(PitchPidConfigCharacteristic);
    ahrsService.addCharacteristic(armEscCharacteristic);
    BLE.addService(ahrsService);
    ahrsDataCharacteristic.writeValue( ActualAttitudeData.bytes, sizeof ActualAttitudeData.bytes );

    // start advertising
    BLE.advertise();
    return true;
}

/*Summary
  -This Function Initialises the Attitude Heading Reference System (AHRS)
  -Set the Degree of freedom being used (when set to 9 Magnetometer is used) 
  -Set the Fusion Algorithm to be uses (Kalman Filter)
  -Set Declination to find true north
  -Configure Kalman free parameters (acc output noise^2) and (gyro output noise ^2)*/
void InitialiseAHRS()
{
  ahrs.begin();
  ahrs.setImuType(ImuType::BMI270_BMM150);
  
  ahrs.setDOF(DOF::DOF_9);

  ahrs.setDeclination(-1.79);  //  dundalk declination

  // ahrs.kalmanX.setQangle(2.25);//acc output noise ^2
  // ahrs.kalmanX.setQbias(0.0049);//gyro output noise ^2

  // ahrs.kalmanY.setQangle(2.25);//acc output noise^2
  // ahrs.kalmanY.setQbias(0.0049);//gyro output noise ^2

  ahrs.kalmanX.setQangle(2.3);//acc output noise ^2
  ahrs.kalmanX.setQbias(0.0055);//gyro output noise ^2

  ahrs.kalmanY.setQangle(2.3);//acc output noise^2
  ahrs.kalmanY.setQbias(0.0055);//gyro output noise ^2

}

/*Summary
  -This Function Calibrates the raw magnetomer raw data
  -Perform Soft and hard Iron Calibration *Found using Motion Sensor Calibration App*
*/
void calibrateMagRawData()
{
  //do hard iron and soft iron correction
  float hiCal[3];
  for (uint8_t i = 0; i<3;i++)
  {
    hiCal[i] = magData[i]-hardIron[i];
  }
  
  //soft iron correction
  for (uint8_t i = 0; i<3;i++)
  {
    magData[i] =  (softIron[i][0]*hiCal[0])+
                  (softIron[i][1]*hiCal[1])+
                  (softIron[i][2]*hiCal[2]);
  }
  data.mx = magData[0];
  data.my = magData[1];
  data.mz = magData[2];
}

/*Summary
  -Initialise/Begin IMU Module 
  -set The debug Serial
  -take n reading to find the Zero Off set error (device must be at 0 and motionless while blue led is on)
*/
void InitialiseIMU(){
    myIMU.debug(Serial);
    //myIMU.onInterrupt(print_data);

    Serial.print("Detected Board - ");
    Serial.println(ahrs.getBoardTypeString());
    if (myIMU.begin() && ahrs.getBoardType() == BoardType::NANO33BLE_SENSE_R2) {
    Serial.println("BMI270 & BMM150 IMUs Connected."); 
    Serial.print("Gyroscope sample rate = ");
    Serial.print(myIMU.gyroscopeSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Gyroscope in degrees/second");
    Serial.print("Accelerometer sample rate = ");
    Serial.print(myIMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in G's");
    Serial.print("Magnetic field sample rate = ");
    Serial.print(myIMU.magneticFieldSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Magnetic Field in uT");
  } 
  else {
    Serial.println("BMI270 & BMM150 IMUs Not Detected.");
    while(1);
  }

  //value gotten from callibration process
  rawGyroOffset.x = -0.12;
  rawGyroOffset.y = -0.07;
  rawGyroOffset.z = -0.01;

  rawAccOffset.x = 0;
  rawAccOffset.y = 0;
  rawAccOffset.z = 0;

}

/*Summary 
  -This is a debug function
  -Prints important values to Serial
*/
void PrintDroneDataToSerial()
{
  Serial.print("pitch: ");
  Serial.print(ahrs.angles.pitch);
  Serial.print("\tRoll: ");
  Serial.print(ahrs.angles.roll);
  Serial.print("\theading: ");
  Serial.print(ahrs.angles.heading);
  Serial.print("\tOutput: ");
  Serial.print(output);
  Serial.print("\t\tpitch Set point");
  Serial.print(AttitudeSetPointData.values.pitch);
  Serial.print("\toutputRightProp= ");
  Serial.print(outputRightProp * escArmed);
  Serial.print("\toutputleftProp= ");
  Serial.println(outputLeftProp * escArmed);
}

/*Summary
  -Call back function for when Setpoint characteristic is written to
  -reads data[]* and maps to AttitudeSetPointData structure utilizing the union
*/
void CallBackSetPointWritten(BLEDevice central, BLECharacteristic characteristic){
  digitalWrite( LED_BUILTIN, HIGH );
  if (characteristic.value())
  {
    Serial.print("Writing Set point ");
    characteristic.readValue(AttitudeSetPointData.bytes,sizeof(AttitudeSetPointData.bytes));
    Serial.println(AttitudeSetPointData.values.pitch);
  }
  digitalWrite( LED_BUILTIN, LOW );
}

/*Summary
  -Call back function for when PID characteristic is written to
  -reads (uint8_t data[])* and maps to DronePIDData structure utilizing the union
  -calls PID configure function with new values (Important to flush out old values and set new PID constants)
*/
void CallBackControlConfigWritten(BLEDevice central, BLECharacteristic characteristic){
  digitalWrite( LED_BUILTIN, HIGH );
  if (characteristic.value())
  {
    Serial.print("Writing PID... Config ERR? ");
    characteristic.readValue(DronePIDData.bytes,sizeof(DronePIDData.bytes));
    pitchPID.configure(DronePIDData.values.MKp,DronePIDData.values.MKi,DronePIDData.values.MKd, PID_HZ, output_bits, output_signed);
    Serial.println(pitchPID.err() != 0);
  }
  digitalWrite( LED_BUILTIN, LOW );
}

/*Summary
  -Call back function for when armESC characteristic is written
  -sets escArmed to true when any value thats not 0 is received
*/
void CallBackArmEscWritten(BLEDevice central, BLECharacteristic characteristic){  
  if (*characteristic.value())
  {
    escArmed = true;
    leftProp.writeMicroseconds(1100);
    rigthProp.writeMicroseconds(1100);
    pitchPID.clear();
  }else
  {
    escArmed = false;
  }
}
