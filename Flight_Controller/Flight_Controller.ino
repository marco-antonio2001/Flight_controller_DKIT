#include "Servo.h"
#include "FastPID.h"
#include <ReefwingAHRS.h>
#include <MyBoschSensor.h>
#include "ArduinoBLE.h"
#include <DroneCustomDataTypes.h>
#include "Common.h"


#define BLE_SENSE_UUID(val) ("F000" val "-0451-4000-B000-000000000000")
#define EULER_ANGLES_SIZE 32
#define ATTITUDE_SET_POINT_SIZE 12
#define DRONE_CONTROL_CONFIG_SIZE 16
#define DRONE_DEFAULT_DEVICE_NAME   "DroneTest"
#define PID_HZ 25
#define PITCH_CRIT_ANGLE_POS 20.0
#define PITCH_CRIT_ANGLE_NEG -20.0
#define SAMPLESIZE 1000


typedef union
{
  EulerAngles values;
  uint8_t bytes[ EULER_ANGLES_SIZE ];
} EulerAngles_ut;

typedef union
{
  AttituteSetPoint values;
  uint8_t bytes[ EULER_ANGLES_SIZE ];
} AttitudeSetPointData_ut;

typedef union
{
  DroneControllConfig values;
  uint8_t bytes[ DRONE_CONTROL_CONFIG_SIZE ];
} DroneControllConfig_ut;

#pragma region Function prototypes
//function prototypes
void InitialiseIMU();
void InitialiseAHRS();
bool InitialiseBLE();
#pragma endregion
#pragma region Obj declaration
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
EulerAngles_ut EulerAnglesData;
AttitudeSetPointData_ut AttitudeSetPointData;
DroneControllConfig_ut DronePIDData;


//BLE variable declaration
BLEService ahrsService(BLE_SENSE_UUID("AB30"));
BLECharacteristic ahrsDataCharacteristic( BLE_SENSE_UUID("AB31"), BLERead | BLENotify, sizeof EulerAnglesData.bytes );
BLECharacteristic ahrsSetPointDataCharacteristic( BLE_SENSE_UUID("AB32"), BLERead | BLEWriteWithoutResponse, sizeof AttitudeSetPointData.bytes );
BLECharacteristic droneControlConfigCharacteristic( BLE_SENSE_UUID("AB33"), BLERead | BLEWriteWithoutResponse, sizeof DronePIDData.bytes );
#pragma endregion
#pragma region variable Declarations

//PID variable declaration
int output_bits = 16;
bool output_signed = true;
int output;
double setPoint = 1;

int outputLeftProp;
int outputRightProp;
int outputleftPropTest;
double throttle=1300;//init val of prop 
bool pitchCriticalAngleFlag = false; // true when pitch angle 

//  Display and Loop Frequency
int loopFrequency = 0;
const long displayPeriod = 5000;
unsigned long previousMillis = 0;
unsigned long previousMillisPID = 0;
unsigned long loopPeriod = 0;
unsigned long filterLoopPeriod = 0;

//accelerometer cal value
float cal_ax,cal_ay,cal_az;

//Gyro cal value 
float cal_gx,cal_gy,cal_gz;



//Magnetometer HardIron calibration
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
void* ptr;
#pragma endregion

void setup() {
    //  Start Serial and wait for connection
    Serial.begin(115200);
    //while (!Serial);

    InitialiseAHRS();
    //start pid with default values
    pitchPID.configure(10,0.1,0.01, PID_HZ, output_bits, output_signed);
    pitchPID.setOutputRange(-1000,1000);
    

    //set up the servo/leftProp
    //send the min signal to Initialise leftProp before connecting battery
    leftProp.attach(9,1000,2200); // (pin, min pulse width, max pulse width in microseconds)
    rigthProp.attach(8,1000,2200); // (pin, min pulse width, max pulse width in microseconds)
    leftProp.writeMicroseconds(1000);
    rigthProp.writeMicroseconds(1000);
    delay(7000);
    Serial.println("esc stared");
    InitialiseIMU();
    InitialiseBLE();
}

void loop() {
  if (myIMU.gyroscopeAvailable()) {  myIMU.readGyroscope(data.gx, data.gy, data.gz);  data.gx -= cal_gx; data.gy -= cal_gy; data.gz -= cal_gz;}
  if (myIMU.accelerationAvailable()) {  myIMU.readAcceleration(data.ax, data.ay, data.az);data.ax -= cal_ax; data.ay -= cal_ay; data.az -= cal_az;}
  if (myIMU.magneticFieldAvailable()) {  myIMU.readMagneticField(magData[0], magData[1], magData[2]); calibrateMagRawData(); }
  
  ahrs.setData(data,true);
  ahrs.update();
  loopPeriod = millis() - previousMillisPID;

  if(loopPeriod >= (1000/PID_HZ)){
      // //update predictive filter

    output = pitchPID.step(AttitudeSetPointData.values.pitch, ahrs.angles.pitch);
    outputLeftProp = constrain(throttle+output,1100,2200); 

    outputRightProp = constrain(throttle-output,1100,2200); 

    if (!pitchCriticalAngleFlag)
    { 
      //write to motor when pitchCriticalAngleFlag flag is false
      leftProp.writeMicroseconds(outputLeftProp);
      rigthProp.writeMicroseconds(outputRightProp);
    }else
    {
      //turn ESC motor off when pitchCriticalAngleFlag is true
      leftProp.writeMicroseconds(1000);
      rigthProp.writeMicroseconds(1000);
    }

    //PrintDroneDataToSerial();
    previousMillisPID = millis();
  }

  BLE.poll();
  EulerAnglesData.values = ahrs.angles;

  //write to ble
  ahrsDataCharacteristic.writeValue( EulerAnglesData.bytes, sizeof EulerAnglesData.bytes );


  if (ahrs.angles.pitch > PITCH_CRIT_ANGLE_POS || ahrs.angles.pitch < PITCH_CRIT_ANGLE_NEG)
  {
    //safe gaurd when angle is too big switch motor off
    pitchCriticalAngleFlag = true;
  }else
  {
    pitchCriticalAngleFlag = false;
  }
  

  loopFrequency++;
}


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
    droneControlConfigCharacteristic.setEventHandler(BLEWritten,CallBackControlConfigWritten);
    // BLE add characteristics
    ahrsService.addCharacteristic(ahrsDataCharacteristic);
    ahrsService.addCharacteristic(ahrsSetPointDataCharacteristic);
    ahrsService.addCharacteristic(droneControlConfigCharacteristic);
    BLE.addService(ahrsService);
    ahrsDataCharacteristic.writeValue( EulerAnglesData.bytes, sizeof EulerAnglesData.bytes );

    // start advertising
    BLE.advertise();
    return true;
}

void InitialiseAHRS()
{
  //  Use default fusion algo and parameters
  ahrs.begin();
  ahrs.setImuType(ImuType::BMI270_BMM150);
  
  ahrs.setDOF(DOF::DOF_9);

  ahrs.setFusionAlgorithm(SensorFusion::KALMAN);
  ahrs.setDeclination(-1.79);                      //  dundalk declination
  ahrs.setBeta(0.8);                               // set beta value **tweek**
  ahrs.setKp(10);
  ahrs.setKi(0.0);

  ahrs.kalmanX.setQangle(2.25);//acc output noise ^2
  ahrs.kalmanX.setQbias(0.0049);//gyro output noise ^2

  ahrs.kalmanY.setQangle(2.25);//acc output noise^2
  ahrs.kalmanY.setQbias(0.0049);//gyro output noise ^2

}

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
  // data.mx = magData[0]*cos(ahrs.angles.rollRadians)-(magData[1]*sin(ahrs.angles.pitchRadians)*sin(ahrs.angles.rollRadians))+(magData[2]*cos(ahrs.angles.pitchRadians)*sin(ahrs.angles.rollRadians));
  // data.my = (magData[1]*cos(ahrs.angles.pitchRadians))+(magData[2]*sin(ahrs.angles.pitchRadians));

  data.mx = magData[0]*cos(ahrs.angles.pitchRadians)-(magData[1]*sin(ahrs.angles.rollRadians)*sin(ahrs.angles.pitchRadians))+(magData[2]*cos(ahrs.angles.rollRadians)*sin(ahrs.angles.pitchRadians));
  data.my = (magData[1]*cos(ahrs.angles.rollRadians))+(magData[2]*sin(ahrs.angles.rollRadians));
  data.mx = magData[0];
  data.my = magData[1];
  data.mz = magData[2];
}

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

  digitalWrite( LED_BUILTIN, HIGH );
  //sensor callibraton acc and gyro 
  float accum_ax,accum_ay,accum_az;

  for(int i = 0;i<SAMPLESIZE;i++ )
  {
      //take SAMPLESIZE readings and see what the accel zero off set error is 
      if (myIMU.accelerationAvailable()) {
      myIMU.readAcceleration(data.ax,data.ay,data.az);
      accum_ax += data.ax;
      accum_ay += data.ay;
      accum_az += (data.az -2);
    }
  }
  cal_ax = accum_ax/SAMPLESIZE;
  cal_ay = accum_ay/SAMPLESIZE;
  cal_az = accum_az/SAMPLESIZE;

  float accum_gx,accum_gy,accum_gz;
  for(int i = 0;i<SAMPLESIZE;i++ )
  {
      //take SAMPLESIZE readings and see what the accel zero off set error is 
      if (myIMU.gyroscopeAvailable()) {
      myIMU.readGyroscope(data.gx,data.gy,data.gz);
      accum_gx += data.gx;
      accum_gy += data.gy;
      accum_gz += (data.gz);
    }
  }
  digitalWrite( LED_BUILTIN, LOW );
}


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
  Serial.print(outputRightProp);
  Serial.print("\toutputleftProp= ");
  Serial.println(outputLeftProp);
}

void CallBackSetPointWritten(BLEDevice central, BLECharacteristic characteristic){
  digitalWrite( LED_BUILTIN, HIGH );
  if (characteristic.value())
  {
    characteristic.readValue(AttitudeSetPointData.bytes,sizeof(AttitudeSetPointData.bytes));
  }
  digitalWrite( LED_BUILTIN, LOW );
}

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