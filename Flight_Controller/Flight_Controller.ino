#include "Servo.h"
#include "FastPID.h"
#include <ReefwingAHRS.h>
#include <MyBoschSensor.h>
#include "ArduinoBLE.h"
#include <DroneCustomDataTypes.h>
#include "Common.h"

#define BLE_UUID_AHRS_SERVICE                    "F000AB30-0451-4000-B000-000000000000"
#define BLE_UUID_AHRS_DATA                       "F000AB31-0451-4000-B000-000000000000"
#define BLE_UUID_AHRS_SETPOINT_DATA              "F000AB32-0451-4000-B000-000000000000"
#define BLE_UUID_DRONE_CONTROL_CONFIG_DATA       "F000AB33-0451-4000-B000-000000000000"
#define EULER_ANGLES_SIZE 32
#define ATTITUDE_SET_POINT_SIZE 12
#define DRONE_CONTROL_CONFIG_SIZE 16
#define DRONE_DEFAULT_DEVICE_NAME   "DroneTest"
#define PID_HZ 25

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
FastPID rollPID;
EulerAngles_ut EulerAnglesData;
AttitudeSetPointData_ut AttitudeSetPointData;
DroneControllConfig_ut DronePIDData;


//BLE variable declaration
BLEService ahrsService( BLE_UUID_AHRS_SERVICE );
BLECharacteristic ahrsDataCharacteristic( BLE_UUID_AHRS_DATA, BLERead | BLENotify, sizeof EulerAnglesData.bytes );
BLECharacteristic ahrsSetPointDataCharacteristic( BLE_UUID_AHRS_SETPOINT_DATA, BLERead | BLEWriteWithoutResponse, sizeof AttitudeSetPointData.bytes );
BLECharacteristic droneControlConfigCharacteristic( BLE_UUID_DRONE_CONTROL_CONFIG_DATA, BLERead | BLEWriteWithoutResponse, sizeof DronePIDData.bytes );
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

//  Display and Loop Frequency
int loopFrequency = 0;
const long displayPeriod = 5000;
unsigned long previousMillis = 0;
unsigned long previousMillisPID = 0;
unsigned long loopPeriod = 0;
unsigned long filterLoopPeriod = 0;

uint8_t value[32];
void* ptr;
#pragma endregion

void setup() {
    //  Start Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);

    InitialiseAHRS();
    //start pid with default values
    rollPID.configure(10,0.1,0.01, PID_HZ, output_bits, output_signed);
    rollPID.setOutputRange(-1000,1000);
    

    //set up the servo/leftProp
    //send the min signal to Initialise leftProp before connecting battery
    leftProp.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    rigthProp.attach(8,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    leftProp.writeMicroseconds(1000);
    rigthProp.writeMicroseconds(1000);
    delay(1000);
    Serial.println("delay started Turn on power supply");
    //delay(10000);
    Serial.println("delay done");
    InitialiseIMU();

    if ( InitialiseBLE() )
    {
        digitalWrite( LED_BUILTIN, HIGH );
        delay(1000);
        digitalWrite( LED_BUILTIN, LOW );
    }else
    {
        digitalWrite( LED_BUILTIN, HIGH );
        delay(1000);
        digitalWrite( LED_BUILTIN, LOW );
        delay(1000);
        digitalWrite( LED_BUILTIN, HIGH );
        delay(1000);
        digitalWrite( LED_BUILTIN, LOW );
        delay(1000);
        digitalWrite( LED_BUILTIN, HIGH );
    }
}

void loop() {
  if (myIMU.gyroscopeAvailable()) {  myIMU.readGyroscope(data.gx, data.gy, data.gz);  }
  if (myIMU.accelerationAvailable()) {  myIMU.readAcceleration(data.ax, data.ay, data.az);  }
  if (myIMU.magneticFieldAvailable()) {  myIMU.readMagneticField(data.mx, data.my, data.mz);  }

  // //update predictive filter
  ahrs.setData(data,true);
  ahrs.update();

  loopPeriod = millis() - previousMillisPID;

  if(loopPeriod >= (1000/PID_HZ)){
    output = rollPID.step(AttitudeSetPointData.values.roll, ahrs.angles.roll);
    outputLeftProp = constrain(throttle+output,1000,2000); 
    outputRightProp = constrain(throttle-output,1000,2000); 

    leftProp.writeMicroseconds(outputLeftProp);
    rigthProp.writeMicroseconds(outputRightProp);

    //PrintDroneDataToSerial();
    previousMillisPID = millis();
  }

  BLE.poll();
  EulerAnglesData.values = ahrs.angles;

  BLEDevice central = BLE.central();


  if (central)
  {
      ahrsDataCharacteristic.writeValue( EulerAnglesData.bytes, sizeof EulerAnglesData.bytes );
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
    
  ahrs.setFusionAlgorithm(SensorFusion::MAHONY);
  ahrs.setDeclination(-1.79);                      //  dundalk declination
  ahrs.setBeta(0.8);                               // set beta value **tweek**
  ahrs.setKp(10);
  ahrs.setKi(0);
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

}

void PrintDroneDataToSerial()
{
  Serial.print("Roll: ");
  Serial.print(ahrs.angles.roll);
  Serial.print("\tOutput: ");
  Serial.print(output);
  Serial.print("\t\tRoll Set point");
  Serial.print(AttitudeSetPointData.values.roll);
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
    Serial.print("Writing PID coefficient ERR? ");
    characteristic.readValue(DronePIDData.bytes,sizeof(DronePIDData.bytes));
    rollPID.configure(DronePIDData.values.MKp,DronePIDData.values.MKi,DronePIDData.values.MKd, PID_HZ, output_bits, output_signed);
    Serial.println(rollPID.err());
  }
  digitalWrite( LED_BUILTIN, LOW );
}