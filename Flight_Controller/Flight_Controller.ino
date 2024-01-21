#include "Servo.h"
#include "FastPID.h"
#include <ReefwingAHRS.h>
#include <MyBoschSensor.h>
#include "ArduinoBLE.h"

#define BLE_UUID_AHRS_SERVICE                    "F000AB30-0451-4000-B000-000000000000"
#define BLE_UUID_AHRS_DATA                       "F000AB31-0451-4000-B000-000000000000"

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
Servo ESC;

//BLE variable declaration

#define DRONE_DEFAULT_DEVICE_NAME   "DroneTest"

EulerAngles_ut EulerAnglesData;

BLEService ahrsService( BLE_UUID_AHRS_SERVICE );
BLECharacteristic ahrsDataCharacteristic( BLE_UUID_AHRS_DATA, BLERead | BLENotify, sizeof EulerAnglesData.bytes );

//PID variable declaration
float Kp=1, Ki=0.5, Kd=0.25, Hz=50;
int output_bits = 8;
bool output_signed = false;
uint8_t output;
double setPoint = 25;

//PID obj Initialise
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

//ESC control variable
uint8_t outputESC;
uint8_t outputESCTest;

//  Display and Loop Frequency
int loopFrequency = 0;
const long displayPeriod = 5000;
unsigned long previousMillis = 0;
unsigned long previousMillisPID = 0;
unsigned long periodOfPID = 0;

void setup() {
    //  Start Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);

    InitialiseAHRS();

    //set up the servo/esc
    ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds)

    InitialiseIMU();

    if ( InitialiseBLE() )
    {
        digitalWrite( LED_BUILTIN, HIGH );
        delay(1000);
        digitalWrite( LED_BUILTIN, LOW );
    }
}

void loop() {
  if (myIMU.gyroscopeAvailable()) {  myIMU.readGyroscope(data.gx, data.gy, data.gz);  }
  if (myIMU.accelerationAvailable()) {  myIMU.readAcceleration(data.ax, data.ay, data.az);  }
  if (myIMU.magneticFieldAvailable()) {  myIMU.readMagneticField(data.mx, data.my, data.mz);  }

  ahrs.setData(data);
  ahrs.update();

  periodOfPID = millis() - previousMillisPID;

  if(periodOfPID >= 20){
    output = myPID.step(setPoint, ahrs.angles.roll);
    // Serial.print("\tLoop Frequency: ");
    // Serial.print(periodOfPID);
    // Serial.println(" ms");

    loopFrequency = 0;
    previousMillisPID = millis();

  }

  if (millis() - previousMillis >= 1) {
    //  Display sensor data every displayPeriod, non-blocking.

    outputESC =  map(output, 0, 255, 0, 180); 
    outputESCTest = map(ahrs.angles.roll,-90,90,0,180);
    ESC.write(outputESC);
    EulerAnglesData.values = ahrs.angles;

    BLEDevice central = BLE.central();

    if (central)
    {
        Serial.print( "Connected to central: " );
        Serial.println( central.address() );
        ahrsDataCharacteristic.writeValue( EulerAnglesData.bytes, sizeof EulerAnglesData.bytes );
    }
    
    Serial.print("Roll= ");
    Serial.println(EulerAnglesData.values.roll,HEX);
    // Serial.print(" SetPoint= ");
    // Serial.print(ahrs.angles.pitch);
    // Serial.print(" outputPID= ");
    // Serial.print(output);
    // Serial.print(" outputESCTest= ");
    // Serial.println(outputESCTest);

    // Serial.print("\tPitch= ");
    // Serial.print(ahrs.angles.pitch, 2);
    // Serial.print("\tYaw= ");
    // Serial.print(ahrs.angles.yaw, 2);
    // Serial.print("\tHeading= ");
    // Serial.println(ahrs.angles.heading, 2);
    //Serial.print("\tLoop Frequency: ");
    //Serial.print(loopFrequency);
    //Serial.println(" Hz");

    // loopFrequency = 0;
    // previousMillis = millis();
  }

  loopFrequency++;
}

bool InitialiseBLE()
{
    if ( !BLE.begin() )
    {
        return false;
    }
    // set advertised local name and service UUID:
    BLE.setDeviceName( DRONE_DEFAULT_DEVICE_NAME );
    BLE.setLocalName( DRONE_DEFAULT_DEVICE_NAME );
    BLE.setAdvertisedService( ahrsService );

    // BLE add characteristics
    ahrsService.addCharacteristic( ahrsDataCharacteristic );
    BLE.addService( ahrsService );
    ahrsDataCharacteristic.writeValue( EulerAnglesData.bytes, sizeof EulerAnglesData.bytes );

      // start advertising
    BLE.advertise();

    return true;
}

void InitialiseAHRS()
{
    //  Use default fusion algo and parameters
    ahrs.begin();
    
    ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
    ahrs.setDeclination(-1.79);                      //  dundalk declination
    ahrs.setBeta(0.8);                               // set beta value **tweek**
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