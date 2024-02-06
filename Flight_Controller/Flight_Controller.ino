#include "Servo.h"
#include "FastPID.h"
#include <ReefwingAHRS.h>
#include <MyBoschSensor.h>
#include "ArduinoBLE.h"

#define BLE_UUID_AHRS_SERVICE                    "F000AB30-0451-4000-B000-000000000000"
#define BLE_UUID_AHRS_DATA                       "F000AB31-0451-4000-B000-000000000000"
#define BLE_UUID_AHRS_DATA                       "F000AB32-0451-4000-B000-000000000000"
#define EULER_ANGLES_SIZE 32
#define DRONE_DEFAULT_DEVICE_NAME   "DroneTest"
#define PID_HZ 25

typedef union
{
  EulerAngles values;
  uint8_t bytes[ EULER_ANGLES_SIZE ];
} EulerAngles_ut;

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
FastPID myPID;
EulerAngles_ut EulerAnglesData;
EulerAngles_ut EulerAnglesSetPointData;


//BLE variable declaration
BLEService ahrsService( BLE_UUID_AHRS_SERVICE );
BLECharacteristic ahrsDataCharacteristic( BLE_UUID_AHRS_DATA, BLERead | BLENotify, sizeof EulerAnglesData.bytes );
BLECharacteristic ahrsSetPointDataCharacteristic( BLE_UUID_AHRS_DATA, BLERead | BLEWrite, sizeof EulerAnglesSetPointData.bytes );
#pragma endregion
#pragma region variable Declarations
//PID variable declaration
float Kp=1, Ki=0.5, Kd=0.25;
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

uint8_t value[32];
void* ptr;
#pragma endregion

void setup() {
    //  Start Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);

    InitialiseAHRS();

    myPID.configure(Kp, Ki, Kd, PID_HZ, output_bits, output_signed);
    myPID.setOutputRange(-1000,1000);
    

    //set up the servo/leftProp
    //send the min signal to Initialise leftProp before connecting battery
    leftProp.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    rigthProp.attach(8,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    leftProp.writeMicroseconds(1000);
    rigthProp.writeMicroseconds(1000);
    //delay(10000);

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

  //update predictive filter
  ahrs.setData(data);
  ahrs.update();

  loopPeriod = millis() - previousMillisPID;

  if(loopPeriod >= (1000/PID_HZ)){
    output = myPID.step(setPoint, ahrs.angles.roll);
    outputLeftProp = constrain(throttle+output,1000,2000); 
    outputRightProp = constrain(throttle-output,1000,2000); 
    
    // Serial.print("Roll: ");
    // Serial.print(ahrs.angles.roll);
    // Serial.print("\tOutput: ");
    // Serial.print(output);
    // Serial.print("\tLoop Frequency: ");
    // Serial.print(loopPeriod);
    // Serial.print(" ms");
    // Serial.print("\toutputRightProp= ");
    // Serial.print(outputRightProp);
    // Serial.print("\toutputleftProp= ");
    // Serial.print(outputLeftProp);
    // Serial.print("\tMapped= ");
    // Serial.println(constrain(output,1000,2000));
    loopFrequency = 0;
    previousMillisPID = millis();

  }

  //TODO: decide on a AHRS loop Frequency and also for BLE
  if (millis() - previousMillis >= 1) {
  
    BLE.poll();
    EulerAnglesData.values = ahrs.angles;

    BLEDevice central = BLE.central();
  

    if (central)
    {
        // Serial.print( "Connected to central: " );
        // Serial.println( central.address() );
        ahrsDataCharacteristic.writeValue( EulerAnglesData.bytes, sizeof EulerAnglesData.bytes );
    }

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
    // BLE add characteristics
    ahrsService.addCharacteristic(ahrsDataCharacteristic);
    ahrsService.addCharacteristic(ahrsSetPointDataCharacteristic);
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

void CallBackSetPointWritten(BLEDevice central, BLECharacteristic characteristic){
  int i = 0;
  Serial.print("Characteristic event, written: ");
  if (characteristic.value())
  {
    
    characteristic.readValue(EulerAnglesSetPointData.bytes,sizeof(EulerAnglesSetPointData.bytes));
    Serial.print(EulerAnglesSetPointData.values.roll);
  }
  Serial.println("\tEnd of Function");
}