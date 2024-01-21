#include "Servo.h"
#include "FastPID.h"
#include <ReefwingAHRS.h>
#include <Arduino_BMI270_BMM150.h>


class MyBoschSensor: public BoschSensorClass {

  public:
    MyBoschSensor(TwoWire& wire = Wire) : BoschSensorClass(wire) {};

  protected:
    virtual int8_t configure_sensor(struct bmi2_dev *dev)
    {
      int8_t rslt;
      uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };

      struct bmi2_int_pin_config int_pin_cfg;
      int_pin_cfg.pin_type = BMI2_INT1;
      int_pin_cfg.int_latch = BMI2_INT_NON_LATCH;
      int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
      int_pin_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
      int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
      int_pin_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

      struct bmi2_sens_config sens_cfg[2];
      sens_cfg[0].type = BMI2_ACCEL;
      sens_cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
      sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_800HZ;
      sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
      sens_cfg[1].type = BMI2_GYRO;
      sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
      sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
      sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
      sens_cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

      rslt = bmi2_set_int_pin_config(&int_pin_cfg, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_set_sensor_config(sens_cfg, 2, dev);
      if (rslt != BMI2_OK)
        return rslt;

      rslt = bmi2_sensor_enable(sens_list, 2, dev);
      if (rslt != BMI2_OK)
        return rslt;

      return rslt;
    }
};

MyBoschSensor myIMU(Wire1);

ReefwingAHRS ahrs;
SensorData data;
Servo ESC;

float Kp=1, Ki=0.5, Kd=0.25, Hz=50;
int output_bits = 8;
bool output_signed = false;
uint8_t output;

uint8_t outputESC;
uint8_t outputESCTest;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
float setPoints[] = {10.5,30.5,50.0};
int setPoint = 25.5;
int j=0;

//  Display and Loop Frequency
int loopFrequency = 0;
const long displayPeriod = 5000;
unsigned long previousMillis = 0;
unsigned long previousMillisPID = 0;

void setup() {

  //  Initialise the AHRS
  //  Use default fusion algo and parameters
  ahrs.begin();
  
  ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
  ahrs.setDeclination(-1.79);                      //  Sydney, Australia
  ahrs.setBeta(0.8);

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  myIMU.debug(Serial);
  //myIMU.onInterrupt(print_data);

  //set up the servo/esc
  ESC.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 

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

void loop() {
  if (myIMU.gyroscopeAvailable()) {  myIMU.readGyroscope(data.gx, data.gy, data.gz);  }
  if (myIMU.accelerationAvailable()) {  myIMU.readAcceleration(data.ax, data.ay, data.az);  }
  if (myIMU.magneticFieldAvailable()) {  myIMU.readMagneticField(data.mx, data.my, data.mz);  }
  
  if(millis() - previousMillis >= displayPeriod){
     setPoint = 25.5;
      j = (j+1)%3;
      setPoint = setPoints[j];
  }

  ahrs.setData(data);
  ahrs.update();

  if(millis() - previousMillisPID >= 20){
    output = myPID.step(setPoint, ahrs.angles.roll);
  }

  if (millis() - previousMillis >= 20) {
    //  Display sensor data every displayPeriod, non-blocking.

    outputESC =  map(output, 0, 255, 0, 180); 
    outputESCTest = map(ahrs.angles.roll,-90,90,0,180);
    ESC.write(outputESCTest);

    Serial.print("Roll: ");
    Serial.print(ahrs.angles.roll, 2);
    Serial.print(" SetPoint= ");
    Serial.print(ahrs.angles.pitch);
    Serial.print(" outputPID= ");
    Serial.print(output);
    Serial.print(" outputESCTest= ");
    Serial.println(outputESCTest);

    // Serial.print("\tPitch= ");
    // Serial.print(ahrs.angles.pitch, 2);
    // Serial.print("\tYaw= ");
    // Serial.print(ahrs.angles.yaw, 2);
    // Serial.print("\tHeading= ");
    // Serial.println(ahrs.angles.heading, 2);
    //Serial.print("\tLoop Frequency: ");
    //Serial.print(loopFrequency);
    //Serial.println(" Hz");

    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}