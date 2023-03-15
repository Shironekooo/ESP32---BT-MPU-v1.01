/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-ble-server-client/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Variables
float RawAccelerometerValueX, RawAccelerometerValueY, RawAccelerometerValueZ;

bool calibrateState = false;
bool deviceConnected = false;

// BLE server name
#define bleServerName "MPU6050_ESP32"
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Accelerometer Characteristic and Descriptor
BLECharacteristic mpuAccelerometerXCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor mpuAccelerometerXDescriptor(BLEUUID((uint16_t)0x2902));

BLECharacteristic mpuAccelerometerYCharacteristics("cba1d467-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor mpuAccelerometerYDescriptor(BLEUUID((uint16_t)0x2902));

BLECharacteristic mpuAccelerometerZCharacteristics("cba1d468-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor mpuAccelerometerZDescriptor(BLEUUID((uint16_t)0x2902));

// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

void initMPU()
{
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void initBLE()
{
  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *mpuService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
  // X - Axis
  mpuService->addCharacteristic(&mpuAccelerometerXCharacteristics);
  mpuAccelerometerXDescriptor.setValue("Accelerometer X axis ");
  mpuAccelerometerXCharacteristics.addDescriptor(new BLE2902());

  // Y - Axis
  mpuService->addCharacteristic(&mpuAccelerometerYCharacteristics);
  mpuAccelerometerYDescriptor.setValue("Accelerometer X axis ");
  mpuAccelerometerYCharacteristics.addDescriptor(new BLE2902());

  // Z - Axis
  mpuService->addCharacteristic(&mpuAccelerometerZCharacteristics);
  mpuAccelerometerZDescriptor.setValue("Accelerometer X axis ");
  mpuAccelerometerZCharacteristics.addDescriptor(new BLE2902());

  // Start the service
  mpuService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void setup()
{
  Serial.begin(115200);
  initMPU();
  initBLE();
}

void DEBUG_rawAccelData()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  RawAccelerometerValueX = a.acceleration.x;
  RawAccelerometerValueY = a.acceleration.y;
  RawAccelerometerValueZ = a.acceleration.z;

  // X ACCELEROMETER AXIS
  static char AXTemp[6];
  dtostrf(RawAccelerometerValueX, 6, 2, AXTemp);
  mpuAccelerometerXCharacteristics.setValue(AXTemp);
  mpuAccelerometerXCharacteristics.notify();
  Serial.print(RawAccelerometerValueX);
  Serial.print("  ");

  // Y ACCELEROMETER AXIS
  static char AYTemp[6];
  dtostrf(RawAccelerometerValueY, 6, 2, AYTemp);
  mpuAccelerometerYCharacteristics.setValue(AYTemp);
  mpuAccelerometerYCharacteristics.notify();
  Serial.print(RawAccelerometerValueY);
  Serial.print("  ");

  // Z ACCELEROMETER AXIS
  static char AZTemp[6];
  dtostrf(RawAccelerometerValueZ, 6, 2, AZTemp);
  mpuAccelerometerZCharacteristics.setValue(AZTemp);
  mpuAccelerometerZCharacteristics.notify();
  Serial.print(RawAccelerometerValueZ);
  Serial.println();
}

void MainActivity()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (deviceConnected)
  {
    // Read Accelerometer
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    RawAccelerometerValueX = a.acceleration.x;
    RawAccelerometerValueY = a.acceleration.y;
    RawAccelerometerValueZ = a.acceleration.z;

    // X ACCELEROMETER AXIS
    static char AXTemp[6];
    dtostrf(RawAccelerometerValueX, 6, 2, AXTemp);
    mpuAccelerometerXCharacteristics.setValue(AXTemp);
    mpuAccelerometerXCharacteristics.notify();
    Serial.print("RawAccelerometerValueX: ");
    Serial.print(RawAccelerometerValueX);
    Serial.println("");

    // Y ACCELEROMETER AXIS
    static char AYTemp[6];
    dtostrf(RawAccelerometerValueY, 6, 2, AYTemp);
    mpuAccelerometerYCharacteristics.setValue(AYTemp);
    mpuAccelerometerYCharacteristics.notify();
    Serial.print("RawAccelerometerValueY: ");
    Serial.print(RawAccelerometerValueY);
    Serial.println("");

    // Z ACCELEROMETER AXIS
    static char AZTemp[6];
    dtostrf(RawAccelerometerValueZ, 6, 2, AZTemp);
    mpuAccelerometerZCharacteristics.setValue(AZTemp);
    mpuAccelerometerZCharacteristics.notify();
    Serial.print("RawAccelerometerValueZ: ");
    Serial.print(RawAccelerometerValueZ);
    Serial.println("");
  }
}

/*

void DeviceCalibrate(){
// chane this to the function where it reads inputs from android application
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    accelCalibratedX = (a.acceleration.x);
    accelCalibratedY = (a.acceleration.y);
    accelCalibratedZ = (a.acceleration.z);

    calibrateState = true;
  }
}
  */

/*
void PostureCheck(){
if (calibrateState == true) {
    if (a.acceleration.x >= (accelCalibratedX + accelSlouchDifX)) {
      Serial.print("Bad Posture Detected!");
      //insert kato counter nga variable
      //insert vibrate function here
    }

    // add more for other axis dimensions
    else
      Serial.print("Normal Posture.");

  } else
    Serial.print("Device not Calibrated");
}
}
*/

void loop()
{
  // DeviceCheckIfCalibrated

  // Main Activity

  // Debug
  DEBUG_rawAccelData();
  delay(75);
}