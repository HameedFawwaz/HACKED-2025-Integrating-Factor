#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// // MPU6050 Configuration --- need this?
#define MPU6050_ADDR            0x68    // MPU6050 device address
 #define MPU6050_ACCEL_XOUT_H    0x3B
 #define MPU6050_PWR_MGMT_1      0x6B

// Processing Configuration
//#define WINDOW_SIZE     10      // Size of moving average window
//#define SAMPLE_RATE     50     // Sample rate in Hz
//#define G               9.81    // Gravity constant

// // I2C pins for ESP32 -- need this?
#define I2C_SDA 01 // 21 for Arduino, 01 for ESP32
#define I2C_SCL 02 // 22 for Arduino, 02 for ESP32

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  while (!Serial) {
    delay(10); // Wait for Serial
  }

  // Initialize I2C
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set acc range, gyro range, filter bandwidth
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);
}

void loop() {
  // Get a new sensor event
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  
  // get accel and rot data
  float ax = a.acceleration.x - 0.65;
  float ay = a.acceleration.y + 0.16;
  float az = a.acceleration.z + 1.46;

  float axRounded = round(ax * 10) / 10;
  float ayRounded = round(ay * 10) / 10;
  float azRounded = round(az * 10) / 10;

  // convert to deg/s
  float gx = round(g.gyro.x * 180.0 / 3.14159)+1;
  float gy = round(g.gyro.y * 180.0 / 3.14159)+1;
  float gz = round(g.gyro.z * 180.0 / 3.14159)-5;

  // Print data
  Serial.print(axRounded, 6);
  Serial.print(",");
  Serial.print(ayRounded, 6);
  Serial.print(",");
  Serial.print(azRounded, 6);
  Serial.print(",");
  Serial.print(gx, 6);
  Serial.print(",");
  Serial.print(gy, 6);
  Serial.print(",");
  Serial.println(gz, 6);
  //delay(50); //comment this after prelim data testing for faster response time
  delay(10); 
}
