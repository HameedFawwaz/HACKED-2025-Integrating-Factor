#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>


// // MPU6050 Configuration --- need this?
#define MPU6050_ADDR            0x68    // MPU6050 device address
 #define MPU6050_ACCEL_XOUT_H    0x3B
 #define MPU6050_PWR_MGMT_1      0x6B

// Processing Configuration
#define WINDOW_SIZE     10      // Size of moving average window
#define SAMPLE_RATE     50     // Sample rate in Hz
#define G               9.81    // Gravity constant

// // I2C pins for ESP32 -- need this?
#define I2C_SDA 01 // 21 for Arduino, 01 for ESP32
#define I2C_SCL 02 // 22 for Arduino, 02 for ESP32



Adafruit_MPU6050 mpu;

// Structure to hold IMU data
typedef struct {
    float acc_x, acc_y, acc_z;    // Acceleration in m/s²
    float gyro_x, gyro_y, gyro_z; // Angular velocity in rad/s
} IMUData;

// Structure to hold position and velocity
typedef struct {
    float x, y, z;         // Position in meters
    float vx, vy, vz;      // Velocity in m/s
} MotionData;

//IMU data init

IMUData data;
MotionData motion;
unsigned long lastSampleTime = 0;
unsigned long currentTime;
unsigned long lastTime;

//UDP Init

const char* ssid = "Fawwaz's Pixel";  // Replace with your WiFi SSID
const char* password = "12345678";  // Replace with your WiFi password

WiFiUDP udp;
const char* remoteIP = "10.152.142.22";  // Replace with your computer's local IP
const int remotePort = 80;  // Port to send data to


// Initialize MPU6050 ---need this??
void initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);  // Power Management 1 register
    Wire.write(0);                    // Wake up MPU6050 (remove sleep mode)
    Wire.endTransmission(true);
}

// Read IMU data ---- dont use this 
// IMUData readIMUData() {
//     IMUData data;
//     uint8_t buffer[14];
    
//     // Request 14 bytes from MPU6050
//     Wire.beginTransmission(MPU6050_ADDR);
//     Wire.write(MPU6050_ACCEL_XOUT_H);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU6050_ADDR, 14, true);
    
//     // Read all 14 bytes
//     for (int i = 0; i < 14; i++) {
//         buffer[i] = Wire.read();
//     }
    
//     // Convert raw data to actual values
//     // MPU6050 default scale: ±2g for accelerometer, ±250°/s for gyroscope
//     const float accel_scale = 2.0 * G / 32768.0;    // For ±2g range
//     const float gyro_scale = 250.0 / 32768.0;       // For ±250°/s range

//     data.acc_x = (int16_t)((buffer[0] << 8) | buffer[1]) * accel_scale;
//     data.acc_y = (int16_t)((buffer[2] << 8) | buffer[3]) * accel_scale;
//     data.acc_z = (int16_t)((buffer[4] << 8) | buffer[5]) * accel_scale;
//     data.gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]) * gyro_scale;
//     data.gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]) * gyro_scale;
//     data.gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]) * gyro_scale;
    
//     return data;
// }

IMUData readIMUData() {
      //IMUData data;
      float ax, ay, az, gx, gy, gz, offset_x, offset_y, offset_z;
      
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;

      gx = g.gyro.x;
      gy = g.gyro.y;
      gz = g.gyro.z;

      offset_x = -0.9;
      offset_y = 0.18;
      offset_z = 1.56;
      

      
      // // Convert to meaningful values
      //  data.acc_x = ax / 16384.0 * G;  // Convert to m/s²
      //  data.acc_y = ay / 16384.0 * G;
      //  data.acc_z = az / 16384.0 * +G;
       data.acc_x = (a.acceleration.x + offset_x);  // Convert to m/s²
       data.acc_y = (a.acceleration.y + offset_y);
       data.acc_z = (a.acceleration.z + offset_z - 9.81);
       data.gyro_x = g.gyro.x / 131.0;   // In degrees/second
       data.gyro_y = g.gyro.y / 131.0;
       data.gyro_z = g.gyro.z / 131.0;

      return data;
  }

// // Process IMU data and update motion data
// void processIMUData(IMUData imu_data, MotionData* motion, 
//                    MovingAverage* filter_x, MovingAverage* filter_y, 
//                    MovingAverage* filter_z) {
//     float dt = 1.0f / SAMPLE_RATE;
    
//     // Apply moving average filter to acceleration data
//     float filtered_acc_x = updateMovingAverage(filter_x, imu_data.acc_x);
//     float filtered_acc_y = updateMovingAverage(filter_y, imu_data.acc_y);
//     float filtered_acc_z = updateMovingAverage(filter_z, imu_data.acc_z - G);

//     // Update velocity using filtered acceleration
//     motion->vx = filtered_acc_x * dt;
//     motion->vy = filtered_acc_y * dt;
//     motion->vz = filtered_acc_z * dt;

//     // Update position using velocity
//     motion->x = motion->vx * dt;
//     motion->y = motion->vy * dt;
//     motion->z = motion->vz * dt;
// }

void processIMUData(IMUData imu_data, MotionData* motion) {
    
    currentTime = millis();  // Get current time in milliseconds
    float dt = (currentTime - lastTime) / 1000.0;  // Convert to seconds
    lastTime = currentTime;


    // Update velocity using acceleration
    motion->vx += imu_data.acc_x * dt;
    motion->vy += imu_data.acc_y * dt;
    motion->vz += imu_data.acc_z * dt;

    // Update position using velocity
    motion->x += motion->vx * dt;
    motion->y += motion->vy * dt;
    motion->z += motion->vz * dt;
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }


    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    // Initialize MPU6050 and check if it was successful
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip!");
        while (1) {
            delay(1000); // Halt execution if MPU6050 is not found
        }
    }

    // Configure MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 initialized successfully");

    lastSampleTime = millis();
    lastTime = millis();
}

void loop() {
    currentTime = millis(); //millis() gives the number of milliseconds elapsed since start of program 
    
    // Check if it's time for a new sample
    if (currentTime - lastSampleTime >= (1000 / SAMPLE_RATE)) {
        // Read IMU data
        IMUData imu_data = readIMUData(); 
        
        
        

        // Process data
        processIMUData(imu_data, &motion);
        
        udp.beginPacket(remoteIP, remotePort);
         udp.print(motion.x);
         udp.print(",");
         udp.print(motion.y);
         udp.print(",");
         udp.print(motion.z);
         udp.print(",");

         udp.print(motion.vx);
         udp.print(",");
         udp.print(motion.vy);
         udp.print(",");
         udp.print(motion.vz);
         udp.print(",");

        udp.print(data.acc_x);
        udp.print(",");
        udp.print(data.acc_y);
        udp.print(",");
        udp.print(data.acc_z);

        udp.endPacket();

        // Update timing
        lastSampleTime = currentTime;
    }
}











