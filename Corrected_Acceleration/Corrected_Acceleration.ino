#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <HardwareSerial.h>

// // MPU6050 Configuration --- need this?
#define MPU6050_ADDR_1            0x68    // MPU6050 device address
#define MPU6050_ADDR_2            0x69    

// Processing Configuration
#define WINDOW_SIZE     50      // Size of moving average window
#define SAMPLE_RATE     50     // Sample rate in Hz
#define G               9.81    // Gravity constant

// // I2C pins for ESP32 -- need this?
#define I2C_SDA 4 // 21 for Arduino, 01 for ESP32
#define I2C_SCL 5 // 22 for Arduino, 02 for ESP32

const int buzzerPin = 2; 
const int PushButton = 1;


unsigned long buzzStartTime = 0;  //tracks when the buzzing starts
const unsigned long BuzzDuration = 5000;  //5 seconds in milliseconds (can be changed)
bool isBuzzing = false;  //tracks if we're currently in a buzz cycle

uint32_t timer = millis();

Adafruit_MPU6050 mpu;

// Structure to hold IMU data
typedef struct {
    float acc_x, acc_y, acc_z;    // Acceleration in m/s²
    float gyro_x, gyro_y, gyro_z; // Angular velocity in rad/s
    float vx, vy, vz;
    float x, y, z;
    float dt;                     // delta time (to be sent to python script)
} IMUData;

// Structure to hold previous IMU Data
typedef struct {
    float acc_x, acc_y, acc_z;    // Acceleration in m/s²
    float vx, vy, vz;             // Old Velocity Data in m/s
    float x, y, z;
} OldIMUData;

// Structure to hold position and velocity
typedef struct {
    float x, y, z;         // Position in meters
    float vx, vy, vz;      // Velocity in m/s
} MotionData;

//IMU data init
IMUData data;
OldIMUData old_data;
MotionData motion;
unsigned long lastSampleTime = 0;
unsigned long currentTime;
unsigned long lastTime;

// Buffers for moving average filter
float velocityBuffer[WINDOW_SIZE][3] = {0};
float positionBuffer[WINDOW_SIZE][3] = {0};
int bufferIndex = 0;

//UDP Init

const char* ssid = "Fawwaz's Pixel";  // Replace with your WiFi SSID
const char* password = "12345678";  // Replace with your WiFi password

WiFiUDP udp;
const char* remoteIP = "10.152.142.22";  // Replace with your computer's local IP
const int remotePort = 80;  // Port to send data to

// // Initialize MPU6050 ---need this??
// void initMPU6050() {
//     Wire.beginTransmission(MPU6050_ADDR);
//     Wire.write(MPU6050_PWR_MGMT_1);  // Power Management 1 register
//     Wire.write(0);                    // Wake up MPU6050 (remove sleep mode)
//     Wire.endTransmission(true);
// }

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

float computeMovingAverage(float buffer[][3], int index, int axis) {
    float sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += buffer[i][axis];
    }
    return sum / WINDOW_SIZE;
}

IMUData readIMUData() {
      //IMUData data;
      float ax, ay, az, gx, gy, gz, offset_x, offset_y, offset_z;
      
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      ax = round(a.acceleration.x * 10) / 10;
      ay = round(a.acceleration.y * 10) / 10;
      az = round(a.acceleration.z * 10) / 10;

      gx = g.gyro.x;
      gy = g.gyro.y;
      gz = g.gyro.z;

      offset_x = -0.75;
      offset_y = 0.06;
      offset_z = 1.56;
      

      
      // // Convert to meaningful values
      //  data.acc_x = ax / 16384.0 * G;  // Convert to m/s²
      //  data.acc_y = ay / 16384.0 * G;
      //  data.acc_z = az / 16384.0 * +G;
       data.acc_x = round(a.acceleration.x + offset_x);  // Convert to m/s²
       data.acc_y = round(a.acceleration.y + offset_y);
       data.acc_z = round(a.acceleration.z + offset_z - 9.81);
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

const int constraint = 5000;
void processIMUData(IMUData imu_data, MotionData* motion) {
    currentTime = millis();  
    float dt = (currentTime - lastTime);  
    lastTime = currentTime;


    // velocityBuffer[bufferIndex][0] = imu_data.acc_x * dt;
    // velocityBuffer[bufferIndex][1] = imu_data.acc_y * dt;
    // velocityBuffer[bufferIndex][2] = imu_data.acc_z * dt;

    // motion->vx = computeMovingAverage(velocityBuffer, bufferIndex, 0);
    // motion->vy = computeMovingAverage(velocityBuffer, bufferIndex, 1);
    // motion->vz = computeMovingAverage(velocityBuffer, bufferIndex, 2);

    // positionBuffer[bufferIndex][0] = motion->x + motion->vx * dt;
    // positionBuffer[bufferIndex][1] = motion->y + motion->vy * dt;
    // positionBuffer[bufferIndex][2] = motion->z + motion->vz * dt;

    // motion->x = computeMovingAverage(positionBuffer, bufferIndex, 0) / 1000;
    // motion->y = computeMovingAverage(positionBuffer, bufferIndex, 1) / 1000;
    // motion->z = computeMovingAverage(positionBuffer, bufferIndex, 2) / 1000;

    motion->vx += (imu_data.acc_x - old_data.acc_x) * dt;
    motion->vy += (imu_data.acc_y - old_data.acc_y) * dt;
    motion->vz += (imu_data.acc_z - old_data.acc_z) * dt;

    motion->vx = constrain(motion->vx, -constraint, constraint);
    motion->vy = constrain(motion->vy, -constraint, constraint);
    motion->vz = constrain(motion->vz, -constraint, constraint);


    motion->x += (motion->vx - old_data.vx) * dt / 1000;
    motion->y += (motion->vy - old_data.vy) * dt / 1000;
    motion->z += (motion->vz - old_data.vz) * dt / 1000;

    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
    data.dt = dt;
}

class KalmanFilterIMU {
public:
    float dt;
    float x[3]; // State vector [ax, ay, az]
    float P[3][3]; // Error covariance matrix
    float Q[3][3]; // Process noise covariance
    float R[3][3]; // Measurement noise covariance
    float H[3][3]; // Measurement matrix
    
    KalmanFilterIMU(float process_noise, float measurement_noise, float error_covariance) {
        this->dt = data.dt;
        
        for (int i = 0; i < 3; i++) {
            x[i] = 0.0;
            for (int j = 0; j < 3; j++) {
                P[i][j] = (i == j) ? error_covariance : 0.0;
                Q[i][j] = (i == j) ? process_noise : 0.0;
                R[i][j] = (i == j) ? measurement_noise : 0.0;
                H[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }

    void predict() {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                P[i][j] += Q[i][j];
            }
        }
    }
    
    void update(float accel[3]) {
        float S[3][3], K[3][3], y[3];
        
        for (int i = 0; i < 3; i++) {
            y[i] = accel[i] - x[i];
            for (int j = 0; j < 3; j++) {
                S[i][j] = P[i][j] + R[i][j];
            }
        }

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                K[i][j] = P[i][j] / S[i][j];
            }
        }

        for (int i = 0; i < 3; i++) {
            x[i] += K[i][i] * y[i];
            for (int j = 0; j < 3; j++) {
                P[i][j] -= K[i][j] * H[i][j] * P[i][j];
            }
        }
    }

    void getFilteredAcceleration(float output[3]) {
        for (int i = 0; i < 3; i++) {
            output[i] = x[i];
        }
    }
};

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

    pinMode(buzzerPin, OUTPUT); //makes pin5 output
    pinMode(PushButton, INPUT); //makes pin4 input
    pinMode(LED_BUILTIN, OUTPUT);

    // Set initial accel data to be 0 on first boot
    old_data.acc_x = 0;
    old_data.acc_x = 0;
    old_data.acc_x = 0;
}

const float process_noise = 0.05;
const float measurement_noise = 0.05;
const float covariance = 1.2;

int i = 0;

float threshold = 0.75*9.81;

KalmanFilterIMU kf(process_noise, measurement_noise, covariance);

void loop() {
  // Check if it's time for a new sample
  // Read IMU data
  IMUData imu_data = readIMUData(); 
        
  float accel_data[3] = {data.acc_x, data.acc_y, data.acc_z};
  kf.predict();
  kf.update(accel_data);
  float filtered_accel[3];
  kf.getFilteredAcceleration(filtered_accel);
      
  data.acc_x = filtered_accel[0];
  data.acc_y = filtered_accel[1];
  data.acc_z = filtered_accel[2];

  // // Process data
  processIMUData(imu_data, &motion);
  float acc_mag;
  acc_mag = sqrt(data.acc_x*data.acc_x + data.acc_y*data.acc_y + data.acc_z*data.acc_z);

  int ButtonState = digitalRead(PushButton);
  if (ButtonState == true && !isBuzzing){ //this if checks if button is pressed and buzzer is not buzzing
    buzzStartTime = millis();
    isBuzzing = true;
  }
  if (acc_mag >= threshold && !isBuzzing){
    buzzStartTime = millis();
    isBuzzing = true;
  }

  if (isBuzzing) { 
    if ((millis() - buzzStartTime) < BuzzDuration){ //this if statement checks if it's still in the 5second window
      // Create a square wave for buzzer tone
      digitalWrite(buzzerPin, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      delayMicroseconds(250); // For ~500Hz tone
      //delay(100);
      digitalWrite(buzzerPin, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      delayMicroseconds(250);
      //delay(100);
    }
    else {
      digitalWrite(buzzerPin, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      isBuzzing = false;
    }
  }

    // if (i == 2){
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
        udp.print(",");

        udp.print(data.gyro_x);
        udp.print(",");
        udp.print(data.gyro_y);
        udp.print(",");
        udp.print(data.gyro_z);
        udp.print(",");


        udp.print(data.dt);

        udp.endPacket();

        old_data.acc_x = data.acc_x;
        old_data.acc_y = data.acc_y;
        old_data.acc_z = data.acc_z;

        old_data.vx = motion.vx;
        old_data.vy = motion.vy;
        old_data.vz = motion.vz;

        // Update timing
        
    // }
    // else if (i > 10){
    //   i = 0;
    // }
    // i++;

    delay(250);
}
