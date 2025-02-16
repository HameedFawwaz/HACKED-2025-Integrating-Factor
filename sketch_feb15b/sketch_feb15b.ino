

// void loop() {
//     // 1. Read raw data
//     int16_t ax, ay, az, gx, gy, gz;
//     mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//     //scaling 16bit values to meaningful values
//     float AccX = ax / 16384.0;  // max is ±2g
//     float AccY = ay / 16384.0;
//     float AccZ = az / 16384.0;
//     float GyroX = gx / 131.0;   // max is ±250 deg/s
//     float GyroY = gy / 131.0;
//     float GyroZ = gz / 131.0;

//     // 2. Calculate pitch/roll from Acc
//     float rollAcc  = atan2(AccY, AccZ) * 180 / PI;
//     float pitchAcc = atan2(-AccX, sqrt(AccY*AccY + AccZ*AccZ)) * 180 / PI;

//     // 3. Integrate Gyro to get pitch/roll rates
//     unsigned long currentTime = millis();
//     float dt = (currentTime - lastTime) / 1000.0f;
//     lastTime = currentTime;

//     // Integrate gyro (gyroscope = degrees per second)
//     pitch += GyroX * dt;
//     roll  += GyroY * dt;

//     // 4. Complementary filter
//     float alpha = 0.98f;
//     pitch = alpha * pitch + (1.0f - alpha) * pitchAcc;
//     roll  = alpha * roll  + (1.0f - alpha) * rollAcc;

//     Serial.print("Pitch: "); Serial.print(pitch);
//     Serial.print("  Roll: "); Serial.println(roll);

//     delay(10);
// }



#include <Wire.h>

// // MPU6050 Configuration --- need this?
// #define MPU6050_ADDR            0x68    // MPU6050 device address
// #define MPU6050_ACCEL_XOUT_H    0x3B
// #define MPU6050_PWR_MGMT_1      0x6B

// Processing Configuration
#define WINDOW_SIZE     10      // Size of moving average window
#define SAMPLE_RATE     100     // Sample rate in Hz
#define G               9.81    // Gravity constant

// // I2C pins for ESP32 -- need this?
// #define I2C_SDA 21
// #define I2C_SCL 22

// Structure to hold IMU data
typedef struct {
    float acc_x, acc_y, acc_z;    // Acceleration in m/s²
    float gyro_x, gyro_y, gyro_z; // Angular velocity in rad/s
    float pitch, roll;            // angles
} IMUData;

// Structure to hold position and velocity
typedef struct {
    float x, y, z;         // Position in meters
    float vx, vy, vz;      // Velocity in m/s
} MotionData;

// Circular buffer for moving average
typedef struct {
    float* values;
    int size;
    int index;
    int count;
    float sum;
} MovingAverage;

// Global variables
MovingAverage* filter_x;
MovingAverage* filter_y;
MovingAverage* filter_z;
MovingAverage* filter_pitch; 
MovingAverage* filter_roll;
MotionData motion = {0};
unsigned long lastSampleTime = 0;
unsigned long currentTime;
unsigned long lastTime;
float pitch = 0;
float roll = 0;

// Initialize moving average filter
MovingAverage* createMovingAverage(int window_size) {
    MovingAverage* filter = (MovingAverage*)malloc(sizeof(MovingAverage));
    filter->values = (float*)calloc(window_size, sizeof(float));
    filter->size = window_size;
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0f;
    return filter;
}

// Add value to moving average
float updateMovingAverage(MovingAverage* filter, float new_value) {
    if (filter->count == filter->size) {
        filter->sum -= filter->values[filter->index];
    }
    filter->values[filter->index] = new_value;
    filter->sum += new_value;
    
    if (filter->count < filter->size) filter->count++;
    filter->index = (filter->index + 1) % filter->size;
    
    return filter->sum / filter->count;
}

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
    IMUData data;
    int16_t ax, ay, az, gx, gy, gz;
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert to meaningful values
    data.acc_x = ax / 16384.0 * G;  // Convert to m/s²
    data.acc_y = ay / 16384.0 * G;
    data.acc_z = az / 16384.0 * G;
    data.gyro_x = gx / 131.0;   // In degrees/second
    data.gyro_y = gy / 131.0;
    data.gyro_z = gz / 131.0;
    
    return data;
}

// Process IMU data and update motion data
void processIMUData(IMUData imu_data, MotionData* motion, 
                   MovingAverage* filter_x, MovingAverage* filter_y, 
                   MovingAverage* filter_z, float* pitch, float* roll) {

    currentTime = millis();  // Get current time in milliseconds
    float dt = (currentTime - lastTime) / 1000.0;  // Convert to seconds
    lastTime = currentTime;
    
    //getting pitch and roll values from accelerometer
    float pitchAcc = atan2(imu_data.acc_x, sqrt(imu_data.acc_y * imu_data.acc_y + 
                          imu_data.acc_z * imu_data.acc_z)) * 180/PI;
    float rollAcc = atan2(imu_data.acc_y, sqrt(imu_data.acc_x * imu_data.acc_x + 
                         imu_data.acc_z * imu_data.acc_z)) * 180/PI;

    // Complementary filter for pitch and roll
    *pitch = 0.96 * (*pitch + imu_data.gyro_x * dt) + 0.04 * pitchAcc;
    *roll = 0.96 * (*roll + imu_data.gyro_y * dt) + 0.04 * rollAcc;

    // Apply moving average filter to acceleration data
    float filtered_acc_x = updateMovingAverage(filter_x, imu_data.acc_x);
    float filtered_acc_y = updateMovingAverage(filter_y, imu_data.acc_y);
    float filtered_acc_z = updateMovingAverage(filter_z, imu_data.acc_z - G);

    // Update velocity using filtered acceleration
    motion->vx = filtered_acc_x * dt;
    motion->vy = filtered_acc_y * dt;
    motion->vz = filtered_acc_z * dt;

    // Update position using velocity
    motion->x = motion->vx * dt;
    motion->y = motion->vy * dt;
    motion->z = motion->vz * dt;
}

// void processIMUData(IMUData imu_data, MotionData* motion, 
//                    MovingAverage* filter_x, MovingAverage* filter_y, 
//                    MovingAverage* filter_z) {
    
//     currentTime = millis();  // Get current time in milliseconds
//     float dt = (currentTime - lastTime) / 1000.0;  // Convert to seconds
//     lastTime = currentTime;

//     // Apply moving average filter to acceleration data
//     float filtered_acc_x = updateMovingAverage(filter_x, imu_data.acc_x);
//     float filtered_acc_y = updateMovingAverage(filter_y, imu_data.acc_y);
//     float filtered_acc_z = updateMovingAverage(filter_z, imu_data.acc_z - G);

//     // Update velocity using filtered acceleration
//     motion->vx += filtered_acc_x * dt;
//     motion->vy += filtered_acc_y * dt;
//     motion->vz += filtered_acc_z * dt;

//     // Update position using velocity
//     motion->x += motion->vx * dt;
//     motion->y += motion->vy * dt;
//     motion->z += motion->vz * dt;
// }

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    
    // Initialize I2C
    Wire.begin(I2C_SDA);
    Wire.begin(I2C_SCL);
    Wire.setClock(400000); // Set I2C clock to 400kHz
    
    // Initialize MPU6050
    initMPU6050();
    Serial.println("MPU6050 initialized");
    
    // Create moving average filters
    filter_x = createMovingAverage(WINDOW_SIZE);
    filter_y = createMovingAverage(WINDOW_SIZE);
    filter_z = createMovingAverage(WINDOW_SIZE);
    filter_pitch = createMovingAverage(WINDOW_SIZE);
    filter_roll = createMovingAverage(WINDOW_SIZE);
    
    // Initialize timing
    lastSampleTime = millis();
}

void loop() {
    currentTime = millis(); //millis() gives the number of milliseconds elapsed since start of program 
    
    // Check if it's time for a new sample
    if (currentTime - lastSampleTime >= (1000 / SAMPLE_RATE)) {
        // Read IMU data
        IMUData imu_data = readIMUData(); 
        
        // Process data
        processIMUData(imu_data, &motion, filter_x, filter_y, filter_z);
        
        // Print results
        Serial.printf("Position: (%.2f, %.2f, %.2f) m\n", 
                     motion.x, motion.y, motion.z);
        Serial.printf("Velocity: (%.2f, %.2f, %.2f) m/s\n", 
                     motion.vx, motion.vy, motion.vz);
        
        // Update timing
        lastSampleTime = currentTime;
    }
}











