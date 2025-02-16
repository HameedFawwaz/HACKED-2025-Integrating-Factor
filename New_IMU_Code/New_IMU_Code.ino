#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define MPU6050_ADDR            0x68    // MPU6050 device address
 #define MPU6050_ACCEL_XOUT_H    0x3B
 #define MPU6050_PWR_MGMT_1      0x6B

// Processing Configuration
#define WINDOW_SIZE     10      // Size of moving average window
#define SAMPLE_RATE     100     // Sample rate in Hz
#define G               9.81    // Gravity constant

// // I2C pins for ESP32 -- need this?
 #define I2C_SDA 21
 #define I2C_SCL 22

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

// Circular buffer for moving average
typedef struct {
    float* values;
    int size;
    int index;
    int count;
    float sum;
} MovingAverage;



void setup(void) {
Serial.begin(115200);
while (!Serial)
delay(10); // will pause Zero, Leonardo, etc until serial console opens
Serial.println("Adafruit MPU6050 test!");
// Try to initialize!
if (!mpu.begin()) {
Serial.println("Failed to find MPU6050 chip");
while (1) {
delay(10);
}
}
Serial.println("MPU6050 Found!");
mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
Serial.print("Accelerometer range set to: ");
switch (mpu.getAccelerometerRange()) {
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
switch (mpu.getGyroRange()) {
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
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
Serial.print("Filter bandwidth set to: ");
switch (mpu.getFilterBandwidth()) {
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





// Global variables
MovingAverage* filter_x;
MovingAverage* filter_y;
MovingAverage* filter_z;
MotionData motion = {0};
unsigned long lastSampleTime = 0;
unsigned long currentTime;
unsigned long lastTime;

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


void readIMUData() {
      IMUData data;
      int16_t ax, ay, az, gx, gy, gz;
      
      sensors_event_t a, g, temp;
      data = mpu.getEvent(%a, &g, &temp);

    ax = a.acceleration.x
    ay = a.acceleration.y
    az = a.acceleration.z

    gx = g.gyro.x
    gy = g.gyro.y
    gz = g.gyro.z

    
    // Convert to meaningful values
    data.acc_x = ax / 16384.0 * G;  // Convert to m/s²
    data.acc_y = ay / 16384.0 * G;
    data.acc_z = az / 16384.0 * G;
    data.gyro_x = gx / 131.0;   // In degrees/second
    data.gyro_y = gy / 131.0;
    data.gyro_z = gz / 131.0;
    
    return data;
}



void processIMUData(IMUData imu_data, MotionData* motion, 
                   MovingAverage* filter_x, MovingAverage* filter_y, 
                   MovingAverage* filter_z) {
    
    currentTime = millis();  // Get current time in milliseconds
    float dt = (currentTime - lastTime) / 1000.0;  // Convert to seconds
    lastTime = currentTime;

    // Apply moving average filter to acceleration data
    float filtered_acc_x = updateMovingAverage(filter_x, imu_data.acc_x);
    float filtered_acc_y = updateMovingAverage(filter_y, imu_data.acc_y);
    float filtered_acc_z = updateMovingAverage(filter_z, imu_data.acc_z - G);

    // Update velocity using filtered acceleration
    motion->vx += filtered_acc_x * dt;
    motion->vy += filtered_acc_y * dt;
    motion->vz += filtered_acc_z * dt;

    // Update position using velocity
    motion->x += motion->vx * dt;
    motion->y += motion->vy * dt;
    motion->z += motion->vz * dt;
}




void loop() {

  currentTime = millis(); //millis() gives the number of milliseconds elapsed since start of program 
/* Get new sensor events with the readings */

data = processIMUdata(data);

if (currentTime - lastSampleTime >= (1000 / SAMPLE_RATE)) {
        // Read IMU data

        sensors_event_t a, g, temp;
        data = readIMUdata();
        IMUData imu_data = readIMUData(); 
        
        // Process data
        processIMUData(imu_data, &motion, filter_x, filter_y, filter_z);
        
        // Print results

        Serial.printf("Position: (%.2f, %.2f, %.2f) m\n", motion.x, motion.y, motion.z);
        Serial.printf("Velocity: (%.2f, %.2f, %.2f) m/s\n", motion.vx, motion.vy, motion.vz);

        
        // Update timing
        lastSampleTime = currentTime;
    }



/* Print out the values */
Serial.print("Acceleration X: ");
Serial.print(a.acceleration.x);
Serial.print(", Y: ");
Serial.print(a.acceleration.y);
Serial.print(", Z: ");
Serial.print(a.acceleration.z);
Serial.println(" m/s^2");
Serial.print("Rotation X: ");
Serial.print(g.gyro.x);
Serial.print(", Y: ");
Serial.print(g.gyro.y);
Serial.print(", Z: ");
Serial.print(g.gyro.z);
Serial.println(" rad/s");
Serial.print("Temperature: ");
Serial.print(temp.temperature);
Serial.println(" degC");
Serial.println("");
delay(500);
}