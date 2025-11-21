#include <MPU6050.h>

// ==================== 硬件引脚定义 ====================
// ESP32-S3默认IIC的接口为8（SDA）和9（SCL）

// ==================== 传感器对象 ====================
MPU6050 mpu; // MPU6050加速度计和陀螺仪

void readMPU6050();
void print_imu_log();

// ==================== 读取MPU6050数据 ====================
void readMPU6050()
{
    // 读取原始加速度和陀螺仪数据
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 转换为实际单位
    float accelX = ax / 16384.0; // ±2g范围，灵敏度为16384 LSB/g
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    float gyroX = gx / 16.4; // ±2000度/秒范围，灵敏度为16.4 LSB/(度/秒)
    float gyroY = gy / 16.4;
    float gyroZ = gz / 16.4;

    Serial.printf("加速度 X:%.2fg Y:%.2fg Z:%.2fg | ", accelX, accelY, accelZ);
    Serial.printf("陀螺仪 X:%.2f°/s Y:%.2f°/s Z:%.2f°/s\n", gyroX, gyroY, gyroZ);
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Hello, ESP32-S3!");
    // 初始化MPU6050
    Wire.begin();
    Serial.println("初始化MPU6050...");
    // 验证MPU6050连接
    if (mpu.testConnection())
    {
        Serial.println("MPU6050连接成功");
    }
    else
    {
        Serial.println("MPU6050初始化失败，请检查接线或I2C地址!");
        while (1)
            ;
    }
    // 设置MPU6050范围和灵敏度
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    Serial.println("MPU6050初始化成功");
}

void loop()
{
    // put your main code here, to run repeatedly:
    delay(10); // this speeds up the simulation
    readMPU6050();
}
