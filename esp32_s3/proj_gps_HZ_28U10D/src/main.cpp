#include <Arduino.h>
#include <TinyGPSPlus.h>  // GPS解析库（PlatformIO会自动安装）

// -------------------------- 配置参数 --------------------------
#define GPS_TX_PIN 17  // ESP32-S3 发送引脚（连接GPS RX）
#define GPS_RX_PIN 16  // ESP32-S3 接收引脚（连接GPS TX）
#define GPS_BAUD 9600  // HZ-28U10D 默认波特率（9600-N-8-1）
#define SERIAL_BAUD 115200  // 串口监视器波特率（与platformio.ini一致）

// 创建GPS解析对象和硬件串口对象（使用Serial2）
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // ESP32-S3 硬件串口2（RX2=GPIO16，TX2=GPIO17）

// -------------------------- 函数声明 --------------------------
void printGPSData();  // 打印GPS解析后的数据
void checkGPSStatus();  // 检查GPS信号状态

void setup() {
  // 初始化串口监视器（用于调试输出）
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);  // 等待USB串口就绪（PlatformIO中可省略，但保留更稳定）
  
  // 初始化GPS模块串口（Serial2）
  // 格式：begin(波特率, 串口模式, RX引脚, TX引脚)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // 打印初始化信息
  Serial.println("=== ESP32-S3 + HZ-28U10D GPS 采集程序(PlatformIO版) ===");
  Serial.printf("GPS串口已初始化(波特率:%d, RX:%d, TX:%d)\n", 
               GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("等待GPS信号...（请确保在户外开阔环境）");
}

void loop() {
  // 读取GPS串口数据并解析（持续接收NMEA语句）
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    // 可选：启用串口回显，查看原始NMEA数据（调试用）
    // Serial.write(c);
    gps.encode(c);  // 解析NMEA语句
  }

  // 每1秒打印一次GPS数据（避免输出过快）
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    checkGPSStatus();
    printGPSData();
    Serial.println("----------------------------------------");
  }

  // 检查GPS数据接收状态（异常提示）
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("警告: 未接收到GPS数据!");
    Serial.println("  请检查, 1. 接线是否正确 (GPS TX→ESP32 RX16) 2. 模块电源(3.3V) 3. 波特率是否匹配");
  }

  // 移除错误的 reset() 调用！TinyGPSPlus 库无此方法，无需手动重置（库内部会自动管理缓冲区）
}

// -------------------------- 函数实现 --------------------------
// 检查GPS信号状态（卫星数、定位状态）
void checkGPSStatus() {
  Serial.print("GPS状态:");
  if (gps.satellites.value() > 0) {
    // HDOP：水平精度因子（0.5-1.0优秀，1.0-2.0良好，>5.0较差）
    Serial.printf("已搜索到卫星(数量：%d) | HDOP: %.1f | 定位状态：%s\n",
                 gps.satellites.value(),
                 gps.hdop.hdop(),
                 gps.location.isValid() ? "已定位" : "未定位");
  } else {
    Serial.println("未搜索到卫星 (请在户外开阔环境等待)");
  }
}

// 打印解析后的GPS数据（经纬度、海拔、速度等）
void printGPSData() {
  // 纬度（度.分格式，N/S表示南北半球）
  Serial.print("纬度：");
  if (gps.location.isValid()) {
    Serial.printf("%.6f %c\n",
                 gps.location.lat(),
                 gps.location.lat() > 0 ? 'N' : 'S');
  } else {
    Serial.println("无效");
  }

  // 经度（度.分格式，E/W表示东西半球）
  Serial.print("经度：");
  if (gps.location.isValid()) {
    Serial.printf("%.6f %c\n",
                 gps.location.lng(),
                 gps.location.lng() > 0 ? 'E' : 'W');
  } else {
    Serial.println("无效");
  }

  // 海拔高度（单位：米，GPS椭球高度，与实际海拔有差异）
  Serial.print("海拔：");
  if (gps.altitude.isValid()) {
    Serial.printf("%.1f 米\n", gps.altitude.meters());
  } else {
    Serial.println("无效");
  }

  // 移动速度（单位：km/h）
  Serial.print("速度：");
  if (gps.speed.isValid()) {
    Serial.printf("%.1f km/h\n", gps.speed.kmph());
  } else {
    Serial.println("无效");
  }

  // UTC时间（GPS时间，需自行转换为本地时间）
  Serial.print("UTC时间: ");
  if (gps.time.isValid()) {
    Serial.printf("%02d:%02d:%02d\n",
                 gps.time.hour(),
                 gps.time.minute(),
                 gps.time.second());
  } else {
    Serial.println("无效");
  }

  // UTC日期（年-月-日）
  Serial.print("UTC日期: ");
  if (gps.date.isValid()) {
    Serial.printf("20%02d-%02d-%02d\n",
                 gps.date.year(),
                 gps.date.month(),
                 gps.date.day());
  } else {
    Serial.println("无效");
  }
}