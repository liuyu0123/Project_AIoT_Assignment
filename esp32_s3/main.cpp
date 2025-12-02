#include <Wire.h>
#include <MPU6050.h>
#include "HX711.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// ==================== 硬件引脚定义 ====================
const int FSR_PIN = 34;           // FSR402压力传感器引脚
const int TRIG_PIN = 26;          // HC-SR04超声波Trig引脚
const int ECHO_PIN = 27;          // HC-SR04超声波Echo引脚
const int STOP_BUTTON_PIN = 14;   // 急停按钮引脚
const int MODE_BUTTON_PIN = 25;   // 模式切换按钮引脚
const int LOADCELL_DOUT_PIN = 32; // HX711数据引脚
const int LOADCELL_SCK_PIN = 33;  // HX711时钟引脚
const int SERVO_PIN = 13;         // 舵机控制引脚

// ==================== WiFi配置 ====================
const char* ssid = "Wokwi-GUEST"; // WiFi名称
const char* password = "";        // WiFi密码
const int wifiChannel = 6;        // WiFi通道

// ==================== MQTT配置 ====================
const char* mqtt_server = "broker.hivemq.com"; // MQTT服务器地址
const int mqtt_port = 1883;                    // MQTT端口
const char* mqtt_user = "";                    // MQTT用户名
const char* mqtt_password = "";                // MQTT密码

// ==================== MQTT主题 ====================
const char* topic_control = "smartfarm/control"; // 控制主题
const char* topic_data = "smartfarm/data";       // 数据主题
const char* topic_servo = "smartfarm/servo";     // 舵机控制主题

// ==================== 传感器对象 ====================
MPU6050 mpu;      // MPU6050加速度计和陀螺仪
HX711 scale;      // HX711重量传感器
Servo myServo;    // 舵机对象

// ==================== WiFi和MQTT客户端 ====================
WiFiClient espClient;
PubSubClient client(espClient);

// ==================== 全局变量 ====================
unsigned long lastStopDebounce = 0;      // 急停按钮防抖计时
unsigned long lastModeDebounce = 0;      // 模式按钮防抖计时
const unsigned long debounceDelay = 50;  // 按钮防抖延迟
bool lastStopState = HIGH;               // 急停按钮上次状态
bool lastModeState = HIGH;               // 模式按钮上次状态
bool systemStopped = false;              // 系统停止状态
int currentMode = 0;                     // 当前显示模式
float calibration_factor = 2280;         // HX711校准因子
int servoAngle = 90;                     // 舵机当前角度
unsigned long lastReconnectAttempt = 0;  // 上次重连尝试时间
const unsigned long reconnectInterval = 5000; // MQTT重连间隔
unsigned long lastDataPublish = 0;       // 上次数据发布时间
const unsigned long publishInterval = 2000;   // 数据发布间隔

// ==================== 函数声明 ====================
void setupWiFi();
void mqttCallback(char* topic, byte* payload, unsigned int length);
boolean reconnect();
void handleStopButton();
void handleModeButton();
void readAllSensors();
void readMPU6050();
void readFSR402();
void readHX711();
void readHC_SR04();
void publishSensorData();
void testServo();

// ==================== 初始化设置 ====================
void setup() {
  Serial.begin(115200);
  Serial.println("正在启动智能农业监测系统...");
  
  // 初始化WiFi连接
  setupWiFi();
  
  // 初始化舵机
  myServo.attach(SERVO_PIN);
  myServo.write(servoAngle);
  Serial.println("舵机初始化完成");
  
  // 初始化MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  
  // 初始化MPU6050
  Wire.begin();
  Serial.println("初始化MPU6050...");
  
  mpu.initialize();
  
  // 验证MPU6050连接
  if (mpu.testConnection()) {
    Serial.println("MPU6050连接成功");
  } else {
    Serial.println("MPU6050初始化失败，请检查接线或I2C地址!");
    while (1);
  }
  
  // 设置MPU6050范围和灵敏度
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  Serial.println("MPU6050初始化成功");
  
  // 初始化FSR402
  pinMode(FSR_PIN, INPUT);
  Serial.println("FSR402压力传感器就绪");
  
  // 初始化HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare();
  Serial.println("HX711称重传感器初始化完成");
  
  // 初始化HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("HC-SR04超声波传感器就绪");
  
  // 初始化按钮
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("按钮控制系统启动");
  
  // 测试舵机
  testServo();
  
  Serial.println("\n===== 所有传感器初始化完成 =====");
  Serial.println("按急停按钮暂停/恢复系统");
  Serial.println("按模式按钮切换显示模式");
  Serial.println("模式0: 所有传感器数据");
  Serial.println("模式1: 仅MPU6050数据");
  Serial.println("模式2: 仅FSR402和HX711数据");
  Serial.println("通过MQTT主题 'smartfarm/servo' 控制舵机角度(0-180)");
  Serial.println("通过MQTT主题 'smartfarm/control' 发送控制命令");
  Serial.println("===============================\n");
}

// ==================== WiFi连接设置 ====================
void setupWiFi() {
  Serial.print("正在连接到WiFi");
  WiFi.begin(ssid, password, wifiChannel);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  
  Serial.println(" 连接成功!");
  Serial.print("IP地址: ");
  Serial.println(WiFi.localIP());
}

// ==================== MQTT回调函数 ====================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("收到MQTT消息 [");
  Serial.print(topic);
  Serial.print("]: ");
  
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  
  // 处理舵机控制消息
  if (String(topic) == topic_servo) {
    int angle = message.toInt();
    if (angle >= 0 && angle <= 180) {
      servoAngle = angle;
      myServo.write(servoAngle);
      Serial.print("设置舵机角度: ");
      Serial.println(servoAngle);
      
      // 发布确认消息
      String confirmMsg = "Servo set to " + String(servoAngle);
      client.publish(topic_control, confirmMsg.c_str());
    } else {
      Serial.println("无效的舵机角度，请输入0-180之间的值");
    }
  }
  
  // 处理系统控制消息
  if (String(topic) == topic_control) {
    if (message == "stop") {
      systemStopped = true;
      Serial.println("通过MQTT停止系统");
      client.publish(topic_control, "System stopped");
    } else if (message == "start") {
      systemStopped = false;
      Serial.println("通过MQTT启动系统");
      client.publish(topic_control, "System started");
    } else if (message == "toggle") {
      systemStopped = !systemStopped;
      Serial.print("通过MQTT切换系统状态: ");
      Serial.println(systemStopped ? "停止" : "运行");
      String statusMsg = systemStopped ? "System stopped" : "System started";
      client.publish(topic_control, statusMsg.c_str());
    } else if (message.startsWith("mode ")) {
      // 处理模式切换命令
      int newMode = message.substring(5).toInt();
      if (newMode >= 0 && newMode <= 2) {
        currentMode = newMode;
        Serial.printf("通过MQTT切换到模式 %d\n", currentMode);
        String modeMsg = "Mode changed to " + String(currentMode);
        client.publish(topic_control, modeMsg.c_str());
      } else {
        Serial.println("无效的模式，请输入0-2之间的值");
      }
    }
  }
}

// ==================== MQTT重连函数 ====================
boolean reconnect() {
  if (client.connect("ESP32_SmartFarm", mqtt_user, mqtt_password)) {
    Serial.println("MQTT连接成功");
    
    // 订阅主题
    client.subscribe(topic_control);
    client.subscribe(topic_servo);
    
    // 发布连接成功消息
    client.publish(topic_control, "ESP32 connected");
    return true;
  } else {
    Serial.print("MQTT连接失败, rc=");
    Serial.print(client.state());
    Serial.println(" 5秒后重试...");
    return false;
  }
}

// ==================== 主循环 ====================
void loop() {
  // 确保MQTT连接
  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > reconnectInterval) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
  }
  
  // 处理按钮输入
  handleStopButton();
  handleModeButton();
  
  if (systemStopped) {
    if (millis() % 2000 < 100) { // 每2秒闪烁一次
      Serial.println("系统已暂停 - 按急停按钮恢复");
    }
    delay(100);
    return;
  }
  
  // 根据当前模式显示相应的传感器数据
  switch (currentMode) {
    case 0:
      readAllSensors();
      break;
    case 1:
      readMPU6050();
      break;
    case 2:
      readFSR402();
      readHX711();
      break;
  }
  
  // 定期发布传感器数据到MQTT
  if (millis() - lastDataPublish > publishInterval) {
    lastDataPublish = millis();
    publishSensorData();
  }
  
  delay(500);
}

// ==================== 急停按钮处理 ====================
void handleStopButton() {
  int reading = digitalRead(STOP_BUTTON_PIN);
  if (reading != lastStopState) {
    lastStopDebounce = millis();
  }
  
  if ((millis() - lastStopDebounce) > debounceDelay) {
    if (reading == LOW && lastStopState == HIGH) {
      systemStopped = !systemStopped;
      Serial.println(systemStopped ? "\n系统已停止" : "\n系统已恢复");
      currentMode = systemStopped ? 0 : currentMode;
      
      // 发布状态变化到MQTT
      String statusMsg = systemStopped ? "System stopped by button" : "System started by button";
      if (client.connected()) {
        client.publish(topic_control, statusMsg.c_str());
      }
    }
  }
  
  lastStopState = reading;
}

// ==================== 模式按钮处理 ====================
void handleModeButton() {
  if (systemStopped) return;
  
  int reading = digitalRead(MODE_BUTTON_PIN);
  if (reading != lastModeState) {
    lastModeDebounce = millis();
  }
  
  if ((millis() - lastModeDebounce) > debounceDelay) {
    if (reading == LOW && lastModeState == HIGH) {
      currentMode = (currentMode + 1) % 3;
      Serial.printf("\n切换到模式 %d\n", currentMode);
      
      // 发布模式变化到MQTT
      String modeMsg = "Mode changed to " + String(currentMode);
      if (client.connected()) {
        client.publish(topic_control, modeMsg.c_str());
      }
    }
  }
  
  lastModeState = reading;
}

// ==================== 读取所有传感器数据 ====================
void readAllSensors() {
  readMPU6050();
  readFSR402();
  readHX711();
  readHC_SR04();
  Serial.println("----------");
}

// ==================== 读取MPU6050数据 ====================
void readMPU6050() {
  // 读取原始加速度和陀螺仪数据
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // 转换为实际单位
  float accelX = ax / 16384.0;  // ±2g范围，灵敏度为16384 LSB/g
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  
  float gyroX = gx / 16.4;     // ±2000度/秒范围，灵敏度为16.4 LSB/(度/秒)
  float gyroY = gy / 16.4;
  float gyroZ = gz / 16.4;
  
  Serial.printf("加速度 X:%.2fg Y:%.2fg Z:%.2fg | ", accelX, accelY, accelZ);
  Serial.printf("陀螺仪 X:%.2f°/s Y:%.2f°/s Z:%.2f°/s\n", gyroX, gyroY, gyroZ);
}

// ==================== 读取FSR402数据 ====================
void readFSR402() {
  int rawValue = analogRead(FSR_PIN);
  float voltage = rawValue * (3.3 / 4095.0);
  
  Serial.printf("FSR ADC值: %d | 电压: %.2fV | 压力: ", rawValue, voltage);
  if (rawValue < 100) Serial.println("无压力");
  else if (rawValue < 500) Serial.println("轻压");
  else if (rawValue < 800) Serial.println("中压");
  else Serial.println("重压");
}

// ==================== 读取HX711数据 ====================
void readHX711() {
  float weight = scale.get_units(5);
  Serial.printf("重量: %.2f kg\n", weight);
}

// ==================== 读取HC-SR04数据 ====================
void readHC_SR04() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  
  Serial.printf("距离: %ld cm", distance);
  if (distance < 10) Serial.println(" - 很近!");
  else if (distance < 50) Serial.println(" - 近距离");
  else if (distance < 200) Serial.println(" - 中距离");
  else Serial.println(" - 远距离");
}

// ==================== 发布传感器数据到MQTT ====================
void publishSensorData() {
  if (!client.connected()) {
    return; // 如果MQTT未连接，则不发布数据
  }
  
  // 创建JSON格式的传感器数据
  String jsonData = "{";
  
  // 添加MPU6050数据
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  jsonData += "\"accelX\":" + String(ax / 16384.0, 2) + ",";
  jsonData += "\"accelY\":" + String(ay / 16384.0, 2) + ",";
  jsonData += "\"accelZ\":" + String(az / 16384.0, 2) + ",";
  jsonData += "\"gyroX\":" + String(gx / 16.4, 2) + ",";
  jsonData += "\"gyroY\":" + String(gy / 16.4, 2) + ",";
  jsonData += "\"gyroZ\":" + String(gz / 16.4, 2) + ",";
  
  // 添加FSR402数据
  int fsrValue = analogRead(FSR_PIN);
  jsonData += "\"fsrValue\":" + String(fsrValue) + ",";
  
  // 添加HX711数据
  float weight = scale.get_units(5);
  jsonData += "\"weight\":" + String(weight, 2) + ",";
  
  // 添加HC-SR04数据
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  jsonData += "\"distance\":" + String(distance) + ",";
  
  // 添加系统状态
  jsonData += "\"systemStopped\":" + String(systemStopped ? "true" : "false") + ",";
  jsonData += "\"currentMode\":" + String(currentMode) + ",";
  jsonData += "\"servoAngle\":" + String(servoAngle);
  
  jsonData += "}";
  
  // 发布到MQTT
  client.publish(topic_data, jsonData.c_str());
  Serial.println("传感器数据已发布到MQTT");
}

// ==================== 舵机测试函数 ====================
void testServo() {
  Serial.println("测试舵机...");
  
  // 测试0度
  myServo.write(0);
  Serial.println("舵机角度: 0°");
  delay(1000);
  
  // 测试90度
  myServo.write(90);
  Serial.println("舵机角度: 90°");
  delay(1000);
  
  // 测试180度
  myServo.write(180);
  Serial.println("舵机角度: 180°");
  delay(1000);
  
  // 回到初始位置
  myServo.write(servoAngle);
  Serial.println("舵机测试完成，回到初始位置");
}