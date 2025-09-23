#include <WiFi.h>              // 包含WiFi库，用于ESP32的WiFi连接功能
#include <PubSubClient.h>      // 包含PubSubClient库，用于MQTT协议的客户端实现
 
// ====== 配置区域 ======
// 以下是用户需要配置的部分，包括WiFi和MQTT服务器的信息
const char* ssid = "YourWiFiName";          // WiFi 名称 (与电脑要同一网络)，替换为你的WiFi SSID
const char* password = "YourWiFiPassword";  // WiFi 密码，替换为你的WiFi密码
const char* mqtt_server = "192.168.1.100";  // Windows 主机的局域网 IP，替换为你的MQTT服务器IP地址
 
WiFiClient espClient;           // 创建一个WiFiClient对象，用于TCP连接
PubSubClient client(espClient); // 创建PubSubClient对象，基于WiFiClient，用于MQTT通信
 
// MQTT消息回调函数，当收到订阅主题的消息时调用
void callback(char* topic, byte* message, unsigned int length) {  Serial.print("收到消息: ");  // 打印收到消息的提示
  Serial.print(topic);         // 打印主题名称
  Serial.print(" 内容=");      // 打印内容提示
  for (int i = 0; i < length; i++) { // 循环遍历消息字节
    Serial.print((char)message[i]);  // 逐字节打印消息内容
  }
  Serial.println();            // 换行结束打印
}
 
// 重新连接MQTT服务器的函数，如果连接断开则尝试重连
void reconnect() {  while (!client.connected()) {    // 循环直到连接成功
    Serial.print("尝试连接MQTT..."); // 打印连接尝试提示
    if (client.connect("ESP32Client")) { // 尝试连接MQTT服务器，使用客户端ID "ESP32Client"
      Serial.println("成功!");           // 连接成功提示
      client.subscribe("test/topic");    // 订阅主题 "test/topic"
    } else {                             // 连接失败
      Serial.print("失败 rc=");          // 打印失败提示
      Serial.print(client.state());      // 打印连接状态码
      delay(2000);                       // 等待2秒后重试
    }
  }
}
 
// setup函数，Arduino/ESP32程序的初始化部分，只执行一次
void setup() {  Serial.begin(115200);  // 初始化串口通信，波特率115200，用于调试输出
  WiFi.begin(ssid, password); // 开始连接WiFi，使用配置的SSID和密码
  while (WiFi.status() != WL_CONNECTED) { // 循环等待WiFi连接成功
    delay(500);                           // 每0.5秒检查一次
    Serial.print(".");                    // 打印进度点
  }
  Serial.println("\nWiFi已连接");         // WiFi连接成功提示
  client.setServer(mqtt_server, 1883);    // 设置MQTT服务器地址和端口（默认1883）
  client.setCallback(callback);           // 设置MQTT消息回调函数
}
 
// loop函数，Arduino/ESP32程序的主循环，重复执行
void loop() {  if (!client.connected()) { // 检查MQTT连接状态，如果断开则重连
    reconnect();             // 调用重连函数
  }
  client.loop();             // 处理MQTT客户端的循环任务，如保持连接、处理消息

  // 定时发布一条消息的部分
  static long lastMsg = 0;   // 静态变量，记录上次发送消息的时间（毫秒）
  long now = millis();       // 获取当前时间（毫秒）
  if (now - lastMsg > 3000) { // 如果距离上次发送超过3秒
    lastMsg = now;            // 更新上次发送时间
    String payload = "ESP32 online message!"; // 创建要发送的消息负载
    client.publish("test/topic", payload.c_str()); // 发布消息到主题 "test/topic"
  }
}