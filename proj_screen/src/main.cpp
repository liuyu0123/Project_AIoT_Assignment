#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>// 显示屏引脚（需与硬件接线一致）
#include <image.h>
#include <image2.h>

#define TFT_CS   41
#define TFT_RST  45
#define TFT_DC   40
#define TFT_MOSI 47
#define TFT_SCLK 21
#define TFT_BL   42
// 创建显示屏对象
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 项目启动");
  
  // 初始化背光
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  
  // 初始化显示屏
  tft.init(240, 320);   // 屏幕分辨率 240x320
  tft.setRotation(1);   // 横屏显示
  tft.fillScreen(ST77XX_BLACK); // 清屏为黑色
  
  // 测试显示颜色
  tft.fillScreen(ST77XX_RED);
  delay(1000);
  tft.fillScreen(ST77XX_GREEN);
  delay(1000);
  tft.fillScreen(ST77XX_BLUE);
  delay(1000);
  tft.fillScreen(ST77XX_BLACK);
  }
  
  void loop() {
  // 主循环（暂为空，可后续扩展音频、串口交互等）
  // tft.drawRGBBitmap(0,0,image_1, image_1_width, image_1_height); // 清屏为黑色
  tft.drawRGBBitmap(0,0,image_2, image_2_width, image_2_height); // 清屏为黑色

  Serial.println("主循环运行中...");
  delay(2000);
  }