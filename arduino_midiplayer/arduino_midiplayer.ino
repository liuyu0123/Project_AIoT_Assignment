#include <driver/i2s.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>
#include <vector>

// I2S配置 - 连接MAX98357
#define I2S_WS_PIN      39    // LRCLK (Word Select) - 左右声道时钟
#define I2S_SCK_PIN     38    // BCLK (Bit Clock) - 位时钟
#define I2S_SD_PIN      14    // DIN (Data Input) - 数据输入

// MIDI合成器相关常量
#define SAMPLE_RATE     22050
#define CHANNELS        1
#define BITS_PER_SAMPLE 16
#define AUDIO_BUFFER_SIZE 64
#define MAX_MELODY_LENGTH 256  // 最大旋律长度

// 简单的波形表 - 正弦波
int16_t sine_wave[256];

// MIDI音符相位增量表
uint32_t phase_increments[128];

// MIDI事件队列
QueueHandle_t midiQueue;

// MIDI播放状态
struct MIDINote {
  uint8_t note;
  uint8_t velocity;
  volatile uint32_t phase;
  uint32_t phase_increment;
  volatile bool active;
};

MIDINote active_notes[16];
WebServer server(80);
TaskHandle_t melodyTaskHandle = NULL;
TaskHandle_t audioTaskHandle = NULL;
TaskHandle_t customMelodyTaskHandle = NULL;  // 自定义旋律任务句柄

// MIDI事件结构体
struct MidiEvent {
  uint8_t type;  // 0: noteOn, 1: noteOff, 2: allOff
  uint8_t note;
  uint8_t velocity;
};

// 自定义旋律结构体
struct CustomMelody {
  std::vector<uint8_t> notes;
  std::vector<uint16_t> durations;
  int length;
};

CustomMelody customMelody;
SemaphoreHandle_t melodyMutex;  // 保护自定义旋律数据

// 前向声明所有函数
void generateSineWave();
void precomputePhaseIncrements();
void setupI2S();
void audioTask(void *parameter);
void melodyTask(void *parameter);
void customMelodyTask(void *parameter);
void generateAudioBuffer();
void noteOn(uint8_t note, uint8_t velocity);
void noteOff(uint8_t note);
void allNotesOff();
void handleRoot();
void handlePlay();
void handleStop();
void handleSetMelody();
void handlePlayCustom();
bool parseNumberArray(String input, std::vector<uint16_t>& output);
bool parseNoteArray(String input, std::vector<uint8_t>& output);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-CAM MIDI播放器启动 ===");
  
  // 创建互斥锁
  melodyMutex = xSemaphoreCreateMutex();
  
  // 初始化音频数据
  Serial.println("初始化音频数据...");
  generateSineWave();
  precomputePhaseIncrements();
  
  // 初始化所有音符状态为非活跃
  for(int i = 0; i < 16; i++) {
    active_notes[i].active = false;
    active_notes[i].phase = 0;
    active_notes[i].note = 0;
    active_notes[i].velocity = 0;
    active_notes[i].phase_increment = 0;
  }
  Serial.println("✓ 音符状态初始化完成");
  
  // 初始化默认旋律（小星星）
  customMelody.notes = {60, 60, 67, 67, 69, 69, 67};
  customMelody.durations = {500, 500, 500, 500, 500, 500, 1000};
  customMelody.length = 7;
  
  // 创建MIDI事件队列
  midiQueue = xQueueCreate(32, sizeof(MidiEvent));
  if (midiQueue == NULL) {
    Serial.println("错误: 无法创建MIDI队列");
    return;
  }
  Serial.println("✓ MIDI队列创建成功");
  
  // 初始化SPIFFS文件系统
  if(!SPIFFS.begin(true)){
    Serial.println("错误: SPIFFS初始化失败");
    return;
  }
  Serial.println("✓ SPIFFS文件系统初始化成功");
  
  // 连接WiFi网络
  Serial.println("正在连接WiFi...");
  // WiFi.begin("jsjqr777", "js123456");
  WiFi.begin("LIUYU_Iphone", "19940704");
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi连接成功");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n⚠ WiFi连接失败，将继续以离线模式运行");
  }
  
  // 初始化I2S音频输出
  setupI2S();
  
  // 创建音频任务
  BaseType_t taskCreated = xTaskCreatePinnedToCore(
    audioTask,
    "AudioTask",
    4096,
    NULL,
    2,
    &audioTaskHandle,
    0
  );
  
  if (taskCreated != pdPASS) {
    Serial.println("错误: 无法创建音频任务");
    return;
  }
  Serial.println("✓ 音频任务创建成功");
  
  // 设置Web服务器路由
  if (WiFi.status() == WL_CONNECTED) {
    server.on("/", handleRoot);
    server.on("/play", handlePlay);
    server.on("/stop", handleStop);
    server.on("/setMelody", HTTP_POST, handleSetMelody);  // 新增：设置自定义旋律
    server.on("/playCustom", handlePlayCustom);           // 新增：播放自定义旋律
    server.begin();
    Serial.println("✓ Web服务器启动成功");
  }
  
  Serial.println("=== 系统启动完成 ===");
}

void loop() {
  // 处理Web服务器请求
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }
  
  // 处理MIDI事件队列
  MidiEvent event;
  if (xQueueReceive(midiQueue, &event, 0) == pdTRUE) {
    if (event.type == 0) {
      noteOn(event.note, event.velocity);
    } else if (event.type == 1) {
      noteOff(event.note);
    } else if (event.type == 2) {
      allNotesOff();
    }
  }
  
  vTaskDelay(1 / portTICK_PERIOD_MS);
}

// 音频任务：生成并输出音频数据
void audioTask(void *parameter) {
  Serial.println("音频任务启动 (Core 0)");
  while (1) {
    generateAudioBuffer();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// 默认旋律任务：播放小星星
void melodyTask(void *parameter) {
  Serial.println("默认旋律任务启动 (Core 1)");
  
  // 默认小星星旋律
  static const uint8_t melody[] = {60, 60, 67, 67, 69, 69, 67};
  static const uint16_t durations[] = {500, 500, 500, 500, 500, 500, 1000};
  static const int melody_length = 7;
  
  for(int i = 0; i < melody_length; i++) {
    MidiEvent eventOn = {0, melody[i], 80};
    xQueueSend(midiQueue, &eventOn, portMAX_DELAY);
    vTaskDelay(durations[i] / portTICK_PERIOD_MS);
    MidiEvent eventOff = {1, melody[i], 0};
    xQueueSend(midiQueue, &eventOff, portMAX_DELAY);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  
  Serial.println("默认旋律任务完成");
  melodyTaskHandle = NULL;
  vTaskDelete(NULL);
}

// 自定义旋律任务：播放用户输入的旋律
void customMelodyTask(void *parameter) {
  Serial.println("自定义旋律任务启动 (Core 1)");
  
  // 复制旋律数据（避免并发问题）
  std::vector<uint8_t> notes;
  std::vector<uint16_t> durations;
  int length;
  
  if (xSemaphoreTake(melodyMutex, portMAX_DELAY) == pdTRUE) {
    notes = customMelody.notes;
    durations = customMelody.durations;
    length = customMelody.length;
    xSemaphoreGive(melodyMutex);
  } else {
    Serial.println("无法获取旋律数据");
    customMelodyTaskHandle = NULL;
    vTaskDelete(NULL);
    return;
  }
  
  // 播放自定义旋律
  for(int i = 0; i < length && i < notes.size() && i < durations.size(); i++) {
    Serial.printf("播放音符: %d, 时长: %dms\n", notes[i], durations[i]);
    
    MidiEvent eventOn = {0, notes[i], 80};
    xQueueSend(midiQueue, &eventOn, portMAX_DELAY);
    vTaskDelay(durations[i] / portTICK_PERIOD_MS);
    MidiEvent eventOff = {1, notes[i], 0};
    xQueueSend(midiQueue, &eventOff, portMAX_DELAY);
    vTaskDelay(50 / portTICK_PERIOD_MS);  // 短暂休止
  }
  
  Serial.println("自定义旋律任务完成");
  customMelodyTaskHandle = NULL;
  vTaskDelete(NULL);
}

// 生成正弦波波形表
void generateSineWave() {
  for(int i = 0; i < 256; i++) {
    float angle = 2.0f * M_PI * i / 256.0f;
    sine_wave[i] = (int16_t)(sinf(angle) * 16383.0f);
  }
  Serial.println("✓ 正弦波波形表生成完成");
}

// 预计算相位增量表
void precomputePhaseIncrements() {
  for (int note = 0; note < 128; note++) {
    float freq = 440.0f * powf(2.0f, (float)(note - 69) / 12.0f);
    phase_increments[note] = (uint32_t)((freq * 4294967296.0f) / (float)SAMPLE_RATE);
  }
  Serial.println("✓ 相位增量表预计算完成");
}

// 设置I2S音频输出配置
void setupI2S() {
  Serial.println("初始化I2S音频输出...");
  
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_SD_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  esp_err_t result = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (result != ESP_OK) {
    Serial.printf("I2S驱动安装失败: %d\n", result);
    return;
  }
  
  result = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (result != ESP_OK) {
    Serial.printf("I2S引脚设置失败: %d\n", result);
    return;
  }
  
  result = i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  if (result != ESP_OK) {
    Serial.printf("I2S时钟设置失败: %d\n", result);
    return;
  }
  
  Serial.println("✓ I2S音频输出初始化完成");
}

// 生成音频缓冲区并发送到I2S
void generateAudioBuffer() {
  static int16_t audio_buffer[AUDIO_BUFFER_SIZE];
  size_t bytes_written;
  
  for(int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
    int32_t mixed_sample = 0;
    int active_count = 0;
    
    for(int j = 0; j < 16; j++) {
      if (active_notes[j].active) {
        uint32_t current_phase = active_notes[j].phase;
        uint8_t wave_index = (current_phase >> 24) & 0xFF;
        int16_t sample = sine_wave[wave_index];
        uint8_t vel = active_notes[j].velocity;
        sample = (sample * vel) >> 7;
        mixed_sample += sample;
        active_notes[j].phase = current_phase + active_notes[j].phase_increment;
        active_count++;
      }
    }
    
    if(active_count > 0) {
      mixed_sample /= active_count;
    }
    
    if(mixed_sample > 16383) mixed_sample = 16383;
    if(mixed_sample < -16383) mixed_sample = -16383;
    
    audio_buffer[i] = (int16_t)mixed_sample;
  }
  
  i2s_write(I2S_NUM_0, audio_buffer, sizeof(audio_buffer), &bytes_written, portMAX_DELAY);
}

// 开始播放MIDI音符
void noteOn(uint8_t note, uint8_t velocity) {
  if (note > 127 || velocity == 0) {
    Serial.printf("无效音符参数: note=%d, velocity=%d\n", note, velocity);
    return;
  }
  
  for(int i = 0; i < 16; i++) {
    if (!active_notes[i].active) {
      active_notes[i].note = note;
      active_notes[i].velocity = velocity;
      active_notes[i].phase_increment = phase_increments[note];
      active_notes[i].phase = 0;
      active_notes[i].active = true;
      
      float freq = 440.0f * powf(2.0f, (float)(note - 69) / 12.0f);
      Serial.printf("音符开始: %d (%0.2f Hz), 音量: %d\n", note, freq, velocity);
      return;
    }
  }
  Serial.println("警告: 无可用音符槽位");
}

// 停止播放MIDI音符
void noteOff(uint8_t note) {
  for(int i = 0; i < 16; i++) {
    if (active_notes[i].active && active_notes[i].note == note) {
      active_notes[i].active = false;
      Serial.printf("音符停止: %d\n", note);
      return;
    }
  }
}

// 停止所有活跃的音符
void allNotesOff() {
  int stopped_count = 0;
  for(int i = 0; i < 16; i++) {
    if (active_notes[i].active) {
      active_notes[i].active = false;
      stopped_count++;
    }
  }
  Serial.printf("停止了 %d 个音符\n", stopped_count);
}

// 解析音符数组字符串
bool parseNoteArray(String input, std::vector<uint8_t>& output) {
  output.clear();
  input.trim();
  
  int start = 0;
  int end = input.indexOf(',');
  
  while (end != -1) {
    String numStr = input.substring(start, end);
    numStr.trim();
    int num = numStr.toInt();
    if (num >= 0 && num <= 127) {
      output.push_back((uint8_t)num);
    }
    start = end + 1;
    end = input.indexOf(',', start);
  }
  
  // 处理最后一个数字
  String numStr = input.substring(start);
  numStr.trim();
  int num = numStr.toInt();
  if (num >= 0 && num <= 127) {
    output.push_back((uint8_t)num);
  }
  
  return output.size() > 0;
}

// 解析时长数组字符串
bool parseNumberArray(String input, std::vector<uint16_t>& output) {
  output.clear();
  input.trim();
  
  int start = 0;
  int end = input.indexOf(',');
  
  while (end != -1) {
    String numStr = input.substring(start, end);
    numStr.trim();
    int num = numStr.toInt();
    if (num > 0 && num <= 10000) {  // 最大10秒
      output.push_back((uint16_t)num);
    }
    start = end + 1;
    end = input.indexOf(',', start);
  }
  
  // 处理最后一个数字
  String numStr = input.substring(start);
  numStr.trim();
  int num = numStr.toInt();
  if (num > 0 && num <= 10000) {
    output.push_back((uint16_t)num);
  }
  
  return output.size() > 0;
}

// Web服务器处理函数 - 主页面（增强版）
void handleRoot() {
  String html = "<!DOCTYPE html>\n";
  html += "<html lang='zh-cn'>\n";
  html += "<head>\n";
  html += "    <meta charset='UTF-8'>\n";
  html += "    <meta name='viewport' content='width=device-width, initial-scale=1.0'>\n";
  html += "    <title>ESP32 MIDI播放器</title>\n";
  html += "    <style>\n";
  html += "        body {\n";
  html += "            font-family: Arial, sans-serif;\n";
  html += "            max-width: 800px;\n";
  html += "            margin: 0 auto;\n";
  html += "            padding: 20px;\n";
  html += "            background-color: #f0f0f0;\n";
  html += "        }\n";
  html += "        h1 {\n";
  html += "            color: #333;\n";
  html += "            text-align: center;\n";
  html += "        }\n";
  html += "        .section {\n";
  html += "            background-color: white;\n";
  html += "            border-radius: 8px;\n";
  html += "            padding: 20px;\n";
  html += "            margin-bottom: 20px;\n";
  html += "            box-shadow: 0 2px 4px rgba(0,0,0,0.1);\n";
  html += "        }\n";
  html += "        button {\n";
  html += "            background-color: #4CAF50;\n";
  html += "            color: white;\n";
  html += "            padding: 10px 20px;\n";
  html += "            border: none;\n";
  html += "            border-radius: 4px;\n";
  html += "            cursor: pointer;\n";
  html += "            margin: 5px;\n";
  html += "            font-size: 16px;\n";
  html += "        }\n";
  html += "        button:hover {\n";
  html += "            background-color: #45a049;\n";
  html += "        }\n";
  html += "        .piano-key {\n";
  html += "            background-color: white;\n";
  html += "            border: 1px solid #333;\n";
  html += "            width: 40px;\n";
  html += "            height: 120px;\n";
  html += "            margin: 2px;\n";
  html += "            display: inline-block;\n";
  html += "            vertical-align: top;\n";
  html += "            cursor: pointer;\n";
  html += "            transition: background-color 0.1s;\n";
  html += "        }\n";
  html += "        .piano-key:active {\n";
  html += "            background-color: #ddd;\n";
  html += "        }\n";
  html += "        input[type='text'], textarea {\n";
  html += "            width: 100%;\n";
  html += "            padding: 10px;\n";
  html += "            margin: 5px 0;\n";
  html += "            box-sizing: border-box;\n";
  html += "            border: 2px solid #ddd;\n";
  html += "            border-radius: 4px;\n";
  html += "            font-family: monospace;\n";
  html += "            font-size: 14px;\n";
  html += "        }\n";
  html += "        textarea {\n";
  html += "            resize: vertical;\n";
  html += "            min-height: 60px;\n";
  html += "        }\n";
  html += "        label {\n";
  html += "            font-weight: bold;\n";
  html += "            color: #555;\n";
  html += "            display: block;\n";
  html += "            margin-top: 10px;\n";
  html += "        }\n";
  html += "        .status {\n";
  html += "            padding: 10px;\n";
  html += "            margin: 10px 0;\n";
  html += "            border-radius: 4px;\n";
  html += "            display: none;\n";
  html += "        }\n";
  html += "        .status.success {\n";
  html += "            background-color: #d4edda;\n";
  html += "            color: #155724;\n";
  html += "            border: 1px solid #c3e6cb;\n";
  html += "            display: block;\n";
  html += "        }\n";
  html += "        .status.error {\n";
  html += "            background-color: #f8d7da;\n";
  html += "            color: #721c24;\n";
  html += "            border: 1px solid #f5c6cb;\n";
  html += "            display: block;\n";
  html += "        }\n";
  html += "        .example {\n";
  html += "            background-color: #f8f9fa;\n";
  html += "            padding: 10px;\n";
  html += "            border-left: 3px solid #4CAF50;\n";
  html += "            margin: 10px 0;\n";
  html += "            font-family: monospace;\n";
  html += "            font-size: 12px;\n";
  html += "        }\n";
  html += "        h3 {\n";
  html += "            color: #555;\n";
  html += "            border-bottom: 2px solid #4CAF50;\n";
  html += "            padding-bottom: 5px;\n";
  html += "        }\n";
  html += "    </style>\n";
  html += "</head>\n";
  html += "<body>\n";
  html += "    <h1>🎵 ESP32 MIDI播放器 🎵</h1>\n";
  html += "    \n";
  html += "    <div class='section'>\n";
  html += "        <h3>📝 自定义旋律编辑器</h3>\n";
  html += "        \n";
  html += "        <label for='notes'>音符序列 (MIDI音符号 0-127):</label>\n";
  html += "        <textarea id='notes' placeholder='例如: 60, 62, 64, 65, 67, 69, 71, 72'>60, 60, 67, 67, 69, 69, 67</textarea>\n";
  html += "        <div class='example'>\n";
  html += "            示例: C4=60, D4=62, E4=64, F4=65, G4=67, A4=69, B4=71, C5=72\n";
  html += "        </div>\n";
  html += "        \n";
  html += "        <label for='durations'>时长序列 (毫秒):</label>\n";
  html += "        <textarea id='durations' placeholder='例如: 500, 500, 500, 500, 500, 500, 1000'>500, 500, 500, 500, 500, 500, 1000</textarea>\n";
  html += "        <div class='example'>\n";
  html += "            示例: 500=半拍, 1000=一拍, 2000=两拍\n";
  html += "        </div>\n";
  html += "        \n";
  html += "        <label for='length'>音符数量:</label>\n";
  html += "        <input type='text' id='length' placeholder='例如: 7' value='7'>\n";
  html += "        \n";
  html += "        <div style='margin-top: 20px;'>\n";
  html += "            <button onclick='setCustomMelody()'>💾 保存旋律</button>\n";
  html += "            <button onclick='playCustomMelody()'>▶️ 播放自定义旋律</button>\n";
  html += "            <button onclick='loadExample(\"twinkle\")'>⭐ 加载小星星</button>\n";
  html += "            <button onclick='loadExample(\"birthday\")'>🎂 加载生日歌</button>\n";
  html += "            <button onclick='loadExample(\"ode\")'>🎼 加载欢乐颂</button>\n";
  html += "        </div>\n";
  html += "        \n";
  html += "        <div id='status' class='status'></div>\n";
  html += "    </div>\n";
  html += "    \n";
  html += "    <div class='section'>\n";
  html += "        <h3>🎹 控制面板</h3>\n";
  html += "        <button onclick='playDefaultMelody()'>▶️ 播放默认旋律</button>\n";
  html += "        <button onclick='stopAll()'>⏹️ 停止所有音符</button>\n";
  html += "    </div>\n";
  html += "    \n";
  html += "    <div class='section'>\n";
  html += "        <h3>🎹 虚拟钢琴</h3>\n";
  html += "        <div id='piano'></div>\n";
  html += "    </div>\n";
  html += "    \n";
  html += "    <script>\n";
  html += "        // 创建虚拟钢琴\n";
  html += "        function createPiano() {\n";
  html += "            const piano = document.getElementById('piano');\n";
  html += "            const whiteKeys = [60, 62, 64, 65, 67, 69, 71, 72, 74, 76, 77, 79, 81, 83, 84];\n";
  html += "            const noteNames = {\n";
  html += "                60: 'C4', 61: 'C#4', 62: 'D4', 63: 'D#4', 64: 'E4',\n";
  html += "                65: 'F4', 66: 'F#4', 67: 'G4', 68: 'G#4', 69: 'A4',\n";
  html += "                70: 'A#4', 71: 'B4', 72: 'C5', 73: 'C#5', 74: 'D5',\n";
  html += "                75: 'D#5', 76: 'E5', 77: 'F5', 78: 'F#5', 79: 'G5',\n";
  html += "                80: 'G#5', 81: 'A5', 82: 'A#5', 83: 'B5', 84: 'C6'\n";
  html += "            };\n";
  html += "            \n";
  html += "            let htmlPiano = '<div style=\"position: relative; display: inline-block;\">';\n";
  html += "            \n";
  html += "            // 白键\n";
  html += "            whiteKeys.forEach(note => {\n";
  html += "                htmlPiano += '<div class=\"piano-key\" onmousedown=\"playNote(' + note + ')\" onmouseup=\"stopNote(' + note + ')\" title=\"' + (noteNames[note] || note) + '\"></div>';\n";
  html += "            });\n";
  html += "            \n";
  html += "            htmlPiano += '</div>';\n";
  html += "            piano.innerHTML = htmlPiano;\n";
  html += "        }\n";
  html += "        \n";
  html += "        // 示例旋律\n";
  html += "        const examples = {\n";
  html += "            twinkle: {\n";
  html += "                notes: '60, 60, 67, 67, 69, 69, 67, 65, 65, 64, 64, 62, 62, 60',\n";
  html += "                durations: '500, 500, 500, 500, 500, 500, 1000, 500, 500, 500, 500, 500, 500, 1000',\n";
  html += "                length: '14',\n";
  html += "                name: '小星星'\n";
  html += "            },\n";
  html += "            birthday: {\n";
  html += "                notes: '60, 60, 62, 60, 65, 64, 60, 60, 62, 60, 67, 65',\n";
  html += "                durations: '375, 125, 500, 500, 500, 1000, 375, 125, 500, 500, 500, 1000',\n";
  html += "                length: '12',\n";
  html += "                name: '生日快乐歌'\n";
  html += "            },\n";
  html += "            ode: {\n";
  html += "                notes: '64, 64, 65, 67, 67, 65, 64, 62, 60, 60, 62, 64, 64, 62, 62',\n";
  html += "                durations: '500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 750, 250, 1000',\n";
  html += "                length: '15',\n";
  html += "                name: '欢乐颂'\n";
  html += "            }\n";
  html += "        };\n";
  html += "        \n";
  html += "        // 加载示例\n";
  html += "        function loadExample(name) {\n";
  html += "            if (examples[name]) {\n";
  html += "                document.getElementById('notes').value = examples[name].notes;\n";
  html += "                document.getElementById('durations').value = examples[name].durations;\n";
  html += "                document.getElementById('length').value = examples[name].length;\n";
  html += "                showStatus('已加载 \"' + examples[name].name + '\" 示例', 'success');\n";
  html += "            }\n";
  html += "        }\n";
  html += "        \n";
  html += "        // 显示状态信息\n";
  html += "        function showStatus(message, type) {\n";
  html += "            const status = document.getElementById('status');\n";
  html += "            status.textContent = message;\n";
  html += "            status.className = 'status ' + type;\n";
  html += "            setTimeout(() => {\n";
  html += "                status.className = 'status';\n";
  html += "            }, 3000);\n";
  html += "        }\n";
  html += "        \n";
  html += "        // 设置自定义旋律\n";
  html += "        function setCustomMelody() {\n";
  html += "            const notes = document.getElementById('notes').value;\n";
  html += "            const durations = document.getElementById('durations').value;\n";
  html += "            const length = document.getElementById('length').value;\n";
  html += "            \n";
  html += "            if (!notes || !durations || !length) {\n";
  html += "                showStatus('请填写所有字段', 'error');\n";
  html += "                return;\n";
  html += "            }\n";
  html += "            \n";
  html += "            const data = new FormData();\n";
  html += "            data.append('notes', notes);\n";
  html += "            data.append('durations', durations);\n";
  html += "            data.append('length', length);\n";
  html += "            \n";
  html += "            fetch('/setMelody', {\n";
  html += "                method: 'POST',\n";
  html += "                body: data\n";
  html += "            })\n";
  html += "            .then(response => response.text())\n";
  html += "            .then(result => {\n";
  html += "                showStatus('旋律已保存: ' + result, 'success');\n";
  html += "            })\n";
  html += "            .catch(error => {\n";
  html += "                showStatus('保存失败: ' + error, 'error');\n";
  html += "            });\n";
  html += "        }\n";
  html += "        \n";
  html += "        // 播放自定义旋律\n";
  html += "        function playCustomMelody() {\n";
  html += "            fetch('/playCustom')\n";
  html += "            .then(response => response.text())\n";
  html += "            .then(result => {\n";
  html += "                showStatus(result, 'success');\n";
  html += "            })\n";
  html += "            .catch(error => {\n";
  html += "                showStatus('播放失败: ' + error, 'error');\n";
  html += "            });\n";
  html += "        }\n";
  html += "        \n";
  html += "        // 播放默认旋律\n";
  html += "        function playDefaultMelody() {\n";
  html += "            fetch('/play')\n";
  html += "            .then(response => response.text())\n";
  html += "            .then(result => {\n";
  html += "                showStatus(result, 'success');\n";
  html += "            });\n";
  html += "        }\n";
  html += "        \n";
  html += "        // 停止所有音符\n";
  html += "        function stopAll() {\n";
  html += "            fetch('/stop')\n";
  html += "            .then(response => response.text())\n";
  html += "            .then(result => {\n";
  html += "                showStatus(result, 'success');\n";
  html += "            });\n";
  html += "        }\n";
  html += "        \n";
  html += "        // 播放单个音符\n";
  html += "        function playNote(note) {\n";
  html += "            fetch('/play?note=' + note);\n";
  html += "        }\n";
  html += "        \n";
  html += "        // 停止单个音符\n";
  html += "        function stopNote(note) {\n";
  html += "            fetch('/stop?note=' + note);\n";
  html += "        }\n";
  html += "        \n";
  html += "        // 初始化\n";
  html += "        createPiano();\n";
  html += "    </script>\n";
  html += "</body>\n";
  html += "</html>\n";
  
  server.send(200, "text/html", html);
}

// Web服务器处理函数 - 播放控制
void handlePlay() {
  if(server.hasArg("note")) {
    int note = server.arg("note").toInt();
    if (note >= 0 && note <= 127) {
      noteOn(note, 80);
      server.send(200, "text/plain; charset=utf-8", "播放音符 " + String(note));
    } else {
      server.send(400, "text/plain; charset=utf-8", "无效的音符编号");
    }
  } else {
    if (melodyTaskHandle == NULL) {
      BaseType_t taskCreated = xTaskCreatePinnedToCore(
        melodyTask,
        "MelodyTask",
        4096,
        NULL,
        1,
        &melodyTaskHandle,
        1
      );
      if (taskCreated == pdPASS) {
        server.send(200, "text/plain; charset=utf-8", "启动默认旋律任务");
      } else {
        server.send(500, "text/plain; charset=utf-8", "无法创建旋律任务");
      }
    } else {
      server.send(200, "text/plain; charset=utf-8", "默认旋律任务已在运行");
    }
  }
}

// Web服务器处理函数 - 停止控制
void handleStop() {
  if(server.hasArg("note")) {
    int note = server.arg("note").toInt();
    if (note >= 0 && note <= 127) {
      noteOff(note);
      server.send(200, "text/plain; charset=utf-8", "停止音符 " + String(note));
    } else {
      server.send(400, "text/plain; charset=utf-8", "无效的音符编号");
    }
  } else {
    allNotesOff();
    server.send(200, "text/plain; charset=utf-8", "停止所有音符");
  }
}

// 处理设置自定义旋律请求
void handleSetMelody() {
  if (!server.hasArg("notes") || !server.hasArg("durations") || !server.hasArg("length")) {
    server.send(400, "text/plain; charset=utf-8", "缺少必要参数");
    return;
  }
  
  String notesStr = server.arg("notes");
  String durationsStr = server.arg("durations");
  String lengthStr = server.arg("length");
  
  std::vector<uint8_t> notes;
  std::vector<uint16_t> durations;
  
  if (!parseNoteArray(notesStr, notes)) {
    server.send(400, "text/plain; charset=utf-8", "音符格式错误");
    return;
  }
  
  if (!parseNumberArray(durationsStr, durations)) {
    server.send(400, "text/plain; charset=utf-8", "时长格式错误");
    return;
  }
  
  int length = lengthStr.toInt();
  if (length <= 0 || length > MAX_MELODY_LENGTH) {
    server.send(400, "text/plain; charset=utf-8", "长度无效（1-256）");
    return;
  }
  
  // 确保数组长度匹配
  if (notes.size() < length || durations.size() < length) {
    server.send(400, "text/plain; charset=utf-8", "数组长度不匹配");
    return;
  }
  
  // 保存自定义旋律
  if (xSemaphoreTake(melodyMutex, portMAX_DELAY) == pdTRUE) {
    customMelody.notes = notes;
    customMelody.durations = durations;
    customMelody.length = length;
    xSemaphoreGive(melodyMutex);
    
    Serial.printf("保存自定义旋律: %d个音符\n", length);
    server.send(200, "text/plain; charset=utf-8", "旋律已保存（" + String(length) + "个音符）");
  } else {
    server.send(500, "text/plain; charset=utf-8", "无法保存旋律");
  }
}

// 处理播放自定义旋律请求
void handlePlayCustom() {
  // 停止当前可能正在运行的自定义旋律任务
  if (customMelodyTaskHandle != NULL) {
    server.send(200, "text/plain; charset=utf-8", "自定义旋律任务已在运行");
    return;
  }
  
  // 创建新的自定义旋律任务
  BaseType_t taskCreated = xTaskCreatePinnedToCore(
    customMelodyTask,
    "CustomMelodyTask",
    4096,
    NULL,
    1,
    &customMelodyTaskHandle,
    1
  );
  
  if (taskCreated == pdPASS) {
    server.send(200, "text/plain; charset=utf-8", "开始播放自定义旋律");
  } else {
    server.send(500, "text/plain; charset=utf-8", "无法创建自定义旋律任务");
  }
}