#include <micro_ros_platformio.h>
#include <WiFi.h> // 注意大小写：WiFi.h 不是 Wifi.h
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include "rcutils/logging.h"

rclc_executor_t executor;
rcl_publisher_t pub;
rcl_publisher_t pub2;
std_msgs__msg__Int32 msg;
std_msgs__msg__String msg_str;
rcl_timer_t timer;
rcl_timer_t timer2;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rcl_node_t node2;


// 将回调函数独立封装
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    msg.data++;
    rcl_publish(&pub, &msg, NULL);
}

void string_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // msg_str.data = "hello world"; //不能这样写
    rosidl_runtime_c__String__assign(&msg_str.data, "hello world");
    rcl_publish(&pub2, &msg_str, NULL);
}

void config_wifi()
{
    // 配置wifi，用于和上位机通信
    IPAddress agent_ip;
    agent_ip.fromString("192.168.0.101");
    char WIFI_NAME[] = "TP-LINK_3371";
    char WIFI_PASSWORD[] = "qwe12345";
    set_microros_wifi_transports(WIFI_NAME, WIFI_PASSWORD, agent_ip, 8888);
    delay(2000); // 给一个延时，等待wifi连接成功
}

void setup()
{
    config_wifi();

    // 初始化 micro-ros 环境
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator); //argc=0, argv=NULL

    // 创建节点
    rclc_node_init_default(&node, "esp32_node", "", &support); //esp32_node是节点（node）名称
    // rclc_node_init_default(&node2, "esp32_str_node", "", &support);

    // 初始化发布者
    // 参数：1. publisher， 2. node， 3. 消息类型， 4. topic名称
    rclc_publisher_init_best_effort(&pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32_counter");
    rclc_publisher_init_best_effort(&pub2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/esp32_string");

    // 初始化定时器+回调函数，定时器与回调永远一对一，而回调可以对应多个 publisher
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(500), timer_callback);
    rclc_timer_init_default(&timer2, &support, RCL_MS_TO_NS(1000), string_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_timer(&executor, &timer2);
}

void loop()
{
    rclc_executor_spin(&executor);
}