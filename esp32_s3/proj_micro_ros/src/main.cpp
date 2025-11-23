#include <micro_ros_platformio.h>
#include <WiFi.h> // 注意大小写：WiFi.h 不是 Wifi.h
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include "rcutils/logging.h"

rclc_executor_t executor;
rcl_publisher_t pub;
std_msgs__msg__Int32 msg;
rcl_timer_t timer;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void setup()
{
    IPAddress agent_ip;
    agent_ip.fromString("192.168.0.101");
    char WIFI_NAME[] = "TP-LINK_3371";
    char WIFI_PASSWORD[] = "qwe12345";
    set_microros_wifi_transports(WIFI_NAME, WIFI_PASSWORD, agent_ip, 8888);
    delay(2000);
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "esp32_node", "", &support);

    // rclc_publisher_init_default(
    //     &pub, &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    //     "esp32_counter");

    // rmw_qos_profile_t qos = rmw_qos_profile_default;
    // qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    // rclc_publisher_init(
    //     &pub, &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    //     "esp32_counter", &qos);

    rclc_publisher_init_best_effort(&pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32_counter");

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50),
                            [](rcl_timer_t *, int64_t)
                            {
                                msg.data++;
                                RCUTILS_LOG_INFO("pub %ld", msg.data);
                                rcl_publish(&pub, &msg, NULL);
                            });

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
}

void loop()
{
    rclc_executor_spin(&executor);
    
    // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}