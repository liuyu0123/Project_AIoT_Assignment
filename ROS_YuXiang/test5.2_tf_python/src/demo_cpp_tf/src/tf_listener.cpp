#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include <chrono>

using namespace std::chrono_literals;

class TfListener : public rclcpp::Node
{
public:
    TfListener() : Node("tf_listener")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
        timer_ = this->create_wall_timer(5s, std::bind(&TfListener::getTransform, this));
    }

    void getTransform()
    {
        try
        {
            const auto transform = tf_buffer_->lookupTransform("base_link", "target_point",
                                                               this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0f));
            const auto &translation = transform.transform.translation;
            const auto &rotation = transform.transform.rotation;
            double yaw, pitch, roll;
            tf2::getEulerYPR(rotation, yaw, pitch, roll);
            RCLCPP_INFO(this->get_logger(), "Translation: x: %f, y: %f, z: %f", translation.x, translation.y, translation.z);
            RCLCPP_INFO(this->get_logger(), "Rotation: yaw: %f, pitch: %f, roll: %f", yaw, pitch, roll);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfListener>());
    rclcpp::shutdown();
    return 0;
}
