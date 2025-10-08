#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono>

using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol;

class PatrolClient : public rclcpp::Node
{ 
    public:
    PatrolClient() : Node("patrol_client")
    {
        patrol_client_ = this->create_client<Patrol>("patrol");
        timer_ = this->create_wall_timer(10s, std::bind(&PatrolClient::timer_callback, this));
        srand(time(NULL));
    }

    void timer_callback()
    {
        // 1. 等待服务器启动
        while (!patrol_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }
        // 2. 创建请求
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = rand() % 15;
        request->target_y = rand() % 15;
        RCLCPP_INFO(this->get_logger(), "Sending request to patrol service: (%f, %f)", request->target_x, request->target_y);
        // 3. 发送请求
        patrol_client_->async_send_request(request, [&](rclcpp::Client<Patrol>::SharedFuture future) -> void
        {
            auto result = future.get();
            if (result->result == Patrol::Response::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Result from patrol service: success");
            }else if (result->result == Patrol::Response::FAIL){
                RCLCPP_INFO(this->get_logger(), "Result from patrol service: fail");
            }
        });
    }

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
