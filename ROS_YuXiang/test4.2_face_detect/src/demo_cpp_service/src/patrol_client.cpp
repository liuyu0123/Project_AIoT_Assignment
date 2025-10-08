#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono>
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol;
using SetP = rcl_interfaces::srv::SetParameters;

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
            } });
    }

    std::shared_ptr<SetP::Response> call_set_parameters(rcl_interfaces::msg::Parameter &parameter)
    {
        // 1. 创建客户端等待服务上线
        auto param_client = this->create_client<SetP>("/turtle_controller/set_parameters");
        while (!param_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }
        // 2. 创建请求
        auto request = std::make_shared<SetP::Request>();
        request->parameters.push_back(parameter);
        // 3. 异步调用，等待并返回响应结果
        auto future = param_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();
        return response;
    }

    void update_server_param_k(double k)
    {
        // 1. 创建参数对象
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";
        // 2. 设置参数值
        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        param.value = param_value;
        // 3. 调用服务
        auto response = call_set_parameters(param);
        if (response != nullptr)
        {
            if (response->results.size() > 0)
            {
                for (auto result : response->results){
                    if (result.successful)
                    {
                        RCLCPP_INFO(this->get_logger(), "Set parameter k successful");
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Set parameter k failed: %s", result.reason.c_str());
                    }
                }
            }
        }else{
            RCLCPP_ERROR(this->get_logger(), "Set parameter k failed");
            return;
        }
    }


private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    node->update_server_param_k(1.5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
