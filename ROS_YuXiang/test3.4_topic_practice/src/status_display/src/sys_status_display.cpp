// #include <QApplication>
// #include <QLabel>
// #include <QString>
// #include "rclcpp/rclcpp.hpp"
// #include "status_interfaces/msg/system_status.hpp"

#include <QApplication>
#include <QLabel>
#include <QString>
#include <sstream>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp"

using status_interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node
{
public:
    SysStatusDisplay() : Node("sys_status_display")
    {
        subscription_ = this->create_subscription<SystemStatus>(
            "sys_status", 10, [&](const SystemStatus::SharedPtr msg) -> void {
                label_ -> setText(get_qstr_from_msg(msg));
            });

        label_ = new QLabel(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        label_->show();
    }
    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg)
    {
        std::stringstream show_str;
        show_str
            << "============System Status Visualization Tool===========" << "\t\n"
            << "  Time: \t" << msg->stamp.sec << "\ts\n"
            << "  UserName: \t" << msg->host_name << "\t\n"
            << "  CPU: \t" << msg->cpu_percent << "\t%\n"
            << "  RAM_USAGE: \t" << msg->memory_percent << "\t%\n"
            << "  RAM_TOTAL: \t" << msg->memory_total << "\tMB\n"
            << "  RAM_REST: \t" << msg->memory_available << "\tMB\n"
            << "  Net_Sent: \t" << msg->net_sent << "\tMB\n"
            << "  Net_Recv: \t" << msg->net_recv << "\tMB\n"
            << "=======================================================";
        return QString::fromStdString(show_str.str());
    }

private:
    rclcpp::Subscription<SystemStatus>::SharedPtr subscription_;
    QLabel *label_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread([&]() -> void
                            { rclcpp::spin(node); });
    spin_thread.detach();
    app.exec();
    rclcpp::shutdown();
    return 0;
}