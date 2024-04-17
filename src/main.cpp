#include "header.h"
#include "vehicle_interfaces/params.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("controlserver_params_node");
    auto controller = std::make_shared<Controller>(params);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(controller);
    std::cout << "[main] Starting controller" << std::endl;
    executor->spin();// Blocked.
    std::cout << "[main] Controller stopped" << std::endl;

    rclcpp::shutdown();
    return 0;
}