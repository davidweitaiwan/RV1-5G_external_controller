#include "header.h"
#include "vehicle_interfaces/params.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Params> params;
    {
        auto ts = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        auto tmp = std::make_shared<Params>("tmp_external_controller_" + std::to_string(ts) + "_params_node");
        params = std::make_shared<Params>(tmp->nodeName + "_params_node");
    }
    auto controller = std::make_shared<Controller>(params);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(controller);
    std::cout << "[main] Starting controller" << std::endl;
    executor->spin();// Blocked.
    std::cout << "[main] Controller stopped" << std::endl;

    rclcpp::shutdown();
    return 0;
}