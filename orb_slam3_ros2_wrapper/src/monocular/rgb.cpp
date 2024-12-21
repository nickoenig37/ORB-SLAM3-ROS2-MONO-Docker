#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgb-slam-node.hpp"

// This is an edit made by Nick Koenig on the rgdb package but to use with rgb

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<ORB_SLAM3_Wrapper::RgbSlamNode>(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR);
    std::cout << "============================ " << std::endl;

    // Changed to multi-threaded executor for having the map as its own thread
    // rclcpp::spin(node);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();

    rclcpp::shutdown();

    return 0;
}
