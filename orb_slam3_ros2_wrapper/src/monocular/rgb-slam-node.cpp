/**
 * @file rgb-slam-node.cpp
 * @brief Implementation of the RgbSlamNode Wrapper class.
 * @author Suchetan R S (rssuchetan@gmail.com) w/ edits by Nick Koenig (nickoenig37@gmail.com)
 */

// This is an edit made by Nick Koenig on the rgdb package but to use with rgb

#include "rgb-slam-node.hpp"
#include <opencv2/core/core.hpp>


namespace ORB_SLAM3_Wrapper
{
    RgbSlamNode::RgbSlamNode(const std::string &strVocFile,
                               const std::string &strSettingsFile,
                               ORB_SLAM3::System::eSensor sensor)
        : Node("ORB_SLAM3_RGB_ROS2") // This is the constructor name we follow in our yaml config file 
    {
        // ROS Subscribers
        // rgbSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "reg_camera/image_raw");
        // depthSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "camera/depth/image_raw");
        // syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *rgbSub_, *depthSub_);
        // syncApproximate_->registerCallback(&RgbSlamNode::RGBCallback, this);

        // To just use rgbSub_ only we create a direct subscription
        rgbSub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10, std::bind(&RgbSlamNode::RGBCallback, this, std::placeholders::_1));

        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("no_imu", 1000, std::bind(&RgbSlamNode::ImuCallback, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1000, std::bind(&RgbSlamNode::OdomCallback, this, std::placeholders::_1));
        // ROS Publishers
        mapDataPub_ = this->create_publisher<slam_msgs::msg::MapData>("map_data", 10);
        mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
        // Services
        getMapDataService_ = this->create_service<slam_msgs::srv::GetMap>("orb_slam3_get_map_data", std::bind(&RgbSlamNode::getMapServer, this,
                                                                                                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        // Timers
        //  mapDataTimer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RgbSlamNode::publishMapData, this));
        
        // TF
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        bool bUseViewer;
        this->declare_parameter("visualization", rclcpp::ParameterValue(true));
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("ros_visualization", rclcpp::ParameterValue(false));
        this->get_parameter("ros_visualization", rosViz_);

        this->declare_parameter("robot_base_frame", "base_link");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("robot_x", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_x", robot_x_);

        this->declare_parameter("robot_y", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_y", robot_y_);

        this->declare_parameter("no_odometry_mode", rclcpp::ParameterValue(false));
        this->get_parameter("no_odometry_mode", no_odometry_mode_);

        // Following are parameters for mapping pointclouds
        this->declare_parameter("map_data_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("map_data_publish_frequency", map_data_publish_frequency_);

        this->declare_parameter("landmark_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("landmark_publish_frequency", landmark_publish_frequency_);

        // Timers
        mapDataCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapDataTimer_ = this->create_wall_timer(std::chrono::milliseconds(map_data_publish_frequency_), std::bind(&RgbSlamNode::publishMapData, this), mapDataCallbackGroup_);
        if (rosViz_)
        {
            mapPointsCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            mapPointsTimer_ = this->create_wall_timer(std::chrono::milliseconds(landmark_publish_frequency_), std::bind(&RgbSlamNode::publishMapPointCloud, this), mapDataCallbackGroup_);
        }

        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(strVocFile, strSettingsFile,
                                                                            sensor, bUseViewer, rosViz_, robot_x_,
                                                                            robot_y_, global_frame_, odom_frame_id_, robot_base_frame_id_);
        
        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR END!");
    }

    RgbSlamNode::~RgbSlamNode()
    {
        rgbSub_.reset();
        // depthSub_.reset();
        imuSub_.reset();
        odomSub_.reset();
        interface_.reset();
        RCLCPP_INFO(this->get_logger(), "DESTRUCTOR!");
    }

    void RgbSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "ImuCallback");
        // push value to imu buffer.
        interface_->handleIMU(msgIMU);
    }

    void RgbSlamNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
    {
        if(!no_odometry_mode_)
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "OdomCallback");
            interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
        }
        // Commented because of spamming
        // else RCLCPP_WARN(this->get_logger(), "Odometry msg recorded but no odometry mode is true, set to false to use this odometry");
    }

    void RgbSlamNode::RGBCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB)
    {
        // RCLCPP_INFO(this->get_logger(), "Received an RGB image with timestamp: %ld", msgRGB->header.stamp.sec);
        Sophus::SE3f Tcw;
        if (interface_->trackRGB(msgRGB, Tcw))
        {            
            isTracked_ = true;
            if (no_odometry_mode_)
            {
                interface_->getDirectMapToRobotTF(msgRGB->header, tfMapOdom_);
            }
            tfBroadcaster_->sendTransform(tfMapOdom_);
            // if (rosViz_)
            // {
            //     publishMapPointCloud();
            // }
            ++frequency_tracker_count_; //added for pcl 
            // publishMapPointCloud();
            // std::thread(&RgbdSlamNode::publishMapPointCloud, this).detach();
        }
    }

    void RgbSlamNode::publishMapPointCloud()
    {
        // sensor_msgs::msg::PointCloud2 mapPCL;
        // interface_->getCurrentMapPoints(mapPCL);
        // mapPointsPub_->publish(mapPCL);

        // For pointcloud map viz
        if (isTracked_)
        {
            // Using high resolution clock to measure time
            auto start = std::chrono::high_resolution_clock::now();

            sensor_msgs::msg::PointCloud2 mapPCL;

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            interface_->getCurrentMapPoints(mapPCL);

            if(mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            mapPointsPub_->publish(mapPCL);
            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to publish map points: " << time_publish_map_points << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "=======================");


            // Calculate the time taken for each line

            // Print the time taken for each line
        }
    }

    void RgbSlamNode::publishMapData()
    {
        if (isTracked_)
        {   
            auto start = std::chrono::high_resolution_clock::now(); //added for pcl

            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing map data");

            RCLCPP_WARN_STREAM(this->get_logger(), "Current ORB-SLAM3 tracking frequency: " << frequency_tracker_count_ / std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - frequency_tracker_clock_).count() << " frames / sec");
            frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();
            frequency_tracker_count_ = 0;

            // publish the map data (current active keyframes etc)
            slam_msgs::msg::MapData mapDataMsg;
            interface_->mapDataToMsg(mapDataMsg, true, false);
            mapDataPub_->publish(mapDataMsg);

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_publishMapData = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to create mapdata: " << time_publishMapData << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "*************************");
        }
    }

    void RgbSlamNode::getMapServer(std::shared_ptr<rmw_request_id_t> request_header,
                                    std::shared_ptr<slam_msgs::srv::GetMap::Request> request,
                                    std::shared_ptr<slam_msgs::srv::GetMap::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "GetMap2 service called.");
        slam_msgs::msg::MapData mapDataMsg;
        interface_->mapDataToMsg(mapDataMsg, false, request->tracked_points, request->kf_id_for_landmarks);
        response->data = mapDataMsg;
    }
}
