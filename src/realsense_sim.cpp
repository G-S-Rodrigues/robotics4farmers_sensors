// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
#include <iostream>


#include "rclcpp/rclcpp.hpp"

#include <librealsense2/rs.hpp>

// #include "std_msgs/msg/string.hpp"

// #include <librealsense2/rs.hpp>
// #include "realsense2_camera_msgs/msg/extrinsics.hpp"
// #include "realsense2_camera_msgs/msg/rgbd.hpp"
// #include "realsense2_camera_msgs/msg/imu_info.hpp"

// using namespace std::chrono_literals;

// /* This example creates a subclass of Node and uses std::bind() to register a
//  * member function as a callback from the timer. */

// class RealsenseNodeSim : public rclcpp::Node
// {
// public:
//     RealsenseNodeSim()
//         : Node("RealSense"), count_(0)
//     {
//         publisher_rgbd = this->create_publisher<realsense2_camera_msgs::msg::RGBD>("RGBD", 10);
//         timer_ = this->create_wall_timer(
//             500ms, std::bind(&RealsenseSim::timer_callback, this));

//         // publisher_imu = this->create_publisher<std_msgs::msg::String>("topic", 10);
//         // timer_ = this->create_wall_timer(
//         //     500ms, std::bind(&MinimalPublisher::timer_callback, this));
//     }

// private:
//     // rs2::pipeline pipe;
//     // rs2::config cfg;
//     // std::string camera_bag_dir;
//     // rs2::align align_to_color;
//     // rs2::pipeline_profile profile;

//     void timer_callback()
//     {

//         auto message = std_msgs::msg::String();
//         message.data = "Hello, world! " + std::to_string(count_++);
//         RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//         publisher_rgbd->publish(message);
//     }
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_rgbd;
//     size_t count_;
// };

class RealSenseNodeSim : public rclcpp::Node
{
public:
    RealSenseNodeSim(const rclcpp::NodeOptions & options) 
        : Node("realsense_node", options), align_to_color(RS2_STREAM_COLOR)
    {
        this->declare_parameter<std::string>("bag_dir", "Directory where the bag file is located");
        this->get_parameter("bag_dir", bag_dir_);
        camera_bag_dir_ = bag_dir_ + "_camera_f.bag";
        RCLCPP_DEBUG(this->get_logger(), "Camera file directory: %s", camera_bag_dir_.c_str());

        initialize_camera();
    }

private:
    void initialize_camera()
    {
        // Configure the pipeline to stream from the bag file
        cfg_.enable_device_from_file(camera_bag_dir_, false);
        cfg_.enable_stream(RS2_STREAM_DEPTH);
        cfg_.enable_stream(RS2_STREAM_COLOR);

        // Start the pipeline
        profile_ = pipe_.start(cfg_);

        // Get the playback device and enable real-time mode
        rs2::device device = profile_.get_device();
        if (rs2::playback playback = device.as<rs2::playback>())
        {
            playback.set_real_time(true);
        }

        // Log the initialization status
        RCLCPP_INFO(this->get_logger(), "Initialized camera from bag file: %s", camera_bag_dir_.c_str());
    }

    std::string camera_bag_dir_;
    std::string bag_dir_;

    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::align align_to_color;
    rs2::pipeline_profile profile_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().arguments(std::vector<std::string>(argv, argv + argc));
    auto node = std::make_shared<RealSenseNodeSim>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}