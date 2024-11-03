#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <r4f_sensors/msg/rgbd.hpp>

class RealSenseNodeSim : public rclcpp::Node
{
public:
    RealSenseNodeSim(const rclcpp::NodeOptions &options)
        : Node("realsense_node", options), align_to_color(RS2_STREAM_COLOR)
    {
        this->declare_parameter<std::string>("data_dir", "Directory where the bag file is located");
        this->get_parameter("data_dir", bag_dir_);
        camera_bag_dir_ = bag_dir_ + "_camera_f.bag";
        RCLCPP_DEBUG(this->get_logger(), "Camera file directory: %s", camera_bag_dir_.c_str());

        initialize_camera();

        // Define QoS settings
        // auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
        //                .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        //                .durability(rclcpp::DurabilityPolicy::Volatile)
        //                .liveliness(rclcpp::LivelinessPolicy::Automatic);

        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
        // rgbd_pub_ = this->create_publisher<r4f_sensors::msg::RGBD>("rgbd", 10);
        rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("color_image", qos);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image", qos);
        timestamp_pub_ = this->create_publisher<std_msgs::msg::UInt64>("timestamp_image", qos);

        // ignore the first 5 frames
        for (int i = 0; i < 5; ++i)
        {
            pipe_.wait_for_frames();
        }

        // Start the camera loop in a separate thread
        camera_thread_ = std::thread(&RealSenseNodeSim::camera_loop, this);
    }
    ~RealSenseNodeSim()
    {
        if (camera_thread_.joinable())
        {
            camera_thread_.join();
        }
    }

private:
    void initialize_camera()
    {
        // Configure the pipeline to stream from the bag file
        cfg_.enable_device_from_file(camera_bag_dir_, true); // whether or not to repeat the playback of the bag file once it reaches the end.
        // cfg_.enable_device_from_file("/home/guilh/data_tese/camera/vinha-09-04/row1.bag", true);
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

    void camera_loop()
    {
        // count frames per second
        auto start_time = std::chrono::high_resolution_clock::now();
        int frame_count = 0;

        while (rclcpp::ok())
        {
            publish_frames();
            frame_count++;

            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

            if (elapsed_time >= 1)
            {
                RCLCPP_DEBUG(this->get_logger(), "Frames per second: %d", frame_count);
                frame_count = 0;                                        // Reset the counter
                start_time = std::chrono::high_resolution_clock::now(); // Reset the timer
            }
        }
    }

    void publish_frames()
    {
        try
        {
            rs2::frameset frameset = pipe_.wait_for_frames();
            frameset = align_to_color.process(frameset);
            rs2::depth_frame depth_frame = frameset.get_depth_frame();
            rs2::video_frame color_frame = frameset.get_color_frame();

            cv::Mat color_image(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame.get_data());
            // cv::Mat color_image_rgb;
            // cv::cvtColor(color_image, color_image_rgb, cv::COLOR_BGR2RGB);
            cv::Mat depth_image(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16UC1, (void *)depth_frame.get_data());

            auto depth_msg = sensor_msgs::msg::Image();
            auto rgb_msg = sensor_msgs::msg::Image();
            auto timestamp_msg = std_msgs::msg::UInt64();

            // rgbd_msg.header.stamp = this->now();
            timestamp_msg.data = frameset.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
            // timestamp_msg.data = static_cast<uint64_t>(frameset.get_frame_metadata(RS2_FRAME_METADATA_TIME_OF_ARRIVAL));
            rgb_msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", color_image).toImageMsg();
            depth_msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth_image).toImageMsg();

            {
                std::lock_guard<std::mutex> lock(mtx_);
                timestamp_pub_->publish(timestamp_msg);
                rgb_pub_->publish(rgb_msg);
                depth_pub_->publish(depth_msg);
            }
        }
        catch (const rs2::error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in publish_frames: %s", e.what());
        }
    }

    std::string camera_bag_dir_;
    std::string bag_dir_;

    std::thread camera_thread_;

    rs2::pipeline pipe_;
    rs2::config cfg_;
    rs2::align align_to_color;
    rs2::pipeline_profile profile_;

    // rclcpp::Publisher<r4f_sensors::msg::RGBD>::SharedPtr rgbd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestamp_pub_;
    std::mutex mtx_;
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