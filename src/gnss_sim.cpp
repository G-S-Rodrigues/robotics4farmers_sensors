#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "r4f_msgs/msg/gnss_reading.hpp"

#include "gnss_compass_utils/read_gnss_compass.h"

class GnssNodeSim : public rclcpp::Node
{
public:
    GnssNodeSim(const rclcpp::NodeOptions &options)
        : Node("GnssNode", options)
    {
        this->declare_parameter<std::string>("data_dir", "Directory where the anpp file is located");
        this->get_parameter("data_dir", anpp_dir_);
        gnss_anpp_dir_ = anpp_dir_ + "_gnss.anpp";
        RCLCPP_INFO(this->get_logger(), "GNSS file directory: %s", gnss_anpp_dir_.c_str());

        anpp_file = fopen(gnss_anpp_dir_.c_str(), "rb");

        if (anpp_file == NULL)
        {
            printf("Error opening anpp file\n");
            exit(EXIT_FAILURE);
        }
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
        gnss_pub_ = this->create_publisher<r4f_msgs::msg::GnssReading>("gnss_reading", qos);

        an_decoder_initialise(&an_decoder);
        start_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
        // std::cout << "start_time_ms = " << start_time_ms << std::endl;
        start_packet_time_ms = 0;
        last_packet_time_ms = 0;

        // Start the camera loop in a separate thread
        gnss_thread_ = std::thread(&GnssNodeSim::gnss_loop, this);
    }
    ~GnssNodeSim()
    {
        if (gnss_thread_.joinable())
        {
            gnss_thread_.join();
        }
    }

private:
    std::thread gnss_thread_;
    std::string anpp_dir_;
    std::string gnss_anpp_dir_;

    an_decoder_t an_decoder;
    an_packet_t *an_packet;

    system_state_packet_t system_state_packet;
    unix_time_packet_t unix_time_packet;
    raw_sensors_packet_t raw_sensors_packet;

    int bytes_read;
    uint64_t start_time_ms;
    uint64_t start_packet_time_ms;
    uint64_t last_packet_time_ms;
    FILE *anpp_file;

    rclcpp::Publisher<r4f_msgs::msg::GnssReading>::SharedPtr gnss_pub_;
    std::mutex mtx_;

    void gnss_loop()
    {
        // count frames per second
        auto start_time = std::chrono::high_resolution_clock::now();
        int gnss_count = 0;

        while (rclcpp::ok())
        {
            publish_packet();
            gnss_count++;

            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

            if (elapsed_time >= 1)
            {
                RCLCPP_DEBUG(this->get_logger(), "Gnss publish per second: %d", gnss_count);
                gnss_count = 0;                                         // Reset the counter
                start_time = std::chrono::high_resolution_clock::now(); // Reset the timer
            }
        }
    }

    void publish_packet()
    {
        if ((bytes_read = fread(an_decoder_pointer(&an_decoder), sizeof(uint8_t), an_decoder_size(&an_decoder), anpp_file)) > 0)
        {
            /* increment the decode buffer length by the number of bytes received */
            an_decoder_increment(&an_decoder, bytes_read);

            // RCLCPP_DEBUG(this->get_logger(), "bytes_read: %d", bytes_read);

            while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
            {
                // RCLCPP_DEBUG(this->get_logger(), "an_packet->id: %d ", an_packet->id);

                if (an_packet->id == packet_id_system_state) /* system state packet */
                {
                    /* copy all the binary data into the typedef struct for the packet */
                    /* this allows easy access to all the different values             */
                    if (decode_system_state_packet(&system_state_packet, an_packet) == 0)
                    {   
                        auto gnssreading_msg_ = r4f_msgs::msg::GnssReading();

                        // RCLCPP_DEBUG(this->get_logger(), "decode message");
                        last_packet_time_ms = static_cast<uint64_t>(system_state_packet.unix_time_seconds) * 1000 + system_state_packet.microseconds / 1000;

                        gnssreading_msg_.timestamp.data = last_packet_time_ms;
                        gnssreading_msg_.height.data = system_state_packet.height;
                        gnssreading_msg_.latitude.data = system_state_packet.latitude;
                        gnssreading_msg_.longitude.data = system_state_packet.longitude;
                        for (int i = 0; i < 3; ++i)
                        {
                            gnssreading_msg_.orientation[i].data = system_state_packet.orientation[i];
                        }

                        {
                            std::lock_guard<std::mutex> lock(mtx_);
                            gnss_pub_->publish(gnssreading_msg_);
                        }
                        // std::cout << "last_packet_time_ms  system_state_packet= " << last_packet_time_ms << std::endl;
                        // printf("GNSS UNIX timestamp: %lu \n", last_packet_time_ms);
                    }
                }
                else
                {
                    // printf("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
                }
                if (start_packet_time_ms == 0)
                {
                    start_packet_time_ms = last_packet_time_ms;
                }

                /* If we have a valid timestamp, wait until enough time has passed since starting the replay before sending the packet */
                uint64_t current_time_millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                   std::chrono::system_clock::now().time_since_epoch())
                                                   .count();
                while ((last_packet_time_ms > 0) && ((current_time_millis - start_time_ms) < (last_packet_time_ms - start_packet_time_ms)))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    current_time_millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                                              std::chrono::system_clock::now().time_since_epoch())
                                              .count();
                }

                /* Ensure that you free the an_packet when your done with it or you will leak memory */
                an_packet_free(&an_packet);
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().arguments(std::vector<std::string>(argv, argv + argc));
    auto node = std::make_shared<GnssNodeSim>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}