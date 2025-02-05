/*
This is a cpp node to read and subcribe to the packets from the serial port
*/

#include <rclcpp/rclcpp.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

class VelodynePacketSubscriber : public rclcpp::Node {
public:
    VelodynePacketSubscriber() : Node("velodyne_packet_subscriber") {
        subscription_ = this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
            "/velodyne_packets", 10,
            std::bind(&VelodynePacketSubscriber::packet_callback, this, std::placeholders::_1)
        );
    }

private:
    void packet_callback(const velodyne_msgs::msg::VelodyneScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received %zu Velodyne packets", msg->packets.size());
    }

    rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelodynePacketSubscriber>());
    rclcpp::shutdown();
    return 0;
}
