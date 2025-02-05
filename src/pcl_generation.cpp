/*
This node will:
1. Subscribe to the /velodyne_packets topic
2. Decode the packets
3. Publish the point cloud to /velodyne_points
*/

#include <rclcpp/rclcpp.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>
#include <cstring> // For memcpy

class PclGeneration : public rclcpp::Node {
public:
    PclGeneration() : Node("pcl_generation") {
        // Subscribe to the Velodyne packets
        _pcl_subscriber = this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
            "/velodyne_packets", 10,
            std::bind(&PclGeneration::packet_callback, this, std::placeholders::_1)
        );

        // Publisher for the point cloud
        _pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10);
    }

private:

    void packet_callback(const velodyne_msgs::msg::VelodyneScan::SharedPtr msg) {
        // Create a PointCloud2 message
        auto pcl_msgs = sensor_msgs::msg::PointCloud2();
        pcl_msgs.header.stamp = msg->header.stamp;
        pcl_msgs.header.frame_id = "velodyne";

        // Define the PointCloud2 fields: x, y, z, intensity
        pcl_msgs.height = 1; // Unstructured point cloud
        pcl_msgs.is_dense = true; // Contains invalid points
        pcl_msgs.point_step = 16; // Size of a point in bytes (4 fields * 4 bytes each)
        pcl_msgs.is_bigendian = false;

        // Initialize the PointCloud2 fields
        sensor_msgs::msg::PointField field_x, field_y, field_z, field_intensity;

        field_x.name = "x";
        field_x.offset = 0;
        field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_x.count = 1;

        field_y.name = "y";
        field_y.offset = 4;
        field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_y.count = 1;

        field_z.name = "z";
        field_z.offset = 8;
        field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_z.count = 1;

        field_intensity.name = "intensity";
        field_intensity.offset = 12;
        field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field_intensity.count = 1;

        pcl_msgs.fields = {field_x, field_y, field_z, field_intensity};

        // Iterate through the packets and decode the points
        std::vector<float> points;
        for (const auto& packet : msg->packets) {
            decode_packet(packet, points);
        }

        // Fill the point cloud message with the decoded points
        pcl_msgs.width = points.size() / 4; // Number of points
        pcl_msgs.row_step = pcl_msgs.point_step * pcl_msgs.width; // Size of the point cloud in bytes
        pcl_msgs.data.resize(points.size() * sizeof(float));
        std::memcpy(pcl_msgs.data.data(), points.data(), pcl_msgs.data.size());

        // Publish the point cloud message
        _pcl_publisher->publish(pcl_msgs);
    }


    void decode_packet(const velodyne_msgs::msg::VelodynePacket& packet, std::vector<float>& points) {
        const uint8_t* data = packet.data.data();

        for (int block = 0; block < 12; ++block) {
            // Access azimuth angle (2 bytes)
            uint16_t azimuth = data[block * 100 + 2] | (data[block * 100 + 3] << 8);
            float azimuth_rad = azimuth * M_PI / 18000.0; // Convert to radians

            // Decode 32 laser returns in the block
            for (int laser = 0; laser < 32; ++laser) {
                uint16_t distance = data[block * 100 + 4 + laser * 3] |
                                    (data[block * 100 + 5 + laser * 3] << 8);
                uint8_t intensity = data[block * 100 + 6 + laser * 3];

                // Convert distance to meters and compute Cartesian coordinates
                float distance_m = distance * 0.002; // Convert to meters
                float x = distance_m * cos(azimuth_rad);
                float y = distance_m * sin(azimuth_rad);
                float z = distance_m; // For simplicity, assume flat ground

                // Add the decoded point to the points vector
                points.push_back(x);
                points.push_back(y);
                points.push_back(z);
                points.push_back(static_cast<float>(intensity)); // Intensity as float
            }
        }
    }

    rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr _pcl_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pcl_publisher;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclGeneration>());
    rclcpp::shutdown();
    return 0;
}
