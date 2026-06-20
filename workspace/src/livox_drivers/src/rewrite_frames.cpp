#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


class FrameIdRewriterNode : public rclcpp::Node {
private:
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imu_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_publishers_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> imu_subscriptions_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subscriptions_;

public:
    FrameIdRewriterNode() : Node("frame_id_rewriter") {
        const std::vector<std::string> default_input_topics{};
        const std::vector<std::string> default_frame_names{};
        const std::vector<std::string> default_output_topics{};

        const auto input_topics = this->declare_parameter<std::vector<std::string>>("input_topics", default_input_topics);
        auto frame_names = this->declare_parameter<std::vector<std::string>>("frame_names", default_frame_names);
        // namespacing: единый префикс робота (напр. "robot1/") ко всем выходным фреймам,
        // чтобы lidar_70/imu_70 совпали с lio_sam lidarFrame и статиком mount у 2+ роботов.
        const auto frame_prefix = this->declare_parameter<std::string>("frame_prefix", "");
        for (auto &fn : frame_names) {
            fn = frame_prefix + fn;
        }
        const auto output_topics = this->declare_parameter<std::vector<std::string>>("output_topics", default_output_topics);
        const auto queue_size = this->declare_parameter<int>("queue_size", 10);

        if (input_topics.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'input_topics' is empty.");
            throw std::runtime_error("Parameter 'input_topics' is empty");
        }

        if (frame_names.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'frame_names' is empty.");
            throw std::runtime_error("Parameter 'frame_names' is empty");
        }

        if (input_topics.size() != frame_names.size()) {
            RCLCPP_FATAL(
                this->get_logger(),
                "Parameters size mismatch: input_topics=%zu, frame_names=%zu",
                input_topics.size(),
                frame_names.size());
            throw std::runtime_error("input_topics and frame_names size mismatch");
        }

        if (!output_topics.empty() && output_topics.size() != input_topics.size()) {
            RCLCPP_FATAL(
                this->get_logger(),
                "Parameter 'output_topics' must have same size as 'input_topics'. "
                "output_topics=%zu, input_topics=%zu",
                output_topics.size(),
                input_topics.size());
            throw std::runtime_error("output_topics and input_topics size mismatch");
        }

        pointcloud_publishers_.reserve(input_topics.size());
        pointcloud_subscriptions_.reserve(input_topics.size());
        imu_publishers_.reserve(input_topics.size());
        imu_subscriptions_.reserve(input_topics.size());

        for (size_t i = 0; i < input_topics.size(); ++i) {
            const std::string &input_topic = input_topics[i];
            const std::string &output_topic = output_topics[i];
            const std::string &frame_id = frame_names[i];

            if (input_topic.find("imu") != std::string::npos) {
                auto publisher =
                    this->create_publisher<sensor_msgs::msg::Imu>(
                        output_topic,
                        queue_size);

                auto callback =
                    [publisher, frame_id](sensor_msgs::msg::Imu::SharedPtr msg) {
                        msg->header.frame_id = frame_id;
                        publisher->publish(*msg);
                    };

                auto subscription = this->create_subscription<sensor_msgs::msg::Imu>(input_topic, queue_size, std::move(callback));

                imu_publishers_.push_back(publisher);
                imu_subscriptions_.push_back(subscription);
            }
            else {
                auto publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, queue_size);

                auto callback =
                    [publisher, frame_id](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                        msg->header.frame_id = frame_id;
                        publisher->publish(*msg);
                    };

                auto subscription =this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic, queue_size, std::move(callback));

                pointcloud_publishers_.push_back(publisher);
                pointcloud_subscriptions_.push_back(subscription);
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Rewriting frame_id: input=%s frame=%s output=%s",
                input_topic.c_str(),
                frame_id.c_str(),
                output_topic.c_str()
            );
        }
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameIdRewriterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
