#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using std::placeholders::_1;

class FakeAmclNode : public rclcpp::Node
{
public:

    FakeAmclNode()
    : Node("fake_amcl_node")
    {

        this->declare_parameter<std::string>(
            "input_topic",
            "/odometry/filtered");

        this->declare_parameter<std::string>(
            "output_topic",
            "/amcl_pose");

        this->declare_parameter<std::string>(
            "frame_id",
            "map");

        input_topic_ =
            this->get_parameter("input_topic").as_string();

        output_topic_ =
            this->get_parameter("output_topic").as_string();

        frame_id_ =
            this->get_parameter("frame_id").as_string();

        publisher_ =
            this->create_publisher<
                geometry_msgs::msg::PoseWithCovarianceStamped>(
                    output_topic_,
                    10);

        subscriber_ =
            this->create_subscription<
                nav_msgs::msg::Odometry>(
                    input_topic_,
                    20,
                    std::bind(
                        &FakeAmclNode::callback,
                        this,
                        _1));

        RCLCPP_INFO(
            this->get_logger(),
            "Fake AMCL started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Subscribe : %s",
            input_topic_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Publish   : %s",
            output_topic_.c_str());
    }

private:

    void callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        geometry_msgs::msg::PoseWithCovarianceStamped pose;

        pose.header.stamp =
            msg->header.stamp;

        pose.header.frame_id =
            frame_id_;

        pose.pose =
            msg->pose;

        publisher_->publish(pose);
    }

    std::string input_topic_;

    std::string output_topic_;

    std::string frame_id_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
            publisher_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr
            subscriber_;
};

int main(
    int argc,
    char **argv)
{

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<FakeAmclNode>());

    rclcpp::shutdown();

    return 0;
}