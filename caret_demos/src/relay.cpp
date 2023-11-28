#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#define QOS_HISTORY_SIZE 10

class Relay : public rclcpp::Node
{
  public:
    Relay(std::string input_topic, std::string output_topic)
    : Node("relay_node"), input_topic_(input_topic), output_topic_(output_topic)
    {
      pub_ = this->create_generic_publisher(
        output_topic, "sensor_msgs/msg/Image", QOS_HISTORY_SIZE);
      sub_ = this->create_generic_subscription(
        input_topic, "sensor_msgs/msg/Image", QOS_HISTORY_SIZE,
      [&](std::shared_ptr<rclcpp::SerializedMessage> msg){
          pub_->publish(*msg);
      });
    }

  private:
    std::string input_topic_;
    std::string output_topic_;
    rclcpp::GenericSubscription::SharedPtr sub_;
    rclcpp::GenericPublisher::SharedPtr pub_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relay>("/topic3", "/topic4"));
  rclcpp::shutdown();
  return 0;
}