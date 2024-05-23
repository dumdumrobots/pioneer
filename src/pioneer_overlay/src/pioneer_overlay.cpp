#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>

class PioneerOverlayNode : public rclcpp::Node
{
public:
    PioneerOverlayNode()
        : Node("pioneer_overlay_node")
    {
        publisher_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("pioneer_overlay_text", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "input_text", 10, std::bind(&PioneerOverlayNode::handle_input_text, this, std::placeholders::_1));
    }

private:
    void handle_input_text(const std_msgs::msg::String::SharedPtr msg)
    {
        auto overlay_message = rviz_2d_overlay_msgs::msg::OverlayText();
        overlay_message.text = msg->data;
        overlay_message.width = 800;
        overlay_message.height = 100;
        overlay_message.text_size = 24;
        overlay_message.line_width = 2.0;
        overlay_message.font = "DejaVu Sans Mono";
        overlay_message.fg_color.r = 1.0;
        overlay_message.fg_color.g = 1.0;
        overlay_message.fg_color.b = 1.0;
        overlay_message.fg_color.a = 1.0;
        overlay_message.bg_color.r = 0.0;
        overlay_message.bg_color.g = 0.0;
        overlay_message.bg_color.b = 0.0;
        overlay_message.bg_color.a = 0.5;

        RCLCPP_INFO(this->get_logger(), "Publishing overlay text: '%s'", overlay_message.text.c_str());
        publisher_->publish(overlay_message);
    }

    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PioneerOverlayNode>());
    rclcpp::shutdown();
    return 0;
}
