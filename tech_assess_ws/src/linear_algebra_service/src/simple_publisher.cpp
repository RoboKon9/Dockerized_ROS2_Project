#include <rclcpp/rclcpp.hpp> //ros2 C++ client library to use ros2 functionalities
#include <std_msgs/msg/string.hpp> //from standard messages library, use string (for text messages)
#include <chrono>

class SimplePublisher : public rclcpp::Node 

{
public:
    
    SimplePublisher():Node("simple_publisher") {
        
        this->counter_ = 0;
        this->publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",10); //topic name hardcoded to be chatter and size of message queue is 10
        this->timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&SimplePublisher::timerCallback,this)); //could do with lamda too

        RCLCPP_INFO(get_logger(),"Publishing at 1 Hz");
    }
    void timerCallback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello from ROS2 - Counter" +  std::to_string(counter_++);
        this->publisher_->publish(message);
    }
private:
    unsigned int counter_; //helper variable to count the number of messages that are published within ros2 topic
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; //smart pointer to the rclcpp's Publisher object
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char * argv[]) {

    rclcpp::init(argc,argv);
    auto node =std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}