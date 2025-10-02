#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleSubscriber : public rclcpp::Node 

{

public:
    SimpleSubscriber() : Node("Simple_Sublscriber") {
        this->subscriber_ = this->create_subscription<std_msgs::msg::String>("chatter",10,std::bind(&SimpleSubscriber::msgcallback,this,std::placeholders::_1)); //msgcallback called upon receiving a new message to the topic with one argument


    }
    void msgcallback(const std_msgs::msg::String &msg){ //this callback receives as input the message it received
        
        RCLCPP_INFO(this->get_logger(), msg.data.c_str());
    }

private:
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; 
 

};

int main(int argc, char* argv[]) {

    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}