#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <tech_assess_msgs/srv/least_squares.hpp>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"


class LeastSquaresClient : public rclcpp::Node 
{

    public:

    LeastSquaresClient( std::shared_ptr<tech_assess_msgs::srv::LeastSquares::Request> request) : Node("least_squares_client") 
    {
        // create publisher to topic "thread_activation"
        pub = create_publisher<geometry_msgs::msg::Vector3>("thread_activation",10);

        // create client for least squares service
        client = create_client<tech_assess_msgs::srv::LeastSquares>("least_squares");

        //wait for service to become available. Checking every 1 second.
        while (!client->wait_for_service(std::chrono::seconds(1))) 
        {
            
            RCLCPP_INFO(get_logger(),"Waiting for Service...");
            
            if (!rclcpp::ok()) {

                RCLCPP_ERROR(get_logger(),"Interrupted while waiting for service. Terminating the node...");
                return;

            }
        }
            
        auto result_future =  client->async_send_request(request); //results is std::shared_future<SharedPtr<Response>>
        RCLCPP_INFO(get_logger(),"Waiting for the response");

        
        // Sequential arthitecture. Waiting for response and then continuing with calculation and publishing.

        if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future)== rclcpp::FutureReturnCode::SUCCESS) {
         auto result = result_future.get();
         RCLCPP_INFO(get_logger(),"Response received from the server");
         //transform the response to Eigen form 
         Eigen::Matrix3d received_rotation_matrix;
         Eigen::Vector3d received_displacement_vector;
         Eigen::Vector3d received_transformed_vector ;
         for (int i=0; i<3; i++){
             received_rotation_matrix(i,0)  = result->rot_mat_r[i].x ;
             received_rotation_matrix(i,1)  = result->rot_mat_r[i].y ;
             received_rotation_matrix(i,2)  = result->rot_mat_r[i].z ;
            }

         received_displacement_vector(0) = result->d_r.x;
         received_displacement_vector(1) = result->d_r.y;
         received_displacement_vector(2) = result->d_r.z;

         received_transformed_vector(0) = result->x_t.x;
         received_transformed_vector(1) = result->x_t.y;
         received_transformed_vector(2) = result->x_t.z;

        // Find the vector before transformation

         Eigen::Vector3d x = received_rotation_matrix.transpose()*received_transformed_vector - received_rotation_matrix.transpose()*received_displacement_vector;
         RCLCPP_INFO(get_logger(),"Initial Vector Found");

        // Transform it back to geometry_msgs/Vector3 form ti publish it
         auto pub_msg = geometry_msgs::msg::Vector3();
         pub_msg.x = x(0); 
         pub_msg.y = x(1); 
         pub_msg.z = x(2);

         RCLCPP_INFO(get_logger(),"The initial vector was: [%.3f , %.3f , %.3f]  ",x(0),x(1),x(2));
         pub->publish(pub_msg); }
            


    }
    private:

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub;
    rclcpp::Client<tech_assess_msgs::srv::LeastSquares>::SharedPtr client ;


};

std::shared_ptr<tech_assess_msgs::srv::LeastSquares::Request> load_request_from_yaml(const std::string & yaml_path);

int main(int argc,char* argv[]) {
    rclcpp::init(argc,argv);

    std::string path = ament_index_cpp::get_package_share_directory("linear_algebra_service") + "/config/request.yaml";
    //std::string path = "/home/progressive_robotics/dockerized_ros2_project/tech_assess_ws/src/linear_algebra_service/src/request.yaml";
   

    // reading request from yaml file and given as argument
    auto request = load_request_from_yaml(path) ;


    auto node = std::make_shared<LeastSquaresClient>(request) ;
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}



std::shared_ptr<tech_assess_msgs::srv::LeastSquares::Request> load_request_from_yaml(const std::string & yaml_path)
{
    auto request = std::make_shared<tech_assess_msgs::srv::LeastSquares::Request>();

    YAML::Node config = YAML::LoadFile(yaml_path);

    auto A = config["alpha"];
    auto b = config["beta"];

    for (int i = 0; i < 3; i++) {
        request->alpha[i].x = A[i]["x"].as<double>();
        request->alpha[i].y = A[i]["y"].as<double>();
        request->alpha[i].z = A[i]["z"].as<double>();
    }
    request->beta.x = b["x"].as<double>();
    request->beta.y = b["y"].as<double>();
    request->beta.z = b["z"].as<double>();

    return request;
}



