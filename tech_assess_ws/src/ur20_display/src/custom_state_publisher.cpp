#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <numbers>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <geometry_msgs/msg/pose.hpp>

class StatePublisher : public rclcpp::Node {

    public:
     StatePublisher() : Node("state_publisher") {
        // multi-thread approach to run tf callback and joint state callbacks concurrently.
        cb_group1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        visual_tools_ = std::make_shared<rviz_visual_tools::RvizVisualTools>(
            "base",  // the world frame in our case
            "/rviz_visual_tools", this);
            //shared_from_this() );
        
        visual_tools_->loadMarkerPub(true); //wait for rviz
        visual_tools_->enableBatchPublishing();
        
        joint_angles = declare_parameter<std::vector<double>>("joint_angles",{0.0 , -0.5 , 0.5 , -1.5 ,0.0 , 0.0}); // set a default configuration in case yaml is not loaded.
        
        joint_goal = {0.0 , -0.4, 1.0 , -1.0 , 0.5, 0.0 }; // random joint target. Can be changed as long as the joint limits are respected.

        for (int i=0; i<joint_angles.size(); i++) {

          diffs.push_back(joint_angles[i]-joint_goal[i]); //calculating the difference between target joints and initial joints.
        }
        
        for (auto diff:diffs) {
          amp.push_back(std::abs(diff)); // taking the absolute of the previous differences to create the amplitude vector of each joint oscillation.
        }
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        joint_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        timer1 = create_wall_timer(std::chrono::milliseconds(10),std::bind(&StatePublisher::joint_config_callback,this),cb_group1_); // publish the robot configuration at 100Hz
        timer2 = create_wall_timer(std::chrono::milliseconds(11),std::bind(&StatePublisher::tf_callback,this),cb_group2_); // calculate TF transformations every 90 Hz (slight delay to solve race condition) 

     }


    private:
    std::shared_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_pub;
    rclcpp::TimerBase::SharedPtr timer1;
    rclcpp::TimerBase::SharedPtr timer2;
    std::vector<double> joint_angles ;
    std::vector<double> joint_goal, diffs, amp, joint_pos ;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::CallbackGroup::SharedPtr cb_group1_;
    rclcpp::CallbackGroup::SharedPtr cb_group2_;

    int counter = 0;
    

    void joint_config_callback() {
     
     
     //get_parameter("joint_angles", joint_angles);
     if (counter==0) {joint_pos = joint_angles;}

     else if (counter>0 && counter< 942) { 
      for (int i=0; i<joint_angles.size(); i++) {
       joint_pos[i] = joint_angles[i]+amp[i]*sin(counter*0.010); // 10 ms timestep matching the publishing frequency. oscillation frequency is omega = 1 r/s. 
      }                                                          // Thus, 1.5*T = 9.42s = 942 timesteps in my case. 
     }
     else { RCLCPP_INFO(get_logger(),"Trajectory for 1.5 periods completed. Motion stopped");}
     
     auto msg = sensor_msgs::msg::JointState();
     msg.header.stamp = get_clock()->now();
     msg.name = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
     msg.position = joint_pos;
     //RCLCPP_INFO_STREAM(get_logger(), "Joint angles: " << msg.position[0] << " " << msg.position[1] << " "<< msg.position[2] << " "<< msg.position[3] << " "<< msg.position[4] << " "<< msg.position[5]);
     joint_pub->publish(msg); 
     counter++; //comment this out if you do not want the oscillatory motion
     
    
    }

    void tf_callback() {
     
    //get transforms with tf

    // Tf_elbow_gripper
     std::string from_frame = "forearm_link";
     std::string to_frame = "gripper";

     geometry_msgs::msg::TransformStamped tf_msg_1 = tf_buffer_->lookupTransform(
                from_frame,
                to_frame,
                tf2::TimePointZero //get the latest available tf
            );

     Eigen::Isometry3d T_elbow_gripper = tf2::transformToEigen(tf_msg_1);

     // Tf_world_elbow

     from_frame = "base";
     to_frame = "forearm_link";

     geometry_msgs::msg::TransformStamped tf_msg_2 = tf_buffer_->lookupTransform(
                from_frame,
                to_frame,
                tf2::TimePointZero
            );

     Eigen::Isometry3d T_world_elbow  = tf2::transformToEigen(tf_msg_2);

     // Tf_world_gripper

     from_frame = "base";
     to_frame = "gripper";

     geometry_msgs::msg::TransformStamped tf_msg_3 = tf_buffer_->lookupTransform(
                from_frame,
                to_frame,
                tf2::TimePointZero
            );

     Eigen::Isometry3d T_world_gripper  = tf2::transformToEigen(tf_msg_3);

     // visualize the axis on the gripper 
     visual_tools_->publishAxis(T_world_gripper, 0.2, 0.01);

     // publish corresponding text close to the origin but a bit above

     Eigen::Vector3d text_position = T_world_gripper.translation() + Eigen::Vector3d(0, 0, 0.3);
     geometry_msgs::msg::Pose text_position_msg;
     text_position_msg.position.x = text_position.x();
     text_position_msg.position.y = text_position.y();
     text_position_msg.position.z = text_position.z();

     //visual_tools_->deleteAllMarkers(); //in case we want to delete previous markers

     visual_tools_->publishText(text_position_msg, "T_world_gripper",rviz_visual_tools::WHITE,rviz_visual_tools::XLARGE);

     visual_tools_->trigger();

     

     auto T = T_world_elbow*T_elbow_gripper ; // chain of transform to compare with the corresponding tf result
     
     if (T.rotation().isApprox(T_world_gripper.rotation(), 1e-6) && T.translation().isApprox(T_world_gripper.translation(), 1e-6) ) {

      RCLCPP_INFO(get_logger(),"Transformations match");

     }
     else{RCLCPP_INFO(get_logger(),"Transformations do not match");}
        
     //debugging process to check the produced transformation matrices

     Eigen::Vector3d translation_T = T.translation();  //composed trasformation matrix
     Eigen::Matrix3d rotation_T = T.rotation();
     Eigen::Vector3d translation_Tf = T_world_gripper.translation();  //composed trasformation matrix
     Eigen::Matrix3d rotation_Tf = T_world_gripper.rotation();

     RCLCPP_DEBUG(get_logger(),"flattened Tf rotation matrix is [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", rotation_Tf(0), rotation_Tf(1), rotation_Tf(2),rotation_Tf(3), rotation_Tf(4), rotation_Tf(5),rotation_Tf(6), rotation_Tf(7), rotation_Tf(8));
     RCLCPP_DEBUG(get_logger(),"flattened composed rotation matrix is [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", rotation_T(0), rotation_T(1), rotation_T(2),rotation_T(3), rotation_T(4), rotation_T(5),rotation_T(6), rotation_T(7), rotation_T(8)); 
     RCLCPP_DEBUG(get_logger(),"composed translation vector is [%.3f,%.3f,%.3f]", translation_T(0), translation_T(1), translation_T(2)); 
     RCLCPP_DEBUG(get_logger(),"Tf translation vector is [%.3f,%.3f,%.3f]", translation_Tf(0), translation_Tf(1), translation_Tf(2)); 
    
    }



};

int main(int argc, char** argv) {

  rclcpp::init(argc,argv);
  auto node = std::make_shared<StatePublisher>();

  //MultiThreadedExecutor
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);

  executor.spin();
  return 0;



}