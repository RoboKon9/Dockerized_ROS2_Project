#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <memory>
#include <tech_assess_msgs/srv/least_squares.hpp>
#include <ctime>
#include <geometry_msgs/msg/vector3.hpp>

class LeastSquaresService : public rclcpp:: Node {

    public: 
    LeastSquaresService() : Node("least_squares_service") {
        sub_ = create_subscription<geometry_msgs::msg::Vector3>("thread_activation",10,std::bind(& LeastSquaresService::thread_notification_callback,this,std::placeholders::_1));
        thread_ = std::thread(&LeastSquaresService::thread_function, this);
        service_ = create_service<tech_assess_msgs::srv::LeastSquares>("least_squares",std::bind(&LeastSquaresService::least_squares_callback,this,std::placeholders::_1,std::placeholders::_2))  ;    
        RCLCPP_INFO(get_logger(),"Least Squares Service has started...")  ;
    }

    
    
    
    private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_ ;
    rclcpp::Service<tech_assess_msgs::srv::LeastSquares>::SharedPtr service_ ;
    std::mutex mtx_;
    std::condition_variable c_v_;
    geometry_msgs::msg::Vector3 latest_msg_;
    bool msg_ready_ = false;
    std::thread thread_;

    void thread_notification_callback(const geometry_msgs::msg::Vector3 &msg) {
     std::lock_guard<std::mutex> lock(mtx_);
     latest_msg_ = msg;
     msg_ready_ = true;
     c_v_.notify_one();}

    void thread_function() {
     std::unique_lock<std::mutex> lock(mtx_);
        while (rclcpp::ok()) {
            c_v_.wait(lock, [this]() { return msg_ready_; });
            RCLCPP_INFO(get_logger(), "Received subscriber message is the vector: [%.3f %.3f %.3f]", latest_msg_.x, latest_msg_.y,latest_msg_.z);
            msg_ready_ = false;
        }
    }
    void least_squares_callback(const std::shared_ptr<tech_assess_msgs::srv::LeastSquares::Request> request, const std::shared_ptr<tech_assess_msgs::srv::LeastSquares::Response> result ) {
      try {  
        //take the msg from .srv and transform it in Eigen form 
        Eigen::MatrixXd A(3,3) ;
        for (int i=0; i<3; i++) {

           A(i,0) = request->alpha[i].x;
           A(i,1) = request->alpha[i].y;
           A(i,2) = request->alpha[i].z;

        }
        
        // same for vector b
        Eigen::Vector3d b;
        b(0) = request->beta.x;
        b(1) = request->beta.y;
        b(2) = request->beta.z;

        // Least Squares using Eigen's JacobiSVD
        Eigen::Vector3d X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) ;
        RCLCPP_INFO(get_logger(),"The Least-Squares result vector is: %f %f %f", X(0),X(1),X(2));

        // Transform the vector (chose to pre-multiply)
        //Eigen::internal::setRandomSeed(static_cast<unsigned int>(std::time(nullptr))) // Seed Eigen's internal RNG with current time
        Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
        Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix();  // random rotation matrix 
        for (int i=0; i<3; i++){
         RCLCPP_INFO(get_logger(),"row %d of random rotation matrix is: [%.2f  %.2f %.2f]", i+1, R(i,0),R(i,1),R(i,2)); 
        }

        Eigen::Vector3d d = Eigen::Vector3d::Random();       // random translation

        RCLCPP_INFO(get_logger(),"The random displacement vector is: %.2f %.2f %.2f", d(0),d(1),d(2));

        Eigen::Vector3d x_trans = R*X + d ;

        RCLCPP_INFO(get_logger(),"Transformed vector is: %.2f %.2f %.2f", x_trans(0),x_trans(1),x_trans(2));

        // Responses back to geometry_msgs form

        // Rotation Matrix 

        for (int i=0; i<3; i++) {
            result->rot_mat_r[i].x = R(i,0);
            result->rot_mat_r[i].y = R(i,1);
            result->rot_mat_r[i].z = R(i,2);
        }
        
        // displacement vector
        result -> d_r.x = d(0);
        result -> d_r.y = d(1);
        result -> d_r.z = d(2);

        // Transformed x vector

        result->x_t.x = x_trans(0);
        result->x_t.y = x_trans(1);
        result->x_t.z = x_trans(2);
         
    } catch (std::exception &e) {RCLCPP_INFO(get_logger(),"Error in computation"); } 
}
      
}; 

int main (int argc, char* argv[]) {
    rclcpp::init(argc,argv);
    auto node = std::make_shared<LeastSquaresService>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}