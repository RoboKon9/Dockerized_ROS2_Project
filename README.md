Dockerized ROS2 Project - Technical Assessment for Progressive Robotics

To build the image and start the container run:

*sudo docker-compose up --build*

To run the container service run:

*sudo docker-compose exec -it tech_assess_container bash*

Navigate to the ROS2 workspace and build it:

*cd ~/root/tech_assess_ws*

*colcon build*

source the workspace:

*source install/setup.bash*

Test the linear algebra service:

1) Run the client node

*ros2 run linear_algebra_service least_squares_client*

2) Open a new terminal inside the container

sudo docker exec -it tech_assess_container bash

3) navigate, build and source again

*cd ~/root/tech_assess_ws*

*colcon build*

*source install/setup.bash*

3) Run the server node

*ros2 run linear_algebra_service least_squares_server*

3) The intermediate and final results will be printed on the terminal(s).











