# Project 2.1. Launch Files (C++)

# Problem Statement

Create a file named **launch_project.launch.py** such that when we **launch** the file from the **terminal**:

1. It starts the **rpm_publisher** node.
2. It starts the **rpm_subscriber** node and also allows us to set the value of the **wheel_radius** parameter of our **rpm_subscriber** node.
3. It runs the `ros2 topic echo` command for **/speed** topic of the **rpm_subscriber** node.

# Solution

## **rpm_publisher.cpp Code (No Changes):**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "chrono"
#include "functional"

using namespace std::chrono_literals;

const double RPM_DEFAULT_VALUE = 100.0;

class RpmPubNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rpm_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double rpm_param_val_;

    void publish_rpm()
    {
        auto rpm_value = std_msgs::msg::Float64();
        this->get_parameter("rpm_val", rpm_param_val_);
        rpm_value.data = rpm_param_val_;
        rpm_publisher_->publish(rpm_value);
    }

public:
    RpmPubNode() : Node("rpm_pub_node")
    {
        this->declare_parameter<double>("rpm_val", RPM_DEFAULT_VALUE);
        
        rpm_publisher_ = this->create_publisher<std_msgs::msg::Float64>("rpm", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&RpmPubNode::publish_rpm, this));
        std::cout << "RPM Publisher Node Is Running..." << std::endl;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpmPubNode>());
    rclcpp::shutdown();
    return 0;
}
```

## **rpm_subscriber Code:**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "iostream"
#include "math.h"

**const double DEFAULT_WHEEL_RADIUS = 12.5 / 100;**

class RpmSubNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
    void calculate_and_pub_speed(const std_msgs::msg::Float64 &rpm_msg) const
    {
        **double wheel_radius_param_;
        this->get_parameter("wheel_radius", wheel_radius_param_);**
        auto speed_msg = std_msgs::msg::Float64();
        // Speed[m/s] = { RPM (rev/min) * Wheel_Circumference(meters/rev) } / 60 seconds
        speed_msg.data = (rpm_msg.data * 2 * M_PI * **wheel_radius_param_**) / 60;
        speed_publisher_->publish(speed_msg);
    }

public:
    RpmSubNode() : Node("rpm_sub_node")
    {
        **this->declare_parameter<double>("wheel_radius", DEFAULT_WHEEL_RADIUS);**
        rpm_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "rpm",
            10,
            std::bind(&RpmSubNode::calculate_and_pub_speed, this, std::placeholders::_1));

        speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("speed", 10);

        std::cout << "RPM Subscriber Node Is Running..." << std::endl;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpmSubNode>());
    rclcpp::shutdown();

    return 0;
}
```

## **launch_project.launch.py Code:**

```python
from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="udemy_ros2_pkg",
            executable="rpm_publisher",
            name="rpm_pub_node"
        ),
        Node(
            package="udemy_ros2_pkg",
            executable="rpm_subscriber",
            name="rpm_sub_node",
            parameters=[
                {"wheel_radius":10/100}
            ]
        ),
        ExecuteProcess(
            cmd=["ros2", "topic", "echo", "speed"],
            output="screen" 
        )
    ])
```

## CMakeLists.txt Code **(No Changes)**:

```cmake
cmake_minimum_required(VERSION 3.8)
project(udemy_ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(publisher src/publisher.cpp) 
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp) 
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(rpm_publisher src/rpm_publisher.cpp)
ament_target_dependencies(rpm_publisher rclcpp std_msgs)

add_executable(rpm_subscriber src/rpm_subscriber.cpp)
ament_target_dependencies(rpm_subscriber rclcpp std_msgs)

install(TARGETS 
        publisher 
        subscriber
        rpm_publisher
        rpm_subscriber
        DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```