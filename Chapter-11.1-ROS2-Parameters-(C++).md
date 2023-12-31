# Chapter 11.1. ROS2 Parameters(C++)

## Definition:

ROS2 parameters are **variables** used to store **values** that can be changed during **runtime** - from the **terminal**. ROS2 parameters are stored in a centralized parameter server, which allows nodes to access and modify them dynamically. ROS2 parameters can be of various types, including integers, floating-point numbers, strings, booleans, arrays, and more.

## Purpose/Use of Parameters

Let us once again take a look at the **rpm_publisher.cpp** source code - which we created in **Project 1.1**

### **rpm_publisher.cpp** code:

```cpp
// Including the rclcpp library - for ros2 c++ functionality.
#include "rclcpp/rclcpp.hpp"
// Next we are importing the interface of the messages we are going to publish through this node.
#include "std_msgs/msg/float64.hpp"

#include "chrono"
#include "functional"

using namespace std::chrono_literals;

const double RPM_VALUE = 100.0;  //<<--<<--<<--<<--

class RpmPubNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rpm_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void publish_rpm()
    {
        auto rpm_value = std_msgs::msg::Float64();
        rpm_value.data = RPM_VALUE;
        rpm_publisher_->publish(rpm_value);
    }

public:
    RpmPubNode() : Node("rpm_pub_node")
    {
        rpm_publisher_ = this->create_publisher<std_msgs::msg::Float64>("rpm", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&RpmPubNode::publish_rpm, this));
        std::cout<<"RPM Publisher Node Is Running..."<<std::endl;
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

Here, we have hard-coded the value of **RPM** using the `const double RPM_VALUE = 100.0;` line of code. Now, if we need to change the value of `const double RPM_VALUE` published by the **rpm_publisher**, we would usually think of directly editing the **hard-coded value** in **rpm_publisher.cpp** code i.e at `const double RPM_VALUE = 100.0;` . But by using the **ROS Parameter Concept,** we can change the value of the **RPM** that is being published by the **rpm_publisher** node — without directly editing the source code file **rpm_publisher.cpp** — from the **terminal** itself.

1. Open a **new terminal** in the **workspace** folder → Run the **rpm_publisher** node using the below command.

    ```cpp
    source install/setup.bash
    ros2 run udemy_ros2_pkg rpm_publisher
    ```

1. Open a **second terminal** in the **workspace** folder → Run the **rpm_subscriber** node using the below command.

    ```cpp
    source install/setup.bash
    ros2 run udemy_ros2_pkg rpm_subscriber
    ```

1. Open a **third terminal** in the **workspace** folder and run the below command.

    ```cpp
    ros2 param list
    ```

    ![Untitled](Images/Chapter11.1/Untitled.png)

    You will see a list of some **default** parameters for both the currently running nodes (**rpm_publisher** & **rpm_subscriber**).

1. **To see the value of a parameter** run the following command:

    ```bash
    #ros2 param get <node_name> <param_name>
    ros2 param get rpm_pub_node use_sim_time

    # Checking the value of use_sim_time - a default parameter of rpm_pub_node.
    ```

    ![Untitled](Images/Chapter11.1/Untitled%201.png)

    You will see that **use_sim_time** has a **Boolean** type value which is currently set to **false** — by default — because it uses the time of your local computer. You set the value of **use_sim_time** parameter to **true** if you want to use the time being generated by a **simulation** which you may have been running.

1. If you want to see more information about a parameter, use the following command:
    
    ```bash
    #ros2 param describe <node_name> <param_name>
    ros2 param describe rpm_pub_node use_sim_time
    ```
    
    ![Untitled](Images/Chapter11.1/Untitled%202.png)
    

## Using ROS2 Parameters Concept in rpm_publisher.cpp Source Code

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "chrono"
#include "functional"

using namespace std::chrono_literals;

const double RPM_DEFAULT_VALUE = 100.0; // Default constant RPM Value --- STEP 1

class RpmPubNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rpm_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double rpm_param_val_; // Creating a double variable rpm_param_val_ : for holding the value of 'rpm_val' parameter --- STEP 3

    void publish_rpm()
    {
        auto rpm_value = std_msgs::msg::Float64();

        // WAY 1:

        this->get_parameter("rpm_val", rpm_param_val_);
        // Using the 'Node::get_parameter()' method to summon the 'rpm_val' parameter and store its value (RPM_DEFAULT_VALUE) inside the 'rpm_param_val' variable. --- STEP 4

        rpm_value.data = rpm_param_val_; // --- STEP 5
        // We can change the value of rpm_value.data --> by changing the value of rpm_param_val_ variable --> by changing the value of 'rpm_val' parameter i.e associated with rpm_param_val_ variable ----> From the Terminal itself ---> Without having to change the actual source code (the value of RPM_DEFAULT_VALUE = 100.0 at line 9 of this file).

        // WAY 2:

        // rclcpp::Parameter rpm_param_val_object = this->get_parameter("rpm_val");
        // rpm_value.data = rpm_param_val_object.as_double();

        rpm_publisher_->publish(rpm_value);
    }

public:
    RpmPubNode() : Node("rpm_pub_node")
    {
        this->declare_parameter<double>("rpm_val", RPM_DEFAULT_VALUE); // Using the  'Node::declare_paramerter<ValueT>()' method to create a parameter 'rpm_val' for holding a 'double' type value and setting its default value to RPM_DEFAULT_VALUE   --- STEP 2
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

1. Before proceeding, make sure to **build** your code. 
2. To see our new **rpm_val** parameter in the **terminal,** run the following commands:
    
    **1st Terminal:** 
    
    ```bash
    source install/setup.bash
    ros2 run udemy_ros2_pkg rpm_publisher
    ```
    
    **2nd Terminal:**
    
    ```bash
    ros2 param list
    ```
    
    ![Untitled](Images/Chapter11.1/Untitled%203.png)
    
3. To check the value of **rpm_val** parameter from the **terminal,** run the following command:
    
    **2nd Terminal:**
    
    ```bash
    ros2 param get rpm_pub_node rpm_val
    #ros2 param get <node_name> <param_name>
    #You will get the current value of the rpm_val parameter
    ```
    
4. To see the messages published by **rpm_publisher** node:
    
    **3rd Terminal Parallel To 2nd Terminal:**
    
    ```bash
    ros2 topic echo rpm
    ```
    
5. To change the value of **RPM**  that is published by **rpm_publisher** node — from the **terminal —** without making any changes to the **source code —** run the following command:
    
    **2nd Terminal:**
    
    ```bash
    #ros2 param set <node_name> <parameter_name> <new_value>
    ros2 param set rpm_pub_node rpm_val 50.0
    ```