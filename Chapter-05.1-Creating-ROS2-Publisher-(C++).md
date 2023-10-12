# Chapter 5.1 Creating ROS2 Publisher (C++)

In this lesson, we will create our very own **ROS2 Publisher Node** using **C++**. We'll make a simple publisher that publishes a string that says **Hello World**, followed by a number which increments up every time we publish. Then in a future lesson, we'll create a subscriber to receive the messages we publish.

1. For now, let's open our code editor → **open folder** → open **ros2_cpp_udemy_tutorial** workspace folder.
2. In the VS Code File Explorer Panel on the left, under the **source directory (src)** → **udemy_ros2_pkg** package folder → **source directory(src) →** Create a new file named **publisher.cpp.**
3. Complete information on **ROS2 C++ API** is documented in the official [rclcpp Documentation](https://docs.ros2.org/foxy/api/rclcpp/index.html).

# Commented Code:

```cpp
#include <rclcpp/rclcpp.hpp>
// In order to use the ROS2 Functionality in our C++ code - we need to include the header file 'rclcpp/rclcpp.hpp' which is present inside the rclcpp module (folder)
// The header file can be found at 'opt/ros/humble/include/rclcpp/rclcpp'
// The 'rclcpp' module is a part a ROS2 Client Library or ROS Client Library.
// The 'rclcpp' module of the ROS 2 client library provides a set of C++ classes and functions for creating and interacting with ROS 2 nodes, topics, services, and other ROS 2 features. It allows for the creation of ROS 2 nodes in C++ and provides a C++ API for sending and receiving messages, creating and using services, and interacting with other ROS 2 nodes.
// The ROS 2 client library, also known as RCL (ROS Client Library) is composed of several modules in addition to the rclcpp module, each of which provides a specific set of functionality.
// The rcl package is a low-level package that provides the core functionality for creating, initializing, and managing ROS2 nodes. It is written in C and is intended to be used by other higher-level packages such as rclcpp and rclpy.
// Another package that is part of the RCL is rclpy, which is the ROS Client Library for Python. It provides similar functionality as rclcpp, but for the Python programming language. It allows developers to create ROS2 nodes and interact with the ROS2 system in Python.
// Additionally, there is also rcljava, which is the ROS Client Library for Java, and rclc, which is the ROS Client Library for C. These provide similar functionality as rclcpp and rclpy, but for the Java and C programming languages respectively.

#include <std_msgs/msg/string.hpp>
// opt/ros/humble/include/std_msgs/msg/string.hpp
// This line includes the "string" message type from the std_msgs package, which is a basic message type in ROS2 that can be used to send and receive string data between nodes. The std_msgs package contains many other basic message types as well, such as int32, float64, and boolean.

#include <chrono> 
//Neccesary for giving value of duration argument inside create_wall_timer(duration, callback_func) function below.
#include <functional>
// Necessary import for using the std::bind() function below.

using namespace std::chrono_literals;
// To give the argument value as '1s' in create_wall_timer(duration, callback_func) function.

// Usually when creating a ROS2 node, we create a class which will hold all the functionalities related to that ROS2 node's operation.
class HelloWorldPubNode : public rclcpp::Node
{
// This line of code defines a class named "HelloWorldPubNode" which is inherited in "public" mode from "rclcpp::Node" class.
// The 'Node' class is provided by the rclcpp module (or namespace) which is a part of the ROS 2 client library (RCL).
// Keep in mind that the 'HelloWorldPubNode' is the name of our Node class - but not the name of our node itself.

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // This line of code creates a shared pointer (::SharedPtr) to a 'rclcpp::Publisher<std_msgs::msg::String>' class object "publisher_"
    // A shared pointer is a C++ standard template library (STL) class that is typically implemented as a wrapper around a raw pointer. When a shared pointer is created, it takes ownership of the memory pointed to by the raw pointer .
    // An object in heap memory can have more than one shared pointer pointing to it. When multiple shared pointers are created that point to the same heap object, they all share ownership of the object and each pointer increments a reference count. As long as there is atleast one shared pointer pointing to the object, the object will not be deallocated. When the last shared pointer pointing to the object is destroyed or reassigned, the object's reference count will drop to zero and the object memory is automatically deallocated. A shared pointer is a type of smart pointer in C++ that provides automatic memory management and reference counting
    // The ::SharedPtr clain C++ that provides automatic memory management and reference counting
    // The ::SharedPtr class is a smart pointer class provided by the 'rclcpp' module, it is implemented as a wrapper class around the C++ Standard Template Library (STL) 'std::shared_ptr' class. It provides automatic memory management and reference counting for objects in a ROS 2 node.
    // rclcpp::Publisher<MessageT, AllocatorT> is a template class provided by rclcpp module.
    // In the class template rclcpp::Publisher<MessageT, AllocatorT>, the template parameter MessageT specifies the data type of the message that is to be sent by the publisher object.
    // In "rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;" -- "std_msgs::msg::String" is the value of MessageT parameter
    // In "std_msgs::msg::String" - String is a struct. It is a message type is used to represent a simple string message in ROS2. it contains only one field data of type std::string which is used to store the string data. It is defined as a struct in the std_msgs::msg package, which is a part of ROS2.
    // In "rclcpp::Publisher<std_msgs::msg::String>" - no value has been given for AllocatorT parameter - which will call the default heap allocator.
    // We added an underscore in the name of the object (publisher_) to specify that it is a private scope variable.

    rclcpp::TimerBase::SharedPtr timer_;
    // Creating a shared pointer to the 'rclcpp::TimerBase' class object 'timer_'
    //'TimerBase' class is provided by the 'rclcpp' package/module.

    size_t counter_ = 0;
    // creating a 'size_t' type variable 'counter' and initialising it to zero 0.

    void publish_hello_world()
    { // CallBack Function for initialising 'timer_' object.
        auto message = std_msgs::msg::String();
        // The 'auto' keyword specifies that the type of the variable that is being declared will be automatically deducted from its initializer. In the case of functions, if their return type is auto then that will be evaluated by return type expression at runtime.
        // The type of the mesage is denoted by 'std_msgs::msg::String()'
        message.data = "hello world " + std::to_string(counter_);
        publisher_->publish(message);
        counter_++;
    }

public:
    HelloWorldPubNode() : Node("hello_world_pub_node")
    {
        // Constructor of HelloWorldPubNode class.
        // The constructor does not take any argument for itself - but we need to initialise the parent 'Node' class which takes in - the string "hello_world_pub_node" - which is the name of our node that will be displayed in the terminal when our node is running.
        // The name "hello_world_pub_node" - follows snake_case_format (all small case - each word separated by underscore)- this is the naming convention to follow while naming things such as nodes, topics, parameters that will be broadcasted in the ROS terminal.

        publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);
        // Initiating 'publisher_' private object attribute of the 'HelloWorldPubNode' class object.
        // rclcpp::Node::create_publisher<message_Type> (topic_name, qos)
        // Inside object 'publisher_' , we are storing - the return value - of 'create_publisher<std_msgs::msg::String>("hello_world", 10)' function -- invoked on the 'this' object itself.
        //"std_msgs::msg::String" -- is a wrapper stuct for "std::string" datatype - used to denote the message type of the publisher we are creating.
        //"hello_world" is the name of the topic - over which the publisher will publish the "hello world" string messages.
        // 10 is the qos number - QOS denotes quality of service. It means the publisher will keep record of maximum 10 unpublished messages incase the publisher is not able to publish the messages fast enough on time.

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&HelloWorldPubNode::publish_hello_world, this));
        // Initialising private attribute object 'timer_'
        // We are applyning the 'rclcpp::Node' class function 'create_wall_timer(duration, callback_func)' on the 'this' object and storing the return value inside the timer_ object.
        // The value of duration argument is '1s' - which means the timer will rerun the callback function after every 1 second.
        // To give the value '1s' for the 'duration' argument - we need to write 'using namespace std::chrono_literals' and #include <chrono>
        // We cannot write the argument callback_function directly. Instead, we need to put it under std::bind(&callback_func, this) function as argument along with 'this' object.
    }
};

int main(int argc, char * argv[]) // The argc and argv parameters are standard command-line arguments that are passed to the main function, allowing any additional command-line arguments to be passed to the node - when it is started. These command-line arguments can be used to configure various aspects of the ROS2 system, such as setting the ROS2 master URI, setting the node name, and passing in remappings.
{
    rclcpp::init(argc, argv);
    // Inside init(), we pass the arguments of `int main()` function - argc & argv.
    // The rclcpp::init(argc, argv); code initializes the ROS2 rclcpp library and sets up the ROS2 node by passing in command-line arguments. It is responsible for creating and configuring the node, as well as initializing the ROS2 rclcpp library, including the initialization of the DDS communication layer that is handled by the underlying libraries of ROS2, and parsing any command-line arguments passed to the node, to allow for proper communication and configuration of the node.
    // DDS communication layer gets initiated when the rclcpp library is loaded and it is done by the rclcpp library itself.

    rclcpp::spin(std::make_shared<HelloWorldPubNode>());
    // This rclcpp::spin() function helps us manage all the asynchronous functionality of our ROS node and keeps the node running until we stop it.
    // std::make_shared<Class_Name>() -- is a C++ Standard Template Library (STL) function template that is used to create a shared pointer for a dynamically allocated class object - created in heap memory.
    // std::make_shared<HelloWorldPubNode>() ---> Here we are created a dynamically allocated object of 'HelloWorldPubNode' class in heap memory and stored its address inside a shared pointer. And we passed this shared pointer inside the rclcpp::spin() function as argument.
    // The rclcpp::spin() function starts the event loop of a publisher node, which keeps the node running and actively publishing messages until it is manually stopped by the user via the terminal or programmatically with rclcpp::shutdown() method.
    // The event loop is a mechanism that continuously checks for new messages, and processes them as they arrive and continues to run and check for new messages until the node is manually stopped or programmatically shutdown. In the context of a publisher node, the event loop is responsible for continuously checking for new messages to be published, and publishing them to the topic.

    rclcpp::shutdown();
    // The rclcpp::shutdown(); line of code is used to shut down the ROS2 system and the node by calling the shutdown() function of the rclcpp library. This function is responsible for releasing any resources allocated by the node, closing communication interfaces and stopping the event loop that was started by the rclcpp::spin() function.
    
    return 0;
}
```

# Clean Code Without Comments:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

class HelloWorldPubNode : public rclcpp::Node
{

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    size_t counter = 0;

    void publish_hello_world()
    {
        auto message = std_msgs::msg::String();
        message.data = "hello world " + std::to_string(counter);
        publisher_->publish(message);
        counter++;
    }

public:
    HelloWorldPubNode() : Node("hello_world_pub_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&HelloWorldPubNode::publish_hello_world, this));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<HelloWorldPubNode>());

    rclcpp::shutdown();

    return 0;
}
```