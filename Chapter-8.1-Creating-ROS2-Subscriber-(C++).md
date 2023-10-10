# Chapter 8.1 Creating ROS2 Subscriber (C++)

In this lesson, we are going to build a simple **subscriber node** using **C++** to receive and print the **string message** sent by the **publisher.cpp** file (*that we wrote, built and compiled in previous lesson 6.1*) over DDS.

# Adding The File

- Create a file **subscriber.cpp** file ****in the **ros2_cpp_udemy_tutorial/src/udemy_ros2_pkg/src** directory from the VS Code **Explorer** Sidebar.

![Untitled](Chapter%208%201%20Creating%20ROS2%20Subscriber%20(C++)%209598ce70fd80424e9d455d9623ec290b/Untitled.png)

- Add the following code to the **subscriber.cpp** file:
    
    ```cpp
    #include <rclcpp/rclcpp.hpp>
    #include <std_msgs/msg/string.hpp>
    
    #include <iostream>
    
    class HelloWorldSubNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        // rclcpp::Subscription< CallbackMessageT, AllocatorT, MessageMemoryStrategyT > is a Class Template - provided by the rclcpp package.
        // 'std_msgs::msg::String' is the value of typename argument CallbackMessageT.
        // This message type should be same as the message type of Publisher node messages.
        // AllocatorT & MessageMemoryStrategyT have default values so for now we can ignore them while initiating a class template instance.
        // typename AllocatorT = std::allocator<void>
        // typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy< CallbackMessageT, AllocatorT >
        // In ROS 1, this class was called 'Subscriber' instead.
        void sub_callback(const std_msgs::msg::String &msg) const
        {
            std::cout << msg.data << std::endl;
        }
    
    public:
        HelloWorldSubNode() : Node("hello_world_sub_node")
        {
            subscriber_ = this->create_subscription<std_msgs::msg::String>(
                "hello_world",
                10,
                std::bind(&HelloWorldSubNode::sub_callback, this, std::placeholders::_1));
    
            // rclcpp::Node::create_subscription() is a Node class function template inherited by HelloWorldSubNode class.
    
            // Actual form of this function:
            //*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
            // template<typename MessageT , typename CallbackT , typename AllocatorT , typename CallbackMessageT , typename SubscriptionT , typename MessageMemoryStrategyT >
            // std::shared_ptr< SubscriptionT > rclcpp::Node::create_subscription 	( 	const std::string &  topic_name,
            // 		                                              size_t qos,
            // 		                                              CallbackT && callback,
            // 		                                              options = SubscriptionOptionsWithAllocator<AllocatorT>(),
            // 		                                              msg_mem_strat = ( MessageMemoryStrategyT::create_default() ) 
            // 	                                              ) 
            //*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
            // topic_name =	The topic_name of the topic to subscribe on	- Same as publisher.cpp topic_name.
            // qos = history depth of messages recived by the subscriber - in case subscriber is not fast enough to process the recieved messages on time.
            // callback	= The user-defined callback function to receive a message. It defines what to do with the message after recieving it.
            // options = Additional options for the creation of the Subscription --> has default value.
            // msg_mem_strat = The message memory strategy to use for allocating messages --> has default value.
            //*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
            // MessageT: This is the type of message that the subscription will receive. It is the type of message that will be published to the topic that the subscription is listening to. For example, if the topic is publishing "std_msgs::msg::String" messages, then MessageT would be "std_msgs::msg::String".
            // CallbackT : Type of the callback function, for example, a function pointer, a std::function, or a class member function pointer.
            // AllocatorT: The type of the allocator that will be used for allocating the message instances.
            // CallbackMessageT: This is the type of message that will be passed to the callback function. It could be the same as MessageT if the callback function should receive the same message type as the one that the subscription is receiving, or it could be a different type. For example, if you want the callback function to receive a custom message type, MyCallbackMessage, then CallbackMessageT would be MyCallbackMessage.
            // SubscriptionT is the type of the 'subscription object' that will be created by the 'rclcpp::Node::create_subscription()' function. It is a template parameter that allows you to specify the type of the subscription object that will be returned by the function. It could be a custom class that inherits from rclcpp::SubscriptionBase class or rclcpp::Subscription class or rclcpp::SubscriptionWithAllocator class. By default, it is rclcpp::Subscription<MessageT> which is the built-in class for creating a subscription.
            // MessageMemoryStrategyT: The type of the message memory strategy that will be used.
            //*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
            //      class MyCallbackMessage : public std_msgs::msg::String {
            //      public:
            //        int custom_field;
            //      };
    
            //      class MySubscription : public rclcpp::Subscription<std_msgs::msg::String> {
            //      public:
            //        MySubscription(...) : rclcpp::Subscription<std_msgs::msg::String>(...) {...}
    
            //        void do_something_extra() {...}
            //      };
    
            //      void callback_fn(MyCallbackMessage::SharedPtr msg) {
            //          std::cout << "I heard: '" << msg->data << "'" << std::endl;
            //          std::cout << "Custom field value: " << msg->custom_field << std::endl;
            //      }
    
            //      int main(int argc, char ** argv) {
            //          rclcpp::init(argc, argv);
    
            //          auto node = rclcpp::Node::make_shared("my_node");
    
            //          auto sub = node->create_subscription<std_msgs::msg::String, MyCallbackMessage, MySubscription>(
            //            "topic_name",
            //            rclcpp::QoS(10),
            //            std::bind(callback_fn, std::placeholders::_1)
            //          );
    
            //          sub->do_something_extra();
    
            //          rclcpp::spin(node);
    
            //          rclcpp::shutdown();
            //          return 0;
            //      }
    
            // In this example code, the topic that the subscription is listening to is publishing messages of type "std_msgs::msg::String". But we want to pass the received message to the callback function with an additional custom field "custom_field" of type int. So, the CallbackMessageT is "MyCallbackMessage" which inherits from "std_msgs::msg::String" and has an additional field "custom_field of type int".
    
            // The node->create_subscription<>() method returns an object of "MySubscription" class which is the value for typename SubscriptionT. "MySubscription" is a custom class that inherits from rclcpp::Subscription<std_msgs::msg::String> and has an additional method do_something_extra().
            //*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
    
            // std::bind(&HelloWorldSubNode::sub_callback, this, std::placeholders::_1) 
    
            //This line of code creates a new function object by taking a specific function (sub_callback) from a specific class (HelloWorldSubNode) and connecting it with the current object of HelloWorldSubNode class (this). It also indicates that the first argument passed to this new function object will be passesd as the first argument to the sub_callback function when it is called.
    
            // In simpler terms, it creates a new function that, when called, will execute the sub_callback function on the current object and pass the first argument of the newly created function to the sub_callback function as its first argument.
    
            // std::placeholders::_1 is a placeholder argument that represents the first argument of the sub_callback function. When the function object returned by std::bind() is called, an argument with placeholder _1 is replaced by the first argument in the callback function - in this case the first argument of sub_callback().
        }
    };
    
    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<HelloWorldSubNode>());
        rclcpp::shutdown();
    
        return 0;
    }
    ```
    
    ## Clean Code Without Comments:
    
    ```cpp
    #include <rclcpp/rclcpp.hpp>
    #include <std_msgs/msg/string.hpp>
    
    #include <iostream>
    
    class HelloWorldSubNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
       
        void sub_callback(const std_msgs::msg::String &msg) const
        {
            std::cout << msg.data << std::endl;
        }
    
    public:
        HelloWorldSubNode() : Node("hello_world_sub_node")
        {
            subscriber_ = this->create_subscription<std_msgs::msg::String>(
                "hello_world",
                10,
                std::bind(&HelloWorldSubNode::sub_callback, this, std::placeholders::_1));
        }
    };
    
    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<HelloWorldSubNode>());
        rclcpp::shutdown();
    
        return 0;
    }
    ```
    
- Do the **boldified** **additions** to the **CMakeLists.txt** file:
    
    ```python
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
    
    **add_executable(subscriber src/subscriber.cpp) 
    ament_target_dependencies(subscriber rclcpp std_msgs)**
    
    install(TARGETS 
            **publisher** 
            subscriber
            DESTINATION lib/${PROJECT_NAME}
    )
    
    ament_package()
    ```
    

# Building The Workspace To Incorporate Changes

- Go to **Terminal** Tab → **Run Build Task**

![Untitled](Chapter%208%201%20Creating%20ROS2%20Subscriber%20(C++)%209598ce70fd80424e9d455d9623ec290b/Untitled%201.png)

<aside>
💡 After the workspace is successfully built, you can see the **subscriber** executable file inside **ros2_cpp_udemy_tutorial/build/udemy_ros2_pkg** directory and also its **symbolic link file** inside **ros2_cpp_udemy_tutorial/install/udemy_ros2_pkg/lib/udemy_ros2_pkg** directory.

![Untitled](Chapter%208%201%20Creating%20ROS2%20Subscriber%20(C++)%209598ce70fd80424e9d455d9623ec290b/Untitled%202.png)

![Untitled](Chapter%208%201%20Creating%20ROS2%20Subscriber%20(C++)%209598ce70fd80424e9d455d9623ec290b/Untitled%203.png)

</aside>

# Running The `subscriber` Node From The VS Code Terminal:

- Open a **new terminal** in the **VS Code Editor** - **ros2_cpp_udemy_tutorial** workspace.
- Run the following commands to start the `subcriber` node:
    
    ```bash
    source install/setup.bash
    ros2 pkg list
    #check that the name of udemy_ros2_pkg is included in the list.
    ros2 run udemy_ros2_pkg subscriber
    ```
    

# Running The `publisher` Node From The VS Code Terminal:

- Open a **second terminal** in the **VS Code Editor** - **ros2_cpp_udemy_tutorial** workspace - by clicking on the **Split Terminal** button at the top-right corner of the terminal.
    
    ![Untitled](Chapter%208%201%20Creating%20ROS2%20Subscriber%20(C++)%209598ce70fd80424e9d455d9623ec290b/Untitled%204.png)
    
    ![Untitled](Chapter%208%201%20Creating%20ROS2%20Subscriber%20(C++)%209598ce70fd80424e9d455d9623ec290b/Untitled%205.png)
    
- On the **second terminal**, run the following commands to start the `publisher` node:
    
    ```bash
    source install/setup.bash
    ros2 pkg list
    #check that the name of udemy_ros2_pkg is included in the list.
    ros2 run udemy_ros2_pkg publisher
    ```