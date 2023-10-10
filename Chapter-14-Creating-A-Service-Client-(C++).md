# Chapter 14. Creating A Service Client (C++)

In this lesson, we are going to create a **C++ Service Client Node** for the **OddEvenCheck.srv** custom service interface that we created in the last lesson.

<aside>
💡 **Service Client** 
The node which sends a **request message** to the **service server** node.

</aside>

1. Open your **VS Code** in **Workspace** **Directory**.
2. Create a **service_client.cpp** file in the **src** directory of the **udemy_ros2_pkg** package folder.
3. Add the following code to the **service_client.cpp** file:
    
    ```cpp
    #include "rclcpp/rclcpp.hpp"  // This line includes the header file of the ROS2 Client Library (rclcpp) which is the main library used for creating ROS2 nodes in C++.
    #include "udemy_ros2_pkg/srv/odd_even_check.hpp"  // This line includes the "odd_even_check.hpp" header file located at 'install/udemy_ros2_pkg/include/udemy_ros2_pkg/udemy_ros2_pkg/srv' folder of our workspace directory - so that we can use our newly created 'udemy_ros2_pkg/srv/OddEvenCheck.srv' custom inteface file in our C++ code.
    #include <iostream>  // This line includes the standard C++ library for input and output operations, used for printing output to the console.
    
    int main(int argc, char *argv[])
    {
        rclcpp::init(argc, argv); // This line initializes the ROS2 environment.
    
        auto service_client_node = rclcpp::Node::make_shared("odd_even_check_client_node"); // This line returns a shared pointer of an instance of "rclcpp::Node" class and names this instance "odd_even_check_client_node" and then stores this shared pointer inside "auto service_client_node" variable. The shared pointer makes sure that the node will be automatically destroyed when it is no longer needed, which makes memory management easier.
        auto client = service_client_node->create_client<udemy_ros2_pkg::srv::OddEvenCheck>("odd_even_check"); // This line stores - a shared pointer to an instance of type "rclcpp::Client" for the "udemy_ros2_pkg::srv::OddEvenCheck" service interface which is connected to a service named "odd_even_check" - inside "auto client" variable whose datatype is "rclcpp::Client<udemy_ros2_pkg::srv::OddEvenCheck>::SharedPtr"
        auto request = std::make_shared<udemy_ros2_pkg::srv::OddEvenCheck::Request>(); // auto request stores the shared pointer of the udemy_ros2_pkg::srv::OddEvenCheck::Request class object which has the attribute - int64 number
    
        std::cout << "Please enter a number to check if it is Odd or Even: ";
        std::cin >> request->number;
    
        client->wait_for_service();  // This line blocks the execution of the program until the service is available.
    
        auto result = client->async_send_request(request);  // Sending the "request" shared pointer which points to the "udemy_ros2_pkg::srv::OddEvenCheck::Request" class object carrying message/attribute "int64 number" - to the 'service_server' node via service named "odd_even_check" - through the 'rclcpp::Client<udemy_ros2_pkg::srv::OddEvenCheck>::SharedPtr  client' object pointer's async_send_request() method - which returns a "future response object" - will be stored inside the "auto result" variable. The "future" object represents the result of the asynchronous service request and it can be used to retrieve the result of the service call (the response message - string decision) once it is available.
        
        if (rclcpp::spin_until_future_complete(service_client_node, result) == rclcpp::FutureReturnCode::SUCCESS)
        // If everyrthing is okay and the 'response object' has been recieved successfully from the 'service' then:
        // The 'rclcpp::spin_until_future_complete()' function blocks the execution of the program until the 'future response object' "result" passed as the second argument has successfully recieved the response message - string decision, i.e., the service response is complete.
        // The first argument passed to the function is a pointer to the service client node - that is calling the service. This is done so that the node's execution context can be used to spin the node, i.e., to repeatedly keep checking for incoming service response messages, until the response arrives.This is done so that the node can keep checking for the response, instead of just waiting for it to arrive. This way, the node can be kept active while it waits for the response, hence the term "spin".
        // Spinning a node refers to repeatedly checking and processing incoming messages on that node in a ROS2 system.
        // Once the future object is complete, the function returns a rclcpp::FutureReturnCode, which can be either SUCCESS if the future was completed successfully, or INTERRUPTED if the future was interrupted by a signal or a cancellation request.
        {
            std::cout << "That number is: " << result.get()->decision << std::endl;
        }
        else
        {
            std::cout << "There was an eror processing the request..." << std::endl;
        };
    
        rclcpp::shutdown();  // Terminating the ROS2 environment.
    
        return 0;
    }
    ```
    
4. Save the file and head over to the **CMakeLists.txt** file within the **udemy_ros2_pkg** package folder. 
    
    Do the following **boldified** additions to the **CMakeLists.txt** file:
    
    ```c
    cmake_minimum_required(VERSION 3.8)
    project(udemy_ros2_pkg)
    
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()
    
    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    # Necessary import for using Custom Service Interfaces
    find_package(rosidl_default_generators REQUIRED)  
    
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      set(ament_cmake_copyright_FOUND TRUE)
      set(ament_cmake_cpplint_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()
    endif()
    
    # We need to tell our ros2 compiler - the exact specifics of the newly created custom service interface file - that it needs to have the IDL Code generated for.
    # This line of code should always come before the add_excutable blocks, if you are planning to use the generated custom interface in these executables.
    rosidl_generate_interfaces(${PROJECT_NAME} "srv/OddEvenCheck.srv" ADD_LINTER_TESTS)                                                              
     
    # Set support for using custom interfaces in C++ from this package
    # This line should always be below the "rosidl_generate_interfaces()" code - otherwise it will produce compilation error.
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
    
    add_executable(publisher src/publisher.cpp) 
    ament_target_dependencies(publisher rclcpp std_msgs)
    
    add_executable(subscriber src/subscriber.cpp) 
    ament_target_dependencies(subscriber rclcpp std_msgs)
    
    add_executable(rpm_publisher src/rpm_publisher.cpp)
    ament_target_dependencies(rpm_publisher rclcpp std_msgs)
    
    add_executable(rpm_subscriber src/rpm_subscriber.cpp)
    ament_target_dependencies(rpm_subscriber rclcpp std_msgs)
    
    **add_executable(service_client src/service_client.cpp)
    ament_target_dependencies(service_client rclcpp std_msgs)
    target_link_libraries(service_client "${cpp_typesupport_target}")**
    
    install(TARGETS 
            publisher 
            subscriber
            rpm_publisher
            rpm_subscriber
    				**service_client**
            DESTINATION lib/${PROJECT_NAME}
    )
    
    install(
      DIRECTORY
      launch
      DESTINATION share/${PROJECT_NAME}
    )
    
    ament_package()
    ```
    
5. Save all the files and **compile** the **workspace.**
6. To run the **service_client** node, open a new terminal in the **workspace** directory and run the following commands:
    
    ```bash
    source install/setup.bash
    ros2 run udemy_ros2_pkg service_client
    ```
    
    ![Untitled](Chapter%2014%20Creating%20A%20Service%20Client%20(C++)%20a8c4678d503f45929ecfd84bf2ea1946/Untitled.png)
    
7. Open a **parallel** terminal and run the following command:
    
    ```bash
    ros2 service list
    #To see the list of active services
    ```
    
    ![Untitled](Chapter%2014%20Creating%20A%20Service%20Client%20(C++)%20a8c4678d503f45929ecfd84bf2ea1946/Untitled%201.png)