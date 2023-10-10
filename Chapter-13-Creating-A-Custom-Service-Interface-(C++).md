# Chapter 13. Creating A Custom Service Interface (C++)

In this lesson, we will create a **simple custom ROS2 service interface** of our own - which will contain an integer named **number** as a **request message** and a string named **decision** about whether the integer is “Odd” or “Even” as the **response message**.

# Service Interface

A **service interface** is a **request-response architecture interface**, which allows two **nodes** (service client & service server) to **communicate** with each other through a **request-response communication model**.

# Service Client

The node which sends a **request message** to the service server node.

# Service Server

The node which **processes** the **request message** sent from the **service client** and **gives** a **response message** back to the **service client**.

1. ROS2 also comes with some default service interfaces. To see them, run the `ros2 interface list` command from a terminal.
2. To see the details/content of any interface, run the following bash command: 
    
    `ros2 interface show <complete-interface-name>`
    
    Example: Lets us check the details/content of **std_srvs/srv/SetBool** service interface.
    
    ![Untitled](Chapter%2013%20Creating%20A%20Custom%20Service%20Interface%20(C+%208b595c69713a4763ad486969101c75af/Untitled.png)
    
    In the above picture, 
    
    `bool data` : is the request message of **std_srvs/srv/SetBool** service interface.
    
    `bool success` & `string message` : are the response messages of **std_srvs/srv/SetBool** service interface.
    
    The three dashed line  `---` marks the division between the request message & the response messages of the interface.
    
3. Open your **VS Code** in **Workspace** **Directory**.
4. Under the **udemy_ros2_pkg** package folder, create a new folder named **srv.** Inside this **srv** folder, we are going to create our custom ROS2 service interface files.
5. Create a new file inside the **srv** folder named **OddEvenCheck.srv .**
    
    > *Note that the custom **.srv** files follow the **PascalCasing** Convention for naming.*
    > 
6. Add the following code to **OddEvenCheck.srv** and save the file.
    
    ```bash
    # A Service Interface is like a contract or protocol of communication between the Service-Client Node and the Service-Server Node.
    
    # Request Message Type & Variable Name
    # The Service-Client Node will be feeding an "int64" type value named "number" as a "Request Message" to the Service-Server Node.
    int64 number   # Number to check if odd or even
    
    ---
    # The three dashes signify the division between the Request Message and Response Message in the Service Interface.
    
    # Response Mesage Type & Variable Name
    # The Service-Client will be recieving a "string" named "decision" as a "Response Message" - back from the Service-Server.
    string decision  # Either "Odd" or "Even"
    ```
    
7. Next, head over to the **package.xml** file of the **udemy_ros2_pkg** folder, where we will add in some new dependencies that will help us to use the newly created custom service interface file.
    
    Add the following **boldified** lines of code to the **package.xml** file and **save the file**.
    
    ```xml
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>udemy_ros2_pkg</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="pritam@todo.todo">pritam</maintainer>
      <license>TODO: License declaration</license>
    
      <buildtool_depend>ament_cmake</buildtool_depend>
    
      <depend>rclcpp</depend>
      <depend>std_msgs</depend>
      **<build_depend>rosidl_default_generators</build_depend>**  
      <!-- Indicates that the package requires the rosidl_default_generators package during the build process. rosidl_default_generators contains the default code generators for the ROS 2 Interface Definition Language (IDL). -->
      **<exec_depend>rosidl_default_runtime</exec_depend>**
      <!-- Indicates that the package requires the rosidl_default_runtime package during execution so that the Interface Definition Language (IDL) created can be used during node runtime. -->
      **<member_of_group>rosidl_interface_packages</member_of_group>**
      <!-- To include the newly created service interface to the group of other ROS2 interfaces.  -->
    
      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>
    
      <export>
        <build_type>ament_cmake</build_type>
      </export>
    
    </package>
    ```
    
8. Next, head over to the **CMakeLists.txt** file of the **udemy_ros2_pkg** folder and add the following **boldified** lines of code. 
    
    ```python
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
    **find_package(rosidl_default_generators REQUIRED)**  
    
    if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      set(ament_cmake_copyright_FOUND TRUE)
      set(ament_cmake_cpplint_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()
    endif()
    
    # We need to tell our ros2 compiler - the exact specifics of the newly created custom service interface file - that it needs to have the IDL Code generated for.
    # This line of code should always come before the add_excutable blocks, if you are planning to use the generated custom interface in these executables.
    **rosidl_generate_interfaces(${PROJECT_NAME} "srv/OddEvenCheck.srv" ADD_LINTER_TESTS)**                                                              
     
    # Set support for using custom interfaces in C++ from this package
    # This line should always be below the "rosidl_generate_interfaces()" code - otherwise it will produce compilation error.
    **rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")**
    
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
    
9. **Save** all the files after making the changes and **build** the **workspace.**

Just now when we compiled our workspace, ROS2 generated some new **C++ header files** for us to be able to use our newly created custom interface **OddEvenCheck.srv** in our **ROS2 C++ codes**.

We can see these header files at **ros2_cpp_udemy_tutorial/install/udemy_ros2_pkg/include/udemy_ros2_pkg/udemy_ros2_pkg/srv** directory.

![Untitled](Chapter%2013%20Creating%20A%20Custom%20Service%20Interface%20(C+%208b595c69713a4763ad486969101c75af/Untitled%201.png)

Now we have successfully created our **OddEvenCheck.srv** custom interface file and integrated it with our workspace.

To see the name of our newly created **OddEvenCheck.srv** custom interface in `ros2 interface list` , open a new terminal and run the following commands:

```bash
cd Ros2_Workspaces/ros2_cpp_udemy_tutorial 
# Moving to the workspace directory

source install/setup.bash
# Sourcing the workspace

ros2 interface list
```

![Untitled](Chapter%2013%20Creating%20A%20Custom%20Service%20Interface%20(C+%208b595c69713a4763ad486969101c75af/Untitled%202.png)

We can also see the contents of the **OddEvenCheck.srv** interface using the `ros2 interface show udemy_ros2_pkg/srv/OddEvenCheck` command:

![Untitled](Chapter%2013%20Creating%20A%20Custom%20Service%20Interface%20(C+%208b595c69713a4763ad486969101c75af/Untitled%203.png)