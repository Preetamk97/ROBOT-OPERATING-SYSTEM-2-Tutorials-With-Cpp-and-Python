# Chapter 6.1 Configuring Packages (C++)

In the last lesson, we **wrote** the code for a **C++ ROS2 Publisher Node** which sends a string type message “**Hello World :** ” followed by an integer that increments each time  -  inside the **src** folder of **udemy_ros2_pkg** package of our **ros2_cpp_udemy_tutorial** workspace. 

Now, to run the piece of code we need to **compile/build** it first and create an executable (**.exe**) file.

In ROS2, the **default C++ build system** is called **ament_cmake**. **ament_cmake** is the build system for CMake based packages in ROS 2 (in particular, it will be used for most if not all C/C++ projects). And we will use **ament_cmake** to build our C++ code with the **colcon** command line tool**.**

The core instructions of building our **udemy_ros2_pkg** package are stored inside the  **CMakeLists.txt** file of the **package folder**.

- **Step 1**. Open the **CMakeLists.txt** file of the **udemy_ros2_pkg** package folder in **VS Code**.

- **Step 2**. Add the following lines of code to the **CMakeLists.txt** file anywhere before the last line **ament_package()**

  ```cpp
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  add_executable(publisher src/publisher.cpp) 
  ament_target_dependencies(publisher rclcpp std_msgs)

  install(TARGETS
    publisher
    DESTINATION lib/${PROJECT_NAME}
  )
  ```

  **CMakeLists.txt** file code after adding the above lines of code:

  ```bash
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
  **find_package(rclcpp REQUIRED)
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

  **add_executable(publisher src/publisher.cpp) 
  ament_target_dependencies(publisher rclcpp std_msgs)

  install(TARGETS
    publisher
    DESTINATION lib/${PROJECT_NAME}
  )**

  ament_package()
  ```


  💡 The ROS2 **modules/packages** (**rclcpp & std_msgs**) which contain the header files (**rclcpp/rclcpp.hpp** & **std_msgs/msg/string.hpp**) that we `#included` in our **publisher.cpp** code are called **dependencies**.

  💡 **find_package(rclcpp REQUIRED)** & **find_package(std_msgs REQUIRED) :** We need to add the dependencies used in our **publisher.cpp** file inside this find_package() function in the format of  

  ```
  find_package(<dependency> REQUIRED)
  ```

  💡 **add_executable(publisher src/publisher.cpp)** : 
  - **publisher** = name of the **executable** file that we are going to create for the **publisher.cpp** source code file
  - **src/publisher.cpp =** location of the source code file.


  💡 **ament_target_dependencies(publisher rclcpp std_msgs)** : 
  - **publisher** = name of the **executable** file that we are going to create for the **publisher.cpp** source code file 
  - **rclcpp std_msgs** = **dependencies** of **publisher.cpp** source code file.


  💡
  ```
  install(TARGETS
    <executable_file_name>
    DESTINATION lib/${PROJECT_NAME}
  )
  ``` 

  - `lib/${PROJECT_NAME}` : This gives the **address/destination** for storing the executable file **publisher** when it is created ; which in this case is a **folder** by the name of **${PROJECT_NAME}** inside the **ros2_cpp_udemy_tutorial/install/udemy_ros2_pkg/lib** directory. The variable **PROJECT_NAME** references our **package name** as written at line no.2 of this **CMakeLists.txt** file code as `project(udemy_ros2_pkg)`.


  💡 If you ever want to change your **package name**, you have to change it in 3 places:
    1. The **package folder name.**
    2. **CmakeList.txt** file **Line No.2 — project(udemy_ros2_pkg)**
    3. package.xml file **Line No.4 — <name>udemy_ros2_pkg</name>**

- **Step 3**. Add the following code to the **package.xml** file of the **udemy_ros2_pkg** package.

  ```bash
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  ```

  **rclcpp** & **std_msgs** are the dependencies that our **publisher.cpp** file of the **udemy_ros2_pkg** package uses.

  **package.xml** file after the code addition:

  ```bash
  <?xml version="1.0"?>
  <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
  <package format="3">
    <name>udemy_ros2_pkg</name>
    <version>0.0.0</version>
    <description>TODO: Package description</description>
    <maintainer email="pritam@todo.todo">pritam</maintainer>
    <license>TODO: License declaration</license>

    <buildtool_depend>ament_cmake</buildtool_depend>
    **<depend>rclcpp</depend>
    <depend>std_msgs</depend>**

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <export>
      <build_type>ament_cmake</build_type>
    </export>

  </package>
  ```

  Save your work before proceeding.

- **Step 4. Open a new terminal** from the **ros2_cpp_udemy_tutorial** workspace directory. Run the following commands from the terminal:

  ```bash
  source /opt/ros/humble/setup.bash
  # For setting up the ros2 environment in the terminal.
  # Not needed if you already have added this command to your bashrc 
  # file using the command : echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

  colcon build
  # For compiling the entire **ros2_cpp_udemy_tutorial** workspace directory.
  # After running this command successfully, you can actually see
  # the executable file publisher created 
  # at **ros2_cpp_udemy_tutorial/build/udemy_ros2_pkg** folder
  # But we will use **ros2 run** command from our terminal to run 
  # this **publisher** executable file **-** which will be accessed from 
  # **ros2_cpp_udemy_tutorial/install/udemy_ros2_pkg/lib/udemy_ros2_pkg**

  source install/setup.bash
  # Sourcing the **setup.bash** file present inside the **install** directory of our 
  **# ros2_cpp_udemy_tutorial** workspace - so that our terminal becomes aware of 
  # the location of our **ros2_cpp_udemy_tutorial** workspace.
  # After running this command, all the packages built inside 
  # **ros2_cpp_udemy_tutorial** workspace will be recognised by the terminal.

  # To check if our terminal is sourced with the **ros2_cpp_udemy_tutorial** workspace
  ros2 pkg list
  # This will give a long list of various ros2 packages in which we can see
  # the names of all the packages built inside **ros2_cpp_udemy_tutorial** workspace.
  # In this case there is only one package built inside 
  # **ros2_cpp_udemy_tutorial** workspace -> the **udemy_ros2_pkg** package.

  ros2 run udemy_ros2_pkg publisher
  # ros2 run <name_of_package> <name_of_executable>
  # This command does not work if we have not sourced our terminal 
  # to the **ros2_cpp_udemy_tutorial** workspace folder.
  # So please ensure to always run **source install/setup.bash 
  #** command before running this command.
  ```

- **Step 5.** After running the last command above we cannot see anything on the terminal because the our “Hello World” messages are broadcasted over **ROS2** **DDS Communication Layer** and **not** as **standard output** to the **terminal**. **To see the messages on terminal, open a second terminal** in the  **ros2_cpp_udemy_tutorial** workspace directory → Run the following commands:

  ```bash
  ros2 node list
  # Gives list of active nodes'
  # /hello_world_pub_node

  ros2 topic list
  # Gives list of active topics
  # /hello_world
  # /parameter_events
  # /rosout

  ros2 topic echo /hello_world
  # For listening to the /hello_world topic messages.
  ```

## Change the name of a Node with ros2 run – at run time

You can add many arguments to the `ros2 run` command. Among them, there is one allowing you to directly change the node’s name at run time, without having to re-write/re-compile anything.

The first thing to add is `-ros-args` . Use `-ros-args` only once, and put all arguments after it.

To change the node’s name from “my_node” to “another_node”, use `r __node:=...:`

```bash
$ ros2 run ros2_tutorials_py minimal_node --ros-args -r __node:=another_node
[INFO] [1593588911.947569209] [another_node]: Node has been started.
```

You can see on the logline: the name of the node has been changed!

This feature will be handy when you want to launch multiple nodes with different names. For example, if you have a **single_wheel_controller** node, you can create a **right_wheel_controller** and a **left_wheel_controller** node, using the same code and executable.

And this is important: **you can’t start 2 nodes with the same name**, or else expect to see some weird behavior.