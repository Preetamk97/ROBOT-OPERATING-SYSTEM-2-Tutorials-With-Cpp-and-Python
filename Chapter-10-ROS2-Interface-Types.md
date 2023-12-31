# Chapter 10. ROS2 Interface Types

**ROS2 Interfaces** are **ways of defining** various **Messages, Services, and Actions** that are used by **ROS2 nodes** to **communicate** with each other. 

1. **Message Interfaces** - Stored in `/msg` folders. Message Interfaces (`.msg` files) are - simple text files - that are similar to a **C++ struct data structures** and are **used to represent and exchange data between nodes in a ROS2 system**. Each field in the Message Interface code typically corresponds to a
    -  **built-in ROS 2 data type** such as ***int32, float32, or std::string*** or,
    - **a Message Interface defined on its own**, such as ***geometry_msgs/PoseStamped***
    
    For example, the **std_msgs/msg/String** interface is a **.msg** format code that holds a single field, **"data"** of **string** datatype. 
    
    Examples of **folders/packages** containing **message interfaces** include:
    
    - `std_msgs/msg`: This folder contains several **message interfaces** for representing various **primitive data types**, such as **strings**, **integers,** and **floating-point numbers**. Some examples of interfaces contained in this folder include:
        - `std_msgs/msg/String` is a **.msg** format interface which carries only one field called **data** for holding a **string** type value.
        - `std_msgs/msg/Bool` is a **.msg** format interface that carries only one field called **data** for holding a **bool** type value.
        - `std_msgs/msg/Int32` is a **.msg** format interface which carries only one field called **data** for holding a **int32_t** type value.
        
            ![**A list of Basic Primitive DataTypes in ROS2 and their corresponding names in C++ and Python Language**.*Source - ROS2 Foxy Official Documentation*](Images/Chapter10/Untitled.png)
            <br>(*Image taken directly from ROS 2 Humble Documentation* - https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
        
    - `sensor_msgs/msg`: This folder contains **message interfaces** for representing various types of **sensor data**, such as data from **cameras, lidars, and IMUs**. Some examples of interfaces contained in this folder include:
        - `sensor_msgs/msg/Image` is a **.msg** format interface in ROS2 that is used to represent image data from a **camera**. The interface contains several fields such as **header**, **height,** **width**, **encoding**, **step** and **data.**
        - `sensor_msgs/msg/LaserScan` is a **.msg** format interface in ROS2 that is used to represent data from a **LIDAR sensor**. The interface contains several fields such as **header**, **angle_min**, **angle_max**, **angle_increment**, **time_increment**, **scan_time**, **range_min**, **range_max**, **ranges** and **intensities.**
        - `sensor_msgs/msg/Imu`  is a **.msg** format interface in ROS2 that is used to represent data from an **Inertial Measurement Unit (IMU) sensor.** The interface contains several fields such as **header**, **orientation**, **orientation_covariance**, **angular_velocity**, **angular_velocity_covariance**, **linear_acceleration** and **linear_acceleration_covariance**.
        - `sensor_msgs/msg/FluidPressure` is a **.msg** format interface in ROS2 that is used to represent data from a **fluid pressure sensor.** The interface contains several fields such as **header**, **fluid_pressure**, **variance** and **temperature**.
2. **Service Interfaces** - Stored in the `/srv` folders. A **service interface** is specified by a **.srv** file, which contains **the definitions** of the **request** and **response** message types for the **service**. These message types are used to define the **contract for the service,** specifying what data is sent in as request and what data is expected in the response.
    
    When the **.srv** file is built, it generates **C++ classes for the request and response message types, as well as client and server classes** for handling the communication with the service. These classes provide a programmatic interface for carrying out the service, allowing for the creation of requests, sending them to the service, handling responses and so on.
    
3. **Action Interfaces** - Stored in the `/action` folders. An action interface is defined by a **.action** file. The **.action** file contains the definition of the **goal message**, the **result message** and the **feedback message**, which are used to represent the **action request**, the **final outcome**, and the **intermediate feedback** respectively. 
    
    When the **.action** file is built, it generates **C++ classes** for the **goal, result, and feedback message types**, as well as a **client and server classes** for handling the **communication** with the **action**. These classes provide a **programmatic interface** for working with the action, allowing for the **creation of goals, sending them to the action, handling feedback and results** and so on.
    

To see the list of all the ROS2 interfaces available for communication between nodes, type the command `ros2 interface list` on a **terminal**. All these different **ROS2 interfaces** are organized under different **packages** within the ROS2 installation.