# Chapter 3. Some Basic Terminologies & Features of ROS2

- **Data Distribution Service (DDS)** - DDS (Data Distribution Service) is a Communication Pipeline Interface that enables efficient data sharing among different nodes in ROS 2 ( a pipeline or conduit through which data can flow between nodes). It provides a set of rules and standards that define how nodes can send and receive data, and it provides a consistent, predictable way for nodes to communicate with each other. So, DDS acts as a communication interface between nodes in a distributed system, allowing them to exchange data efficiently and reliably. The DDS Pipeline also allows for security configurations, so that you can secure the data that you send between nodes.
- **Nodes** - Nodes are pieces of code that perform specific tasks and communicate with other nodes over the Data Distribution Service (DDS) in ROS 2.
- **Ways of communication between nodes over DDS:**
    1. **Publisher-Subscriber Model** : In the publisher-subscriber model, a node that has data to share (the **publisher**) sends the data as a **message** to **DDS** over a **topic** (a topic is a named data distribution pipeline or channel through which nodes can publish and subscribe to data), and other nodes that have subscribed to the topic (the **subscribers**) will receive the message and can access the data contained within it. This allows nodes to exchange data asynchronously, without the need for direct connections between them.
    2. **Service Model** : In the service model, a node (the service-server) provides a specific capability or function that other nodes (the service-clients) can request. The service-client sends 1 request message to the service-server, and the server sends 1 response message back to the client after the request has been successfully completed.
    3. **Actions** : In the action model, a node (the action client) sends a goal message to another node (the action server), and the server sends back periodic feedback messages as it works on the goal. Once the goal is complete, the server sends a result message back to the client.
- **Node Parameters -** Node Parameters are variables that can be retrieved and set by nodes at runtime. Node parameters are stored in the ROS 2 parameter server, which is a central repository for storing and managing parameters. Nodes can retrieve and set parameters using the ROS 2 parameter APIs, and they can also subscribe to changes in parameters. They allow nodes to have configurable behavior, which can be useful for tuning the performance of a system or for adapting to different environments**.**
- **Bag Files** - Bag files are files that store data from ROS topics and can be played back later to reproduce the data stream. Used to store the information collected by a robot. Bag files can subscribe to multiple topics and can record all the data that comes in. Bag files can also be used to publish the recorded data over the same corresponding topic names.
- **Packages** - A package in ROS 2 is a collection of code, data, and other resources that together perform a specific task or function. Each package has a specific purpose and contains everything needed to perform that purpose, including code files, data files, configuration files, and documentation. A package can be easily copied and redistributed to other developers who can then use it in their own projects/robots.
- **Cross-Platform Support -** ROS 2 ****can run on multiple operating systems including Linux, Windows, and MacOS.
- ROS2 allows **Easy integration** with ****projects developed in ****ROS 1.

## ROS Simulation and Visualization ****Overview****

ROS also includes software suites to help with robotics simulation and data visualization. This article is just a quick overview so you are aware of these features. These software suites will be gone in more detail later in the course.

### **Simulation**

- **Gazebo** : Gazebo is a free-to-use robot simulator which is able to communicate data over ROS. It is able to keep track of robot positions, as well as mimic the state of a real robot. It includes virtual sensors which can be used to simulate real sensor data so that you are able to test your robot code as if you were running it on a physical robotic system. Gazebo has been recently updated, and partnered with Ignition robotics, leading to the new Ignition Gazebo simulation engine.
    
    Website: [http://gazebosim.org/](http://gazebosim.org/)
    

### **Visualization**

- ***RViz***
    
    RViz is a 3D data visualization software suite which also interacts with data over ROS. RViz comes with data visualization features, as well as other localization based tools for you to interact with your robot, especially of one in simulation.
    
    Website: [https://index.ros.org/p/rviz2/](https://index.ros.org/p/rviz2/)
    
- ***RQT***
    
    RQT is plugin based graphical user interface to be used with ROS. It comes with various graphical plugins, such as a topic publisher, image viewer, parameter updater, node graph visualizer, and much more. It is worth noting that RViz and RQT have certain features with both software suites can perform.
    
    Website: [http://wiki.ros.org/rqt](http://wiki.ros.org/rqt)
    

## Difference Between ROS 2 and ROS1

![https://img-b.udemycdn.com/redactor/raw/article_lecture/2022-11-14_21-26-16-0627c55739999d363c4b2be315b7bc29.jpg](https://img-b.udemycdn.com/redactor/raw/article_lecture/2022-11-14_21-26-16-0627c55739999d363c4b2be315b7bc29.jpg)

Rather than updating the existing ROS1 API’s, ROS developers thought it would 
be best to entirely redo the overall structure in order to address key 
issues from various users of the ROS framework. Here are some 
highlights to the new features which would feel different to those who have 
developed previously in ROS1.

### Decentralized Style Of Communication

One of the most notable changes overall to ROS is the transition to a Data 
Distribution Service (DDS) schema of transmitting data. This means that 
there is no longer a centralized ROS Master (roscore) or Parameter 
Server in which all Nodes have to register with. Instead, every Node can
 act on its own accord without a ROS Master.

### Security Configurations For Topics

This feature was a huge appeal to those who use ROS in applications that 
work with private/confidential data. This allows developers to create 
secure keys for their Nodes, such that without them the topics/data can 
not be discovered nor decrypted.

### Launch File Rework

Launch files in ROS1 used to be XML based. Now they are Python based, in which
 you will need to import the roslaunch python module. Overall the
 layout of organizing things into arguments, Nodes, etc, still exists, 
just now in a Python form.

### Bag File Rework

Bag files also got a slight rework. Functionality pretty much works the 
same, although now data is saved into SQLite database files. It’s also 
worth noting that there is now a default ROS2 topic which monitors 
parameter events, which means you can take parameter change events into 
account with bag files now.

### Official Support For Actions

Although ROS Actions were always a highlighted feature, it always seemed to act 
as an external library (actionlib). In ROS2, actions now have their own 
dedicated functionality and terminal commands to help with development.

### Cross Platform Support

One of the biggest leaps in terms of usability of ROS by the general public
 was also making it available for use natively on Windows and MacOS. 
Although I feel there is still a long way to go to fix all the OS 
specific ROS2 bugs, it may pay off in the long run of making ROS a much 
more widely acceptable framework to use by those not familiar with 
Linux.

### Compatibility With ROS1

ROS2 offers the ROS1 bridge tool in order to allow you to develop new projects in ROS2 
without having to compromise on utilizing existing systems you may have 
developed in ROS1.