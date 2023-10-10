# Chapter 5.2 Creating ROS2 Publisher (Python)

In this lesson, we will create our very own **ROS2 Publisher Node** using **Python**. We'll make a simple publisher that publishes a string that says **Hello World**, followed by a number which increments up every time we publish. Then in a future lesson, we'll create a subscriber to receive the messages we publish.

1. For now, let's open our code editor → **open folder** → open **ros2_py_udemy_tutorial** workspace folder.
2. In the VS Code File Explorer Panel on the left, under the **source directory (src)** → **udemy_ros2_pkg** package folder → **script** folder **→** Create a **new file** named **publisher.py.**
3. For more information on **ROS2 Python API** you can refer to the official **rclpy** **Documentation** ([https://docs.ros2.org/foxy/api/rclpy/index.html](https://docs.ros2.org/foxy/api/rclpy/index.html)).
4. Add the following code to **publisher.py** file**.**
    
    ```python
    import rclpy                        # importing rclpy module : Python library for ROS 2
    from rclpy.node import Node         # importing Node class from rclpy.node module
    from std_msgs.msg import String     # importing message datatype (class) - String - from 
                                        # std_msgs.msg module
    
    class HelloWorldPublisher(Node):    # class 'HelloWorldPublisher' is inheriting from 'Node' class
        
        # Constructor : Is called when an object of this class is created.
        def __init__(self):
            super().__init__("hello_world_pub_node")    # Calling the __init__() method             
                                                        # (constructor) of super class 'Node'.
                                                        # Takes the "name of the node" 
                                                        # as String type argument
                                                        
            # Creating a publisher                                  
            self.pub = self.create_publisher(String, 'hello_world', 10)     
            # self.pub is just an attribute of HelloWorldPublisher class.
            # We are creating a 'publisher' inside attribute 'self.pub' using the 
            # self.create_publisher() method which is inherited from the parent class 'Node'.
            # create_publisher() method takes 3 parameters namely - Message Datatype (String), Topic Name ('hello_world'), qos (quality of service)
            # qos = it is an Integer which tells how many unpublished mesages should the publisher keep in record that are waiting to be sent over the network or delivered to subscribers)
            
            self.counter = 0                            # self.counter is just another attribute 
                                                        # of HelloWorldPublisher class 
                                                        
            # Creating a timer                                            
            self.timer = self.create_timer(0.5, self.publisher_callback_func)   
            # self.create_timer() method is inherited from the parent class 'Node'.
            # This method will trigger the 'self.publisher_callback_func()' function after every o.5 seconds.
            # The create_timer method is provided by the Node class and takes two arguments: the timer period (0.5 seconds) and the callback function (self.publisher_callback_func).
         
        # This method is the callback function that will be executed by the timer.   
        def publisher_callback_func(self):
            msg = String()                                  # creating a new 'std_msgs.msg.String' class message object.
            msg.data = "Hello World" + str(self.counter)    # assigning a value to the message object's 'data' field
            self.pub.publish(msg)                           # publishing the message using the publish() method of self.pub
            self.counter += 1                               # incrementing the self.counter attribute value by 1.
        
    def main(args=None):
        rclpy.init()                            # Initializing the ROS 2 communication system
        my_pub =  HelloWorldPublisher()         # Creating object of HelloWorldPublisher class
        print("Publisher Node is running...")
        
        try:
            rclpy.spin(my_pub)                  # When rclpy.spin() is called with a 'Node' instance ('my_pub' in this case) 
                                                # - it starts the event loop of the ROS 2 system. 
                                                # The event loop is a continuous loop that keeps the ROS 2 node active and responsive to various events, such as receiving messages, calling callbacks, and handling other system tasks.
                                                # As long as the event loop is active, the publisher node will continue to publish messages (using publisher_callback_func(self) method) at the specified rate (0.5 s) and respond to any incoming events. 
                                                # As long as the event loop is active, it blocks the further execution of any other code written below.
                                                # The event loop runs until it is explicitly terminated. In this code, the termination of the event loop is triggered by a KeyboardInterrupt error below. When a keyboard interrupt is detected, the execution of the event loop stops, and the program flow moves to the exception handling block.
                                                
        except KeyboardInterrupt:               # KeyboardInterrupt error is caused on pressing (ctrl + C) to explicitly  
                                                # terminate a process/operation going on in the console/terminal.
            print("Terminating publisher...")
            my_pub.destroy_node()               # Destroying the node           
        
    
    if __name__=='__main__':  
        # It Allows You to Execute the Code inside it When the File Runs as a Script, but Not When The File is Imported as a Module.
        main()
    ```
    
5. You can run this file from your **VS Code** in two ways (*your VS Code must be opened from your **workspace folder***)
    1. Using the **Run Python File Button** at the top-right corner of your VS Code window.
        
        ![Untitled](Chapter%205%202%20Creating%20ROS2%20Publisher%20(Python)%20f59ac1eb3481464cb02194358147144e/Untitled.png)
        
    2. Using **VS Code Integrated Terminal** *(or **any other** terminal)*
        
        ```bash
        python3 /home/pritam/Ros2_Workspaces/ros2_py_udemy_tutorial/src/udemy_ros2_pkg/scripts/publisher.py
        
        # python3 <complete address of your publisher.py file>python3 /home/pritam/Ros2_Workspaces/ros2_py_udemy_tutorial/src/udemy_ros2_pkg/scripts/publisher.py
        
        # python3 <complete address of your publisher.py file>
        ```
        
        ![Untitled](Chapter%205%202%20Creating%20ROS2%20Publisher%20(Python)%20f59ac1eb3481464cb02194358147144e/Untitled%201.png)
        
        We cannot see the messages published by the **publisher.py** file in the **current** terminal because they are being **published/broadcasted** over **ROS 2 DDS** (*Data Distribution Service*).
        
6. **Keep the current terminal open** & **open a 2nd terminal**. 
7. To get the list of **currently active** **nodes** use the following command in the **2nd terminal**:
    
    ```bash
    ros2 node list
    ```
    
    ![Untitled](Chapter%205%202%20Creating%20ROS2%20Publisher%20(Python)%20f59ac1eb3481464cb02194358147144e/Untitled%202.png)
    
    As we can see the name of our **publisher.py** file’s **node** name **hello_world_pub_node** in the the list of the **currently active** **nodes** - this proves that our file **publisher.py** is running successfully.
    
8. To get the list of **currently active** **topics** use the following command in the **2nd terminal**:
    
    ```bash
    ros2 topic list
    ```
    
    This should list our **publisher.py** file’s **topic** name **hello_world** in the output.
    
    ![Untitled](Chapter%205%202%20Creating%20ROS2%20Publisher%20(Python)%20f59ac1eb3481464cb02194358147144e/Untitled%203.png)
    
    **/parameter_events** & **/rosout** are **default** topics of ROS 2 DDS.
    
9. To see the messages published by our **publisher.py** file’s **hello_world** topic, use the following command in the **2nd terminal**:
    
    ```bash
    ros2 topic echo /hello_world
    
    # ros2 topic echo /topic_name
    ```
    
    ![Untitled](Chapter%205%202%20Creating%20ROS2%20Publisher%20(Python)%20f59ac1eb3481464cb02194358147144e/Untitled%204.png)
    

![Untitled](Chapter%205%202%20Creating%20ROS2%20Publisher%20(Python)%20f59ac1eb3481464cb02194358147144e/Untitled%205.png)