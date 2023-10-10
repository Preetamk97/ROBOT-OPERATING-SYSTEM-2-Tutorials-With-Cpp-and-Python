# Chapter 17.1. ROS2 Actions(Python)

In this lesson, we will be doing a project walkthrough to explain how to utilise **Action interfaces** in ROS2.

**ROS Action Interfaces** are another **inter-node communication method** just like ROS Services.

| Action | Service |
| --- | --- |
| Goal | Request |
| Result | Response |
| Feedback | Publisher |

# Project Statement

- We have a **mobile robot**, which is being controlled by an **action client node** and an **action server node.** The **action client node** sends a 3D-coordinate location (**Goal**) ****to the **action server node.**
- Upon receiving the **Goal** by the **action server node,** the robot starts **moving/navigating** - ****from ****its current location - to the given **Goal Point**. ****While the robot is navigating to the given **Goal Point**, the **action server node** keeps sending data ****(Feedback)**** - back to the **action client node** - on how far is the robot still - from the from the given **goal point.**
- When the robot reaches its destination, the **action server node** returns **the time,** on how long it took for the robot to reach its destination, ****to the **action client node,** as **Result.**
    
    
    | Goal | (x,y,z) coordinates |
    | --- | --- |
    | Feedback | Distance remaining |
    | Result | Time Elapsed |

- Additionally, we will also need to create a **Publisher** (topic: **/robot_position**) - for simulating a **position sensor** - which keeps publishing the **current robot position** of the robot. Our **action server node** will find the distance between the **Goal Coordinate Point** and the simulated **current robot position** to send to the **action client node** as **feedback** on how far the robot is from the **Goal Point** yet.

# Creating The Custom Action Interface

- Create a new folder called **action** inside the **src/udemy_ros2_pkg** package folder of your **ros2_py_udemy_tutorial** workspace.
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled.png)
    
- Within this folder, we will create our own **Custom Action Interface file** named **Navigate.action .**
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%201.png)
    
- Add the following code inside **Navigate.action** file.
    
    ```python
    # Goal
    geometry_msgs/Point goal_point
    # geometry_msgs/Point is the ROS Message Datatype for holding 3D spatial coordinates. 
    ---
    # Result
    float32 time_elapsed
    ---
    # Feedback
    float32 distance_to_goal
    ```
    
- Save the file and close it.
- Now, move on to the **package.xml** file within your **udemy_ros2_pkg** package folder.

# Configuring The `package.xml` For The Newly Created Custom Action Interface

- Add the following code (**dependencies**) to the **package.xml** file. These dependencies are necessary for integrating **any custom interface** into our ROS package (udemy_ros2_pkg).
    
    ```xml
    <!-- Adding the below dependency in order to be able to use our Custom ROS Service and Action Interfaces in our package -->
      <!-- The below build_dependency is used to generate the idl(interactive data language) code for our Custom ROS Service and Action Intefaces -->
      <build_depend>rosidl_default_generators</build_depend>
      <!-- Below dependency is added so that the the idl(interactive data language) code can be used at node runtime. -->
      <exec_depend>rosidl_default_runtime</exec_depend>
      <!-- Below dependency is added to include our Custom Intefaces into the ROS2 Interfaces List.  -->
      <member_of_group>rosidl_interface_packages</member_of_group>
    ```
    
- Also add the following dependency (only for the case of **Custom Action Interface**) to the **package.xml** file.
    
    ```xml
    <!-- Adding the below dependency to be able to use Custom Action Interfaces in our package -->
    <depend>action_msgs</depend>
    ```
    
- Save the file and close it.
- Now, move on to the **CMakeLists.txt** file within your **udemy_ros2_pkg** package folder.

# Configuring `CMakeLists.txt` For Our Newly Created Custom Action Interface

- Add the following code to the **CMakeLists.txt** file of your **udemy_ros2_pkg** package folder**.**
    
    ```python
    # Adding the below dependency for configuring all the 
    # Custom ROS Service AND Action Interfaces created inside this package.
    find_package(rosidl_default_generators REQUIRED)
    
    # Adding the below dependency to be able to use geometry_msgs interface collection in our codes within this package.
    find_package(geometry_msgs REQUIRED)
    
    # Telling our compiler exactly what new custom interfaces files we have created in our package
    # that needs to have the ros idl code generated for it.
    rosidl_generate_interfaces(${PROJECT_NAME}
      "srv/OddEvenCheck.srv"
      "srv/TurnCameraService.srv"
      "action/Navigate.action"
      DEPENDENCIES 
      sensor_msgs 
      geometry_msgs
    )
    # ${PROJECT_NAME} signifies the name of our package (udemy_ros2_pkg)
    
    # sensor_msgs/msg/Image is the type of message interface that we are using inside the TurnCameraService.srv as response message datatype. 
    # geometry_msgs/msg/Point is the type of message interface that we are using inside the Navigate.action custom interface as goal message datatype.
    # Since these message interfaces does not belong to the group of standard message intefaces (std_msgs/msg/**), therefore, we need to include these complete interface collections (sensor_msgs and geometry_msgs) as dependencies in the above code of rosidl_generate_interfaces.
    ```
    

---

- That is it for configuration of our package for the newly created custom action interface. Rebuild the workspace before proceeding further.
    
    **Terminal → Run Build Task**
    
- Now open a terminal from your workspace folder and run the following commands.
    
    ```bash
    source install/setup.bash
    ros2 interface list
    ```
    
    We can see the name of our newly created custom action interface **udemy_ros2_pkg/action/Navigate** in the output list.
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%202.png)
    
- To see the contents of **Navigate.action** interface, run the following command from the same terminal.
    
    ```bash
    ros2 interface show udemy_ros2_pkg/action/Navigate
    ```
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%203.png)
    

---

# Writing Our Action Server Node

- Create a new file named **action_server.py** inside the **scripts** folder of the **udemy_ros2_pkg** package folder.
- Add the following code to the file.
    
    ```python
    #! /usr/bin/env python3
    
    import rclpy                        
    from rclpy.node import Node    
    from rclpy.action import ActionServer   # Necessary Import To Create Action Server
    from udemy_ros2_pkg.action import Navigate  # Importing the Custom Action Interface 
    from geometry_msgs.msg import Point   # We use this point message type while creating 
                                        # the robot postion subscriber below.  
    import math     # Importing math module    
    
    class NavigateActionServer(Node):    
        def __init__(self):
            super().__init__("navigate_action_server_node") 
            
            # In the below line, we create our action server.
            self._action_server = ActionServer(self, Navigate, 'navigate', self.navigate_callback)
            # The ActionServer class constructor takes 4 arguments -
            #   1. the object of NavigateActionServer class itself: self
            #   2. the custom action interface to use: Navigate
            #   3. the action topic name (you can give any name you want): 'navigate'
            #   4. the callback function for processing once the server recieves the goal from the client: self.navigate_callback
            
            # Creating a subscriber to recieve the current robot position.
            self.robot_postion_sub = self.create_subscription(Point, 'robot_postion', self.update_robot_position, 1)
            # create_subscription method of Node class takes 4 arguments - message type, subcription topic name, callback method (self.update_robot_position), and qos factor. 
            
            # Placeholder for current robot position
            self.current_robot_position = None
        
        # callback function for robot position subscriber.
        # this function is just gonna keep updating the value of self.current_robot_position 
        def update_robot_position (self, point):
            self.current_robot_position = [point.x, point.y, point.z]
            
        # Defining the navigate_callback function
        def navigate_callback(self, goal_handle):
            print("Goal Recieved...")
            
            # In the below line, we are Unpacking the goal coordinates recived from the action client using goal_handle object - and stuffing them all into a list. 
            # The 'goal_handle' object's 'request' attribute contains the 'goal_point' 3D spatial coordinate value. 
            # 'goal_point' is the name of the action goal message that we have set in the Navigate.action custom interface code. 
            # The 'goal_point' message is of geometry_msgs/Point datatype that holds 3 float64 type values namely - x, y, and z. 
            robot_goal_point = [goal_handle.request.goal_point.x,
                                goal_handle.request.goal_point.y,
                                goal_handle.request.goal_point.z]
            
            # Printing the goal coordinates recieved.
            print("Goal Coordinates Recieved: " + str(robot_goal_point))
            
            # Now since our robot has received the goal coordinates, now it starts to move.
            # Noting the start_time when the robot starts moving towards the given goal_point in seconds.
            start_time = self.get_clock().now().to_msg().sec
            
            # Before recieving the current robot position
            # ********************************************
            # while our subscriber has not recieved any robot current position data yet.
            while self.current_robot_position == None:  
                print ("Robot Current Postion Not Detected...")
                # Below line instructs to wait for a little while (3 seconds in this case) and see if current_robot_position data gets updated.
                rclpy.spin_once(self, timeout_sec=3)
                # since no robot position is detected yet, the loop starts again
            
            # After recieving the current robot position
            # *******************************************
            # printing the current robot position recieved
            print("Current Robot Position Recieved: " + str(self.current_robot_position)) 
            # Calculating distance between current robot position and goal point using math module.   
            distance_to_goal = math.dist(self.current_robot_position, robot_goal_point)
            
            # Untill the robot reaches to its destination 
            while distance_to_goal != 0:
                # Initializing the feedback message of Navigate custom action inteface. 
                feedback_msg = Navigate.Feedback()
                # Loading the feedback message value
                feedback_msg.distance_to_goal = distance_to_goal
                # Publishing the feedback message to action client using goal_handle
                goal_handle.publish_feedback(feedback_msg)
                # printing the current robot position recieved
                print("Current Robot Position Recieved: " + str(self.current_robot_position)) 
                # Calculating the new distance_to_goal.
                distance_to_goal = math.dist(self.current_robot_position, robot_goal_point)
                # wait for 1 sec before publishing the next feedback
                rclpy.spin_once(self, timeout_sec=1)
            
            # ******************************************************************************
            
            # Below lines of code are gonna get executed only when 'distance_to_goal' becomes equal to zero - that means when our robot has reached its given goal_point.
            goal_handle.succeed()   # Letting the goal handle know that we have successfully 
                                    # completed the given task
            end_time = self.get_clock().now().to_msg().sec  # recording the end time of the 
                                                            # task.
            result_msg = Navigate.Result()  # Initializing the Result Message
            result_msg.time_elapsed = float(end_time - start_time)      # Loading the result 
                                                                        # message
            
            # returning the final result message to the action client.
            return result_msg
            
                
    def main(args=None):
        rclpy.init()                            
        action_server_node =  NavigateActionServer()       
        print("Action Server Node is running...")
        
        try:
            # while everything is ok, keep the action_server node alive.
            while rclpy.ok():
                rclpy.spin_once(action_server_node)                  
                                                
        except KeyboardInterrupt:               
            print("Terminating action server node...")
            action_server_node._action_server.destroy()
            action_server_node.destroy_node()                
        
    
    if __name__=='__main__':  
        main()
        
        
    # A Systematic Study of rclpy.spin(), rclpy.spin_once() and rclpy.spin_until_future_complete() functions:
    
    # 1. rclpy.spin() Function:
    
    # This function is a common way to keep your ROS 2 node running and active. When you call rclpy.spin(node), your node will continuously listen for incoming messages, process them, and perform any necessary actions. It's like telling your node, "Stay awake and do your tasks, and let me know if anything happens." This function blocks the program's execution and keeps your node running indefinitely until you interrupt it (e.g., with Ctrl + C).
    
    # 2. rclpy.spin_once() Function:
    
    # Imagine you want your node to do something specific, like checking for messages or updates, but you don't want it to keep running in a loop forever. You use rclpy.spin_once(node) to tell your node, "Check if there's anything new, and then you can go back to what you were doing." It's a way of balancing between staying active and giving your node time to rest. This function is useful when you need to check for updates periodically without fully committing to a continuous loop.
    
    # 3. rclpy.spin_until_future_complete() Function:
    
    # This function is like setting a task for your node and waiting until that task is completed. Instead of just running and checking for updates indefinitely, you give your node a specific task, such as sending a message, and then you wait until that task is finished. It's like saying, "Hey, do this thing, and let me know when you're done." This function is great for scenarios where you want to wait for a specific action to complete before moving on.
    ```
    
- **Save** the file before closing it.
- Mention the file inside **CMakeLists.txt** file of the package in the following block of code.
    
    ```python
    # Specifying our python scripts.
    install(PROGRAMS
      **scripts/action_server.py**
      DESTINATION lib/${PROJECT_NAME}
    )
    ```
    

# Writing Our Action Client Node

- Create a new file named **action_client.py** inside the **scripts** folder of the **udemy_ros2_pkg** package folder.
- Add the following code to it.
    
    ```python
    #! /usr/bin/env python3
    
    import rclpy                        
    from rclpy.node import Node    
    from rclpy.action import ActionClient   # Necessary Import To Create Action Client
    from udemy_ros2_pkg.action import Navigate  # Importing the Custom Action Interface    
    
    class NavigateActionClient(Node):    
        def __init__(self):
            super().__init__("navigate_action_client_node")     
            # In the below line, we create our action server.
            self._action_client = ActionClient(self, Navigate, 'navigate')
            # The ActionClient class constructor takes 3 arguments -
            #   1. the object of NavigateActionClient class itself: self
            #   2. the custom action interface in use: Navigate
            #   3. the action topic name (you can give any name you want): 'navigate'
        
        # function for sending goal to action server
        def send_goal(self, x, y, z):
            # initalizing the goal message
            goal_msg = Navigate.Goal()
            # loading the goal message
            goal_msg.goal_point.x = float(x)
            goal_msg.goal_point.y = float(y)
            goal_msg.goal_point.z = float(z)
            
            # waiting for the action server to become live
            self._action_client.wait_for_server()
            
            # sending goal to the action server
            self._send_goal = self._action_client.send_goal_async(goal_msg, self.feedback_callback)
            # The send_goal_async is an asynchronus method that takes 2 arguments:
            #   1. the goal message that you want to send to the action server : goal_msg
            #   2. a feedback callback method that defines what you want to do with the feedback that the action client will recieve from the action server : self.feedback_callback.
            
            # Now we sent the goal to the action server and added instructions on what to do on recieving the feedbacks.
            # But before recieving the final result from the action server, we need to make sure whether the action server has successfully accepted our action goal or NOT.
            #  Once we confirm this, we can proceed with planning what to do with the final result that will be recieved.
             
            # the below line of code sets the stage for what we are gonna do once we recieve the final result from the action server.
            self._send_goal.add_done_callback(self.server_decision_callback)
            # the add_done_callback method takes in a function as argument that determines/makes sure whether the action server has successfully accepted the goal sent by the client : server_decision_callback.
            # the server_decision_callback will then define further what to do with the result recieved if the goal has been successfully accepted by the server.
        
        
        # defining the feedback callback method.    
        def feedback_callback(self, feedback_msg):
            # unpacking feeback attribute from the feedback_msg object
            feedback = feedback_msg.feedback
            # printing the feedback
            print("Recieved Feedback : " + str(feedback.distance_to_goal))
            
        
        # defining the server decision callback function
        def server_decision_callback(self, future):
            server_decision = future.result()   # Note: This result is just the boolean true/false that tells whether the 
                                                # action server has successfully accepted the goal sent by the action client.
    
            if server_decision.accepted == False:
                print("Goal Rejected By Server")
                return None
            
            # else if, the server_decision.accepted == True then
            print("Goal Accepted")
            self._get_result_future = server_decision.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        
        # defining the get_result_callback method : to process the action result recieved. 
        def get_result_callback(self, future):
            result = future.result().result
            print("Result Recieved : " + str(result.time_elapsed) + " seconds.")
            # shutting down all ROS comunication operations and spinning of our action client once we recieve our result.
            rclpy.shutdown()
            
            
    def main(args=None):
        rclpy.init()                            
        action_client_node =  NavigateActionClient()       
        print("Action Client Node is running...")
        
        try:
            x = input("Enter a X-Coordinate : ")                  
            y = input("Enter a Y-Coordinate : ") 
            z = input("Enter a Z-Coordinate : ") 
            
            action_client_node.send_goal(x, y, z)
            
            rclpy.spin(action_client_node)  # making sure that our action client stays up untill the action server completes 
                                            # its task.
            
        except KeyboardInterrupt:               
            print("Terminating action client node...")
            action_client_node.destroy_node()                
        
    
    if __name__=='__main__':  
        main()
    ```
    
- **Save** the file and close it.
- Mention the file inside **CMakeLists.txt** file of the package in the following block of code.
    
    ```python
    # Specifying our python scripts.
    install(PROGRAMS
      **scripts/action_server.py
    	scripts/action_client.py**
      DESTINATION lib/${PROJECT_NAME}
    )
    ```
    
- Re-build the workspace once, before proceeding any further.

# Test Running Our Action Server And Action Client Nodes

- To run the **action server node**, open a new terminal from the workspace folder and run the following commands.
    
    ```bash
    source install/setup.bash
    ros2 run udemy_ros2_pkg action_server.py
    ```
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%204.png)
    
- To run the **action client node,** open a new terminal from the workspace folder and run the following commands.
    
    ```bash
    source install/setup.bash
    ros2 run udemy_ros2_pkg action_client.py
    ```
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%205.png)
    
- After doing the last step, you can see that the **action_client.py node terminal (terminal 2) i**s asking us to enter the values for x, y, and z-coordinates of the **Goal Point** for the robot to navigate to.
    
    Let us enter the following values : **x = 2.0, y = 3.0, z = 0.0** 
    
    These values represent the **coordinate location** of the **Goal Point.**
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%206.png)
    
- As evident from the **action_server** terminal readings, the **action server** is expecting for the **robot current position** readings, which are to be made available over the **/robot_position** topic.
- Open a new terminal (**terminal 3**) and run the command `ros2 topic list` to see the list of all the active terminals.
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%207.png)
    
- We currently do not have any publisher to publish the **robot current position** readings over the **/robot_position** topic. But we can also manually publish some data over this topic even without a publisher. The messages we publish manually in this case will be 3-D coordinate readings of the **robot current position.** The ROS message datatype for 3-D spatial coordinates is **geometry_msgs/msg/Point.**
    
    To manually publish the **robot current position** readings over the topic **/robot_position,** run the following command from **terminal 3.**
    
    ```bash
    ros2 topic pub --once /robot_position geometry_msgs/msg/Point "{x: 1.0, y: 2.0, z: 0.0}"
    
    # Here, '--once' part signifies that this message is gonna be published over the topic for only once and will NOT be published over and over again infinitely. 
    # '/robot_position' is the topic over which we are gonna publish the message.
    # 'geometry_msgs/msg/Point' is the datatype of the message we are gonna publish.
    # The values of the message x: 1.0, y: 2.0 and z: 0.0 are written within "{}" 
    ```
    
    This command means that our robot’s **current position** has been updated to x = 1.0, y = 2.0 and z = 0.0.
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%208.png)
    
    In the above picture, **terminal 1** shows that the **action server** has successfully received the position coordinates published by us just now and its value of **current robot position** has been updated successfully. Also, the server has calculated the remaining distance between the **newly updated current robot position** and the **Goal Point** and sent the data over to the **action client** as **feedback** (observe **terminal 2**) ****.
    
- Now, let us move our robot to the set **Goal Point.** Run the below command from **terminal 3** and observe the corresponding responses in the other terminals.
    
    ```bash
    ros2 topic pub --once /robot_position geometry_msgs/msg/Point "{x: 2.0, y: 3.0, z: 0.0}"
    ```
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%209.png)
    
    Here we are updating the value of **current robot position** as equal to ****the **Goal Point -** implying to our **action server** that our robot has reached its destination.
    
- Observe the final **Result** value (**time elapsed**) that gets printed on the action client terminal (terminal 2).
    
    ![Untitled](Chapter%2017%201%20ROS2%20Actions(Python)%20731ef2bdf32a498fb56a4fcc6ddcb003/Untitled%2010.png)