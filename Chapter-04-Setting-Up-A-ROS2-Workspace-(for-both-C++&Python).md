# Chapter 4 Setting Up A ROS2 Workspace (for both C++ & Python)

In this chapter, we'll be going over creating a **ROS2 Workspace (*for holding both C++ & Python codes*)** where we can organize our development files. 


## **ROS2 Workspace File Structure**

Before we do anything on our computer, let's get an illustration of what our file structure should look like.

1. First thing to start off our development is to create a **workspace directory**. 

    ![Untitled](Images\Chapter4\Untitled.png)

1. Inside our **workspace directory**, we will create a **source folder (src)**, which is where the **ROS2 packages** we create will live.

    ![Untitled](Images/Chapter4/Untitled%201.png)

1. Other folders, which will also appear in the workspace folder, are the **build, install** and **log** folders which will be auto generated when we compile our workspace.

    ![Untitled](Images/Chapter4/Untitled%202.png)

1. Let us go ahead and focus more on the **source folder (src)** since that is where we will be organizing our project contents. As mentioned earlier, this **source folder** is where our **packages** will be placed. Think of packages like individual software bundles generally used for a specific task/role which can then be shared and distributed to other developers as well.

    ![Untitled](Images/Chapter4/Untitled%203.png)

1. Let's go ahead and see what our package structure looks like. 
    
    ![Untitled](Images/Chapter4/Untitled%204.png)
    
- **src:** This is the folder where we will store our **C++** codes for the package. **Auto-generated** - created automatically when we create our package.
- **package.xml:** This file **contains information about our package** such as the **ROS packages and systems we are using as dependencies** in this package as well as the **author** and **version number** of the package. **Auto-generated** - created automatically when we create our package.
- **CMakeLists.txt:** This file contains specific instructions on compiling our package. **Auto-generated** - created automatically when we create our package.
- **script:** This folder is **not auto-generated** folder . We create this folder manually to store our **Python** scripts for the package. Also, this folder must **mandatorily** **consist** a **blank** file by the name of `__init__.py` . Adding code to `__init__.py` is **optional.**
- Lastly, also keep in mind that within our package folder, we may also need to create **additional folders** (just like **script** folder) for storing a variety of other files such as **launch files**, **custom interface declarations**, **simulation configuration files** etc. as long as they pertain to the purpose of the project. We will go over these in more depth later in the course.

All right, now that you have a general overview of what our ROS2 Workspace File Structure should look like, let's go ahead and create our own workspace.

## **Steps For Creating A ROS2 Workspace**

1. Go to **Home** directory → **Create a new folder** by **right clicking** and clicking **new folder** → We will call this folder **Ros2_Workspaces**. This folder will contain all our ROS 2 workspaces. 
    
    ![Untitled](Images/Chapter4/Untitled%205.png)

    >💡 ROS2 Workspaces generally live in our Home directory

1. Go inside **Ros2_Workspaces** folder → Make a workspace folder named **ros2_cpp_udemy_tutorial (ros2_py_udemy_tutorial)**. 
    
    > 💡 Generally, a ROS 2 workspace folder is named after the robot we are working on. But for the purposes of this tutorial, we will call this workspace - <b>ros2_cpp_udemy_tutorial (ros2_py_udemy_tutorial)</b> , just so we know, that this is our workspace where we work on projects related to this course.

    ![Untitled](Images/Chapter4/Untitled%206.png)

1. Go inside **ros2_cpp_udemy_tutorial (ros2_py_udemy_tutorial)** workspace folder → Create **source folder (src)**. This is the folder where we will create different packages related to the project.
1. Go inside the **source folder (src)**.
1. Inside the **source folder (src),** we are going to create a package folder named **udemy_ros2_pkg .** To do this **:** Open **source folder (src)** directory in a **terminal** → Run the command  `ros2 pkg create udemy_ros2_pkg --build-type ament_cmake`  → This will make a folder **udemy_ros2_pkg** inside **source folder (src)**
1. Close all the terminals.

    > 💡 If you are only going to be using <b>ONLY</b> Python scripts in your ROS package (and no C++ codes), you can replace the <b>ament_cmake</b> part of the above code to <b>ament_python</b>, in which case you would use a <b>setup.py</b> script to create your build instructions (instead of <b>CMakeLists.txt</b>), but then you would not be able to add in any C++ code in that package, whereas <b>ament_cmake</b> allows us to use both C++ and Python code.


    > 💡 If we go inside this <b>udemy_ros2_pkg</b> folder we can see that ROS 2 has created some default files and folders for us which includes an <b>include</b> folder, a <b>src</b> folder, a <b>CMakeLists.txt</b> file & a <b>package.xml</b> file. So let's take a look at these generated files. <br><br>
        - If we open the **package.xml** file in our VS Code using the terminal command `code package.xml` we can see different fields that describe our package. You generally want to fill these fields such as version, description, maintainer and license, before releasing your package to the public. <br>
        - If we open the **CMakeLists.txt** file in our **VS Code** using the terminal command `code CMakeLists.txt`  - that opens it up in a new tab in VS Code → And after you trust it, just make sure that your language is set to **CMake** (At the bottom bar of VS Code). We will edit this file later in the course to tell the compiler about any dependencies we need for compiling our ROS package.

1. Create a folder named **scripts** inside our package folder **udemy_ros2_pkg -** for storing our **Python** scripts for this package. Also add a file named `__init__.py` inside it. For now, we do not need to add any code in this file. 
1. Now we are ready to **compile our ROS2 workspace**, using a build tool called **colcon**

    > 💡 If you have already installed <b>colcon</b> on your system, skip the next steps <b>9, 10, 11,12,13,14</b>

1. **Installing colcon →** Open up a browser and head over to the **ROS2 Humble Offcial Documentation** page → In the vertical navigation tab on the left side of the page - click on **Tutorials** Tab → This opens up the Tutorials page → Under heading **Beginner: Client libraries** - click on [Using colcon to build packages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) → This opens up the tutorials page for **Using `colcon` to build packages**. This will help us walk through the commands to install colcon and configure our terminal environment → Open a **new terminal →** In the browser, scroll down to **Prerequisites → Install colcon** → Copy the command `sudo apt install python3-colcon-common-extensions` and run it in terminal → You'll be prompted for your password. Go ahead - hit Y and enter → This installs the **colcon** into your Ubuntu OS 

1. In the browser, scroll down to the [Setup colcon_cd](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#id14) section.

    > 💡 Here we see two lines which will source the `colcon_cd` command so that we can use it in any new terminal we open. 
    The command `colcon_cd` allows you to quickly change the current working directory of your shell to the directory of a package. As an example `colcon_cd some_ros_package`
    would quickly bring you to the directory `~/ros2_install/src/some_ros_package`
    

1. Copy and paste the commands under [Setup colcon_cd](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#id14) section on the terminal and run them.
    
    ```bash
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
    echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
    ```
    
1. Next scroll down to **Setup `colcon` tab completion** section

    > 💡 This allows you to hit the tab key on your keyboard and autocomplete while writing your **colcon build** commands


1. Copy and paste the commands under **Setup `colcon` tab completion** section on the terminal and run them.
    
    ```bash
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
    ```

1. Close the old terminal.
1. Open a new terminal from **ros2_cpp_udemy_tutorial (ros2_py_udemy_tutorial)** workspace directory.
1. Even though we haven't created any new code yet, we can go ahead and build/compile our workspace. We can do this by running the `colcon build` command in the terminal from **ros2_cpp_udemy_tutorial (ros2_py_udemy_tutorial)** workspace directory.

    > 💡 Be sure to run this command from your workspace folder and not within your package folder.

    So now, our workspace has successfully been built.

    > 💡 Now, if we go over to our **Home Directory** → **Ros2_Workspaces** folder→ **ros2_cpp_udemy_tutorial (ros2_py_udemy_tutorial)**  workspace folder → We can see the newly created folders **build**, **install** and **log** in our workspace folder which are created by default when we build our workspace.
    

1. To add our newly created package **udemy_ros2_pkg** to our terminal environment - we need to source the `setup.bash` file present in the **install** folder of our **ros2_cpp_udemy_tutorial (ros2_py_udemy_tutorial)** workspace folder. So run the following command from the **ros2_cpp_udemy_tutorial (ros2_py_udemy_tutorial)**  workspace folder opened in your terminal -
    
    ```bash
    source install/setup.bash
    ```
    > 💡 Now our terminal is aware of our workspace, and all the packages within it - including our newly created ROS package **udemy_ros2_pkg**.
   
1. Run `ros2 pkg list`  command from the same terminal. This gives us a full list of all the available ros2 packages to us in our OS, but most notably the one we just created which is our **udemy_ros2_pkg** package.
    
    ![Untitled](Images/Chapter4/Untitled%207.png)
    

**Congratulations !**

You have successfully created your very own ROS2 Workspace.