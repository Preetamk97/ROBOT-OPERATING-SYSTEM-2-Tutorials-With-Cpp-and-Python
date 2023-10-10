# Chapter 2. Environment Setup

In this chapter, we are going to setup the **Humble Version of ROS2** and **VS Code** in our Ubuntu 20.04 LTS OS.

It is assumed/expected that you already have **Ubuntu 22.04 LTS  *or any later version of the Linux OS***  installed on your computer and are familiar with the basic Terminal Commands of Ubuntu.

# Installing ROS2 Humble on Ubuntu OS:

1. **Open Terminal :** Shortcut for opening terminal is (**Ctrl + Alt + T**)
2. **Right Click on the Terminal icon** → Select “**Add to Favorites**” → So that we can access it easily in the future.
3. **Run** `sudo apt update`
4. **Install Tilix →** **Run** `sudo apt install tilix` → **Hit Y** → **Enter**.
5. **Configure Tilix** → Open Tilix Terminal → Click on **the 3-line-button** beside search button at the top of Tilix Terminal Window → Select **Preferences** → Go to **Default** Tab → Go to **Command** Tab → Check “**Run the command as a login shell**” → You can go to **Color** tab and do some design customizations of your own for the terminal.
6. Close all the terminals.
7. Reopen Tilix Terminal. You can add more terminals using the “Add terminal right” & “Add terminal down” buttons at the top- left of Tilix Terminal Window. 
8. Go to the **ROS2 Humble Offical Documentation** Site. ([https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html))
9. Click on **Installation** → Click on “**Debian Packages**” under “**Ubuntu Linux - Jammy Jellyfish (22.04)**”. The steps below are borrowed from the **Ubuntu (Debian)** page.
10. **Set Locale** : Run the following commands on Tilix terminal one by one.
    
    ```
    locale  # check for UTF-8
    
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
    locale  # verify settings
    ```
    

1. **Troubleshooting For Future: (**[https://www.debugpoint.com/failed-connect-raw-githubusercontent-com-port-443/](https://www.debugpoint.com/failed-connect-raw-githubusercontent-com-port-443/)**)**
    
    Run `sudo nano /etc/hosts` 
    
    Then at the end of this file, add the IP address:
    
    `185.199.108.133 raw.githubusercontent.com`
    
    ![Untitled](Chapter%202%20Environment%20Setup%20b913f041a9ab406190521731ff40667a/Untitled.png)
    
    Save and close the file.
    
2. **Setup Sources:** Run the following commands on Tilix terminal one by one.
    
    ```
    sudo apt install software-properties-common
    
    sudo add-apt-repository universe
    
    sudo apt update && sudo apt install curl
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
    

1. **Install ROS 2 Packages:** Run the following commands on Tilix terminal one by one.

```
sudo apt update

sudo apt upgrade
#If you get any problem on running the above command.
#Try running "sudo dpkg --configure -a" to troubleshoot.
#Run "sudo apt upgrade" again.
```

- **Desktop Install (Recommended) - Used :** ROS, RViz, demos, tutorials:
    
    `sudo apt install ros-humble-desktop`
    
- **ROS-Base Install (Bare Bones) - Not Used**: Communication libraries, message packages, command line tools. No GUI tools.
    
    `sudo apt install ros-humble-ros-base`
    
- **Development tools - Not Used**: Compilers and other tools to build ROS packages
    
    `sudo apt install ros-dev-tools`
    
    Go with the **Desktop Install (Recommended).** 
    
    ROS2 Humble is now successfully installed on your system. 
    
    Cheers !
    
1. **Environment Setup (Sourcing the script):** You need to run this command everytime you open a new terminal to use ROS 2 on it.
    
    ```
    source /opt/ros/humble/setup.bash
    # You need to run this command everytime you open a new terminal to use Ros2 on it.
    # Replace ".bash" with your shell if you're not using bash
    # Possible values are: setup.bash, setup.sh, setup.zsh
    ```
    

1. **Try some commands to see if its working:**
    
    To check if you installed `ros-humble-desktop`  correctly as instructed above, you can try below commands.
    
    In one terminal, source the setup file and then run a C++ `talker`:
    
    ```
    source /opt/ros/humble/setup.bash
    ros2 run demo_nodes_cpp talker
    ```
    
    In another terminal source the setup file and then run a Python `listener` :
    
    ```
    source /opt/ros/humble/setup.bash
    ros2 run demo_nodes_py listener
    ```
    

You should see the `talker` saying that it’s `Publishing` messages and the `listener` saying `I heard` those messages. This verifies both the C++ and Python APIs are working properly.

Hooray!

1. **Automating the** `source /opt/ros/humble/setup.bash` **Command:**  Go to **Tutorials** page of ROS2 Humble Documentation → Under **Beginner: CLI tools,** click on **Configuring environment →** Scroll down to **Add sourcing to your shell startup script section → Execute the below given command in a Terminal for one time** - and you wont have to run the command `source /opt/ros/humble/setup.bash`  - every time you open a new terminal - - for using ROS 2 - ever again !
    
    `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
    
2. **To check the version of ROS installed - on Terminal** - run the below command:
    
    `printenv | grep -i ROS`
    
3. **Uninstall:**
    
    ```
    sudo apt remove ~nros-humble-* && sudo apt autoremove
    
    sudo rm /etc/apt/sources.list.d/ros2.list
    sudo apt update
    sudo apt autoremove
    # Consider upgrading for packages previously shadowed.
    sudo apt upgrade
    ```
    

# Installing VS Code on Ubuntu OS:

1. **Install** **VS Code** from **Ubuntu Software Center.**
2. **Install VS Code Extensions**:
    - **C/C++ Extension Pack** by Microsoft.
    - ****Python Extension Pack**** by Don Jayamanne.
    - **ROS** by Microsoft.
    - **Code Runner** by Jun Huan

1. Click on **Manage Gear Icon** at the left bottom corner ****of VS Code Window **→ Settings →** Extend the tab **Extensions →** Click on **ROS** (Under Extensions) → Write **humble** in the box under **Ros: Distro**

# Setting Up C++ Environment on Ubuntu OS:

1. First, check to see whether **GCC** is already installed on your Ubuntu system. To verify, open a Terminal window and enter the following command:
    
    `gcc -v`
    

1. If GCC isn't installed, run the following command from the terminal window to update the Ubuntu package lists. An out-of-date Linux distribution can sometimes interfere with attempts to install new packages.
    
    `sudo apt update`
    
    `sudo apt upgrade`
    
2. Next install the **GNU Compiler Tools** and the **GDB Debugger** with this command:
    
    `sudo apt-get install build-essential gdb`
    

# Setting Up Python Environment on Ubuntu OS:

You can download the Python package from the official Ubuntu repository. Here's how to do it:

1. Open up your terminal by pressing **Ctrl + Alt + T**.
2. **Upgrade and update Ubuntu to the latest version**

```bash
sudo apt update && sudo apt upgrade
```

1. Download and Install the latest version of Python:

```bash
sudo apt install python3
```

1. To check if **python** is successfully installed in your system, run the given command from any terminal:
    
    ```bash
    python3 --version
    ```