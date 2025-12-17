# Robot Setup

Also see the documentation for:

- [coord-dsl](https://github.com/secorolab/coord-dsl)
- [coord2b](https://github.com/rosym-project/coord2b)
- [eddie-ros](https://github.com/secorolab/eddie-ros)
- [eddie_pmu_control](https://github.com/secorolab/eddie_pmu_control)
- [eddie_description](https://github.com/secorolab/eddie_description)
- [orocos_kinematics_dynamics](https://github.com/secorolab/orocos_kinematics_dynamics)
- [robif2b](https://github.com/secorolab/robif2b)
- [ros2_kortex_vision](https://github.com/Kinovarobotics/ros2_kortex_vision)

## Workspace Setup

### Requirements

- Ubuntu 24.04
- ROS 2 Jazzy: Follow the instructions in the [ROS 2 installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- SSH keys set up for your GitHub account: Follow the instructions in the [GitHub SSH key guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) to generate a new SSH key and add it to your account.
- Zenoh router: Install it using the command from [the documentation](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html#installation-packages).

`rosdep` for installing dependencies. Install it with:

```bash
sudo apt install python3-rosdep
```

`rosdep` needs to be initialized and updated to use:

```bash
sudo rosdep init
rosdep update
```

### Setup Steps for ROS

Install the following dependencies:

```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base
```

1. Create a ROS workspace directory and navigate to it:

```bash
mkdir -p ~/r4s/src
cd ~/r4s
```

2. Inside the `r4s` directory clone the required repositories and the dependent packages for `eddie_description`:

```bash
vcs import src < src/eddie-ros/r4s.repos
vcs import src < src/eddie_description/dep.repos
```

3. Install the dependencies using `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. Copy `colcon.meta` to the `r4s` directory and build the workspace with `colcon`:

```bash
cp src/eddie-ros/colcon.meta .
colcon build
```

You can now source the workspace:

```bash
source install/setup.bash
```

### Setup steps for SOEM

To communicate with the robot base using EtherCAT, you need to install the SOEM library.

1. Clone the SOEM repository into your home directory and navigate to it:

```bash
cd ~/
git clone https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
```

2. Create a build directory and navigate to it:

```bash
mkdir build
cd build
```

3. Build the SOEM library with CMake and install it:

```bash
cmake -DBUILD_SHARED_LIBS=On -DCMAKE_INSTALL_PREFIX=/usr/local/ ..
sudo make install
```

## Robot startup & shutdown

To power on the robot follow these steps:

1. Start the robot base: Press the green button on side of the base for ca. 3 s.
2. Connect the wireless emergency stop button:
   - Make shure that the red emergency button on the top of the robot and on the remote are released.
   - The green light above the screen at the back of Eddie should be flashing, the red LED has to be off.
   - Press the green button on the remote, the green and red LEDs on the back of the robot start flashing alternately.
   - Press the green button on the back of the robot above the screen.
   - The green light above the screen should be on continously, if not press and release the emergency button on the remote and try again.
3. You can now turn on both arms by pressing the silver buttons near the connectors for 3 s. After some time the lights should turn green.
4. Now you can control both arms using the buttons on the arms wrists or with the Xbox controllers.
5. Connect your PC to the base using the ethernet cable.

To turn off the robot:

1. Make shure to hold onto the arms as they will fall down slowly. Then press the emergency button.
2. Now rest the arms on the sides of the base.
3. Press the button on the side of the base to turn it off. Make sure to always put the wireless emergency stop button back into its chargin dock.

## Check connection

1. Connect the ethernet and set the network settings on your machine.

   `Network` -> `IPv4` -> set to `Manual` with these values:

   - Address: 192.168.1.100
   - Netmask: 255.255.255.0
   - Gateway: 192.168.1.1

2. To get the network interfaces.

   ```bash
   ip a
   ```

   Get the name of the ethernet interface which shows the IP address under `inet`.

3. Check connection, this should print some information about the EtherCAT connection to the robot.

   ```bash
   sudo slaveinfo <eth interface>
   ```
