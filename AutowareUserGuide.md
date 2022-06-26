# Autoware User Guide

- Author: Jason Yan
- Env: **Ubuntu 20.04 LTS**
- Create Date: 30 Apr 2022
- Update Date: 30 Apr 2022

## Build and Run Autoware on Docker

### 1. Install docker and get autoware from github

Please strictly follow the instructions in [<Docker Installation>](https://github.com/Autoware-AI/autoware.ai/wiki/docker-installation) step by step. Here is the summary:

```shell
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install docker-ce
```

Now we can get autoware from github:

```shell
# Clone the docker repository and move into the generic docker folder.
git clone https://github.com/Autoware-AI/docker.git
cd docker/generic
```

### 2.  Build the docker image for autoware (optional)

Please refers to this [tutorial](https://github.com/JasonYanxx/AutowareRepo/blob/main/AutowareUserGuide.md). 

### 3. Run the docker image for autoware

(1) (**Default**) If you do not build the docker image for autoware in [Step 2](# 2.  Build the docker image for autoware (optional)), you can directly pull the docker image for autoware from docker hub:

```shell
# current dir: docker/generic
sudo ./run.sh -c off -t 1.14.0
```

The output will be like:

```
Using options:
    ROS distro: melodic
    Image name: autoware/autoware
    Tag prefix: 1.14.0
    Cuda support: off
    Pre-release version: off
    UID: <1000>
Launching autoware/autoware:1.14.0-melodic
Unable to find image 'autoware/autoware:1.14.0-melodic' locally
1.14.0-melodic: Pulling from autoware/autoware
```

It will pull the docker image `autoware/autoware:1.14.0-melodic` from the docker hub, and automatically create and enter the container of `autoware/autoware:1.14.0-melodic` after the pulling process.

To see the detail use of `run.sh`, you can use the following command:

```shell
./run.sh --help
```

(2) If you have build the  docker image for autoware in [Step 2](# 2.  Build the docker image for autoware (optional)), you can enter the container of your built docker image for autoware:

```shell
sudo ./run.sh -c off -i autoware -t 1.14.0 -b ~/YanWorkSpace/Autoware
```

`-b ~/YanWorkSpace/Autoware` results in that `~/YanWorkSpace/Autoware` in the host machine will b e mounted as a volume on the container under `/home/autoware/Autoware` , which means that `~/YanWorkSpace/Autoware` in the host machine and `/home/autoware/Autoware` in the container will share the storage space. Of course, `~/YanWorkSpace/Autoware` can be replaced by any aviablable location in the hos machine.

The output will be like:

```
Using options:
    ROS distro: melodic
    Image name: autoware
    Tag prefix: 1.14.0
    Cuda support: off
    Autoware Home: /home/yan/YanWorkSpace/Autoware
    Pre-release version: off
    UID: <1000>
Launching autoware:1.14.0-melodic-base
To run a command as administrator (user "root"), use "sudo <command>".
See "man sudo_root" for details.
```

### 4. Explore the autoware container

Assuming we have launch a docker image for autoware based  on the first method in [Step 3](# 3. Run the docker image for autoware ), we can explore what it has in the container. 

The output by executing `ls` is: 

```
# parent dir: /home/autoware
Autoware  shared_dir
```

where `shared_dir` shares the storage space with the `/root/shared_dir` in the host machine. However,normal users are not able to view and manipulate files in `/root/shared_dir`. Therefore it is needed to use a `root` user for such purpose:

```shell
# in terminator of the host machine
sudo passwd root # then set a new password for root user by instruction
su # change to root user
```

In addition, `Autoware` folder store the essential code and data of autoware, and the output by excuting `ls` under `/home/autoware/Autoware` is: 

```
autoware.ai.repos build install log src
```

To test the function of autoware, you can refer to this [tutorial](https://github.com/Autoware-AI/autoware.ai/wiki/ROSBAG-Demo). Note that `autoware.ai` in the tutorial is exactly `Autoware` here.

## Implement The Code from CJC

### 1. Run Autoware in the container

```shell
# current dir: /home/autoware
cd Autoware
source install/setup.bash
roslaunch runtime_manager runtime_manager.launch
```

### 2. Load data in the `Autoware Runtime Manager (ARM)`

Please strictly follow the instructions in `<GNSS欺骗程序使用说明文档_220126.doc>` step by step.

Note that it is better to **wait the loading process of point cloud for several minutes**, otherwise, the matching process is  likely to fail.

### 3. Run the `gnssSpoofing` code

Please strictly follow the instructions in `<GNSS欺骗程序使用说明文档_220126.doc>` step by step. However, some modification should be performed.

(1) Delete the useless `CMakeLists.txt' in the ros workspace

Below is the file structure of the provided code. Before executing `caktin_make` , there are two `CMakeLists.txt` in the ros workspace: one in the root directory of the ros workspace `gnss-spoofing/1`, another in the `gnss-spoofing/1/src`.

```
|---gnss-spoofing
    |---1
        |---CMakeLists.txt
        |---src
            |---CMakeLists.txt
            |---test_pkg
```

According to the rules of `catkin_make`, it is illegal to have a `CMakeLists.txt` in the root directory of the ros workspace, which will result in an error below when  run `catkin_make`. Therefore, we need to delete `CMakeLists.txt` under `gnss-spoofing/1` .

```
The specified base path "/home/autoware/shared_dir/code/gnss_spoofing/1" contains a CMakeLists.txt but "catkin_make" must be invoked in the root of workspace
```

(2) Add the missing ros package `novate_msgs`

If we directly execute `catkin_make`, an error is likely to occur:

```
/home/autoware/shared_dir/code/gnss_spoofing/1/src/test_pkg/src/subscribe_node_cjc.cpp:15:10: fatal error: novatel_msgs/INSPVAX.h: No such file or directory 

#include <novatel_msgs/INSPVAX.h> 

          ^~~~~~~~~~~~~~~~~~~~~~~~ 

compilation terminated. 
```

Through locating the error in the `gnss-spoofing/1/src/test_pkg/src/subscribe_node_cjc.cpp`, `novatel_msgs/INSPVAX.h` is served as a head file. 

```c++
// line 1-16 of subscribe_node_cjc.cpp
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <novatel_msgs/INSPVAX.h> // location related to the above error
#include <std_msgs/String.h>
```

It tells that a ros package `novate_msgs` is needed and not included in current ros workspace. We can download it in the github `https://github.com/ros-drivers/novatel_span_driver`.  To include this ros package, we just need to clone the whole package in the root director of the ros workspace: 

```shell
# current dir: gnss-spoofing/1/src
git clone https://github.com/ros-drivers/novatel_span_driver.git
```

Then we need to include `novatel_msgs` in the `gnss-spoofing/1/src/test_pkg/CMakeLists.txt` and `gnss-spoofing/1/src/test_pkg/package.xml`:

```cmake
# line 10-15 of CMakeLists.txt
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  novatel_msgs # modification
)
```

```xml
<!-- line 51-63 of package.xml -->
<buildtool_depend>catkin</buildtool_depend>
<build_depend>roscpp</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>tf</build_depend>
<!-- modification -->
<build_depend>novatel_msgs</build_depend>
<build_export_depend>roscpp</build_export_depend>
<build_export_depend>std_msgs</build_export_depend>
<build_export_depend>tf</build_export_depend>
<!-- modification -->
<build_export_depend>novatel_msgs</build_export_depend>
<exec_depend>roscpp</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>tf</exec_depend>
<!-- modification -->
<exec_depend>novatel_msgs</exec_depend>
```

With these two modifications, we are able to execute `catkin_make` without errors. 

### 4. Visualize the result

Please strictly follow the instructions in `<GNSS欺骗程序使用说明文档_220126.doc>` step by step. Here is the result:

![image](doc/gnssSpoofingResult.png)

## Debug with VSCode

please follow the [tutorial](https://github.com/JasonYanxx/VS_Code_ROS). 

To enter the modified image with vscode debugging support

```shell
sudo ./run_vscode-debug.sh -c off -t 1.14.0
```

the output info is:

```
Using options:
    ROS distro: melodic
    Image name: autoware/autoware
    Tag prefix: 1.14.0
    Cuda support: off
    Pre-release version: off
    UID: <1000>
Launching autoware/autoware:1.14.0-melodic-vscode-debug
```

## Use the base version for developing

To enter the base version of autoware

```shell
sudo ./run.sh -c off -t 1.14.0 -b /home/yan/YanWorkSpace/Autoware1.14.0_src 
```

the output info is:

```
Using options:
    ROS distro: melodic
    Image name: autoware/autoware
    Tag prefix: 1.14.0
    Cuda support: off
    Autoware Home: /home/yan/YanWorkSpace/Autoware1.14.0_src 
    Pre-release version: off
    UID: <1000>
Launching autoware/autoware:1.14.0-melodic-base
```

However, there is no source code in this image since it only provide essential environment for autoware developing. To build autoware from source in this image, please refer to this [tutorial](https://github.com/Autoware-AI/autoware.ai/wiki/Source-Build). To summarize:

```shell
$ cd Autoware
$ mkdir src
$ wget -O autoware.ai.repos "https://raw.githubusercontent.com/Autoware-AI/autoware.ai/1.14.0/autoware.ai.repos"
$ vcs import src < autoware.ai.repos
$ rosdep update
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=ReleaseO
```

The modified image is published into the local repo with name `autoware/autoware:1.14.0-melodic-base-yan`

To launch it:

```shell
sudo ./run_base.sh -c off -t 1.14.0 -b /home/yan/YanWorkSpace/Autoware1.14.0_src 
```

Now we could modify autoware source code in the host machine and compile it in the docker container.

## Local path planning and following using Gazebo with Autoware

### overview

[![Local path planning and following using Gazebo with Autoware](https://res.cloudinary.com/marcomontalbano/image/upload/v1654182491/video_to_markdown/images/youtube--YwNVhZdf91A-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=YwNVhZdf91A&t=43s "Local path planning and following using Gazebo with Autoware")

### Steps

- If it is the first practice,  please [upgrade](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/56d11e899ce0a198e7206298b3aaaf8004f3a2c6/gazebo_upgrade.md?fileviewer=file-view-default) `gazebo` first. ([Reference]([GitHub - yukkysaito/vehicle_sim](https://github.com/yukkysaito/vehicle_sim)))

- In `Simulation` page
  
  - click `RViz` and load the configuration file `/home/autoware/.rviz/default.rviz`
  
  - click `Gazebo` to launch Gazebo and wait until it is successfully loaded

- In `Setup` page
  
  - click `TF, Vehicle Model` in order

- In `Map` page
  
  - set path of `Point CLoud`: /home/autoware/shared_dir/gazebo_data/vehicle_sim/mcity/pointcloud_map/pointcloud_map.pcd, and then click `Point CLoud`
  - set path of `TF`: /home/autoware/shared_dir/gazebo_data/vehicle_sim/mcity/tf/tf.launch, and then click `TF`

- In `Sensing` page
  
  - tick `voxel_grid_filter` 
    
    ![image](C:\Users\Administrator\Desktop\WorkSpace\AutowareRepo\doc\voxel_grid_filter.png)

- in `Computing` page
  
  - click `app` of `ndt_matching` and tick `Intial Pos`, and then tick `ndt_matching`
    
    ![image](C:\Users\Administrator\Desktop\WorkSpace\AutowareRepo\doc\ndt_matching.png)
  
  - tick `vel_pose_connect`  
    
    ![image](C:\Users\Administrator\Desktop\WorkSpace\AutowareRepo\doc\vel_pose_connect.png)
  
  - click `app`of `waypoint loader`and set the csv path as: /home/autoware/shared_dir/gazebo_data/vehicle_sim/mcity/path/saved_waypoints.csv
    
    ![image](C:\Users\Administrator\Desktop\WorkSpace\AutowareRepo\doc\waypoint_loader.png)
  
  - tick `waypoint_loader` and go to `rviz` to wait for the loading of gloabl waypoints
  
  - tick `lane_navi, lane_rule, lane_stop, lane_select` in order
  
  - tick `astar_void, velocity_set, pure_pursuit, twist_filter` in oder and go to rviz to see if the vehicle starts to move 
