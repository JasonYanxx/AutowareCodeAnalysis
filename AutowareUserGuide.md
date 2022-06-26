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

Now we could modify autoware source code in the host machine and compile it in the docker container.
