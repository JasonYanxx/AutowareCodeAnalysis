# Autoware&Gazebo simulation

### step 1 implement base version of autoware in docker

please refer to [AutowareUserGuide](AutowareUserGuide.md) for details

```shell
# current dir: docker/generic
sudo ./run.sh -c off -t 1.14.0 -b /home/yan/YanWorkSpace/Autoware1.14.0_src 
```

### step 2 complie autoware from source code

please refer to [AutowareUserGuide](AutowareUserGuide.md) for details

```shell
cd Autoware
mkdir src
wget -O autoware.ai.repos "https://raw.githubusercontent.com/Autoware-AI/autoware.ai/1.14.0/autoware.ai.repos"
vcs import src < autoware.ai.repos
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --cmake-args -DCMAKE_BUILD_TYPE=ReleaseO
```

### step 3 upgrade gazebo

- Please refer to the [gazebo offical tutorual](https://classic.gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0) for details. Only small changes are made.

- Note that there is another widely used [tutorial](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/56d11e899ce0a198e7206298b3aaaf8004f3a2c6/gazebo_upgrade.md?fileviewer=file-view-default) about upgrading gazebo. When I used this tutorial for gazebo upgrading, my later steps did not obtain a desired result. Therefore, be careful if you want to use this [tutorial](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/56d11e899ce0a198e7206298b3aaaf8004f3a2c6/gazebo_upgrade.md?fileviewer=file-view-default).

```shell
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

sudo apt-get update

sudo apt-get upgrade gazebo9

sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```

### step 4 download gazebo models

Download gazebo models in advance can speed up gazebo.

```shell
# run gazebo and then exit manually
gazebo

# download and copy gazebo_models into .gazebo/models
sudo cp -r /home/autoware/shared_dir/gazebo_models/ /home/autoware/.gazebo/models/
```

### step 5 start autoware&gazebo simulation

- video tutorial: 

[![ 4:15 / 8:33 autoware&gazebo simulation in docker : motion planning [tutorial step by step]](https://res.cloudinary.com/marcomontalbano/image/upload/v1656247334/video_to_markdown/images/youtube--2daavx76vDI-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=2daavx76vDI&t=160s " 4:15 / 8:33 autoware&gazebo simulation in docker : motion planning [tutorial step by step]")
