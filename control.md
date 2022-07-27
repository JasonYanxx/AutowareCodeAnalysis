## Overview

## Pure pursuit method

### Functionality

- todo

### ROS topic and node

(1) Communication between nodes and related messages

![image](doc/control.png)

- `pure_pursuit` node receives `"/final_waypoints"` from planning module and `"/current_velocity", "/current_pose"` from localization module

- `pure_pursuit` node finds a specific point `X`in `/final_waypoints` and applies interpolation method to obtain **the position `next_target_position` to pursuit** between `X` and `current_pose`

- `pure_pursuit` node calculates the theoretical curvature from`current_pose` to `next_target_position` and the corresponding steering angle

- `pure_pursuit` node calculates the linear speed and angular speed in the `current_pose`

- `pure_pursuit` node calculates the longitudinal acceleration and centripetal acceleration to reach the `next_target_position`

- `pure_pursuit` node publishes `"/twist_raw"` (including linear speed and angular speed and `"/ctr_raw"`(including linear speed, longitudinal acceleration and steering angle)

- `twist_filter` node receives `"/twist_raw"` and `"/ctr_raw"`, and apply lateral limits on speed and acceleration on them, finally publishing `"/twist_cmd"` and `"/ctr_cmd"`

### Source code analysis

#### node `"/pure_pursuit"`

Three main files:

- src\autoware\core_planning\pure_pursuit\src\pure_pursuit_node.cpp

- src\autoware\core_planning\pure_pursuit\src\pure_pursuit_core.cpp

- src\autoware\core_planning\pure_pursuit\src\pure_pursuit.cpp

##### (1) waypoint_follower::PurePursuitNode 的构造函数

##### (2) getNextWaypoint()

- definition: void PurePursuit::getNextWaypoint()

- functionality: search a waypoint which goes far than the look_ahead_distance

- location: core_planning\pure_pursuit\src\pure_pursuit.cpp

- referred location: line 241 in pure_pursuit.cpp

![image](doc\getNextWaypoint.png)

##### (3) canGetCurvature()

- definition: bool PurePursuit::canGetCurvature(double* output_kappa)

- functionality: calculte the position of the next target point and the related curvature

- location: core_planning\pure_pursuit\src\pure_pursuit.cpp

- referred location: line 122 in pure_pursuit_core.cpp

After call `void PurePursuit::getNextWaypoint()` , there might be two major cases:

- case 1: the `next_waypoint` is the first or the last one in `current_waypoints_`
  
  ![image](doc\canGetCurvature_1.png)

- case 2: others
  
  - case 2.1
    ![image](doc\canGetCurvature_21.png)
  
  - case 2.2:
    ![image](doc\canGetCurvature_22.png)

Finally, call `double PurePursuit::calcCurvature(geometry_msgs::Point target) const` to calulate curvature based on Ackerman turning geometry.
![image](doc\canGetCurvature_3.png)

##### (4) calcRelativeCoordinate()

- definition: geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)

- functionality: calculation relative coordinate of point from current_pose frame

- location: common\libwaypoint_follower\src\libwaypoint_follower.cpp

- referred location: line 42 in pure_pursuit.cpp

```cpp
double PurePursuit::calcCurvature(geometry_msgs::Point target) const
{
  ...
  double numerator = 2 * calcRelativeCoordinate(target, current_pose_).y;
  ... 
}
```

**Note **that: `target` and `current_pose_` are under the same coordinate system `/map`.

The **essence **of this function: 

1. regarding `current_pose.position` as the origin and  `current_pose.orientation` to create a new coordinate system;

2. calculating the coordinate of `point_msg` in the new coordinate system.

**Reference**: [节点waypoint_replanner（上）CSDN博客](https://blog.csdn.net/xiaoxiao123jun/article/details/104768813)

##### (5) publishTwistStamped()

```cpp
void PurePursuitNode::publishTwistStamped(
  const bool& can_get_curvature, const double& kappa) const
{
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
  ts.twist.angular.z = can_get_curvature ? kappa * ts.twist.linear.x : 0;
  pub1_.publish(
  pub2_.publish(ccs);
}
```

whereas `computeCommandVelocity()` mainly use the velocity of the **first ** waypoint to obtain the `command_velocity`. The essence of `computeCommandVelocity()` is described below (Note: acc constrains are added by me ):

![image](doc\computeCommandVelocity.png)

this function publish information by topic `"/twist_raw"`:

- linear velocity

- angular velocity 

##### (6) publishControlCommandStamped()

```cpp
void PurePursuitNode::publishControlCommandStamped(
  const bool& can_get_curvature, const double& kappa) const
{
  if (!publishes_for_steering_robot_)
  {
    return;
  }

  autoware_msgs::ControlCommandStamped ccs;
  ccs.header.stamp = ros::Time::now();
  ccs.cmd.linear_velocity = can_get_curvature ? computeCommandVelocity() : 0;
  ccs.cmd.linear_acceleration = can_get_curvature ? computeCommandAccel() : 0;
  ccs.cmd.steering_angle =
    can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa) : 0;

  pub2_.publish(ccs);
}
```

whereas `computeCommandAccel()` mainly use the velocity of the **second ** waypoint to obtain the `command_acc`. The essence of `computeCommandAccel()` is described below (Note: acc constrains are added by me ):
![image](doc\computeCommandAccel.png)

this function publish information by topic `"/ctr_raw"`:

- linear velocity

- linear acceleration

- steering angle

#### node `"/twist_filter"`

对速度和角速度进行满足车辆动力学的检验，不能超过给定的横向加速度阈值，否则进行减速处理

对`"/twist_raw"`中的以下信息进行了车辆动力学的检验和限制，并进行了平滑处理，最后输出 `"/twist_cmd"`:

- angular velocity

对`"/ctr_raw"`中的以下信息进行了车辆动力学的检验和限制，并进行了平滑处理，最后输出 `"/ctr_cmd"`:

- steering angle

注意车辆参数的设置：需要与gazebo设置相同

#### node`"/twist_gate"`

`src\autoware\core_planning\twist_gate\src\twist_gate.cpp`

将`"/twist_cmd"` and `"/ctrl_cmd"`的等信息进行汇总，发布`"/vehicle_cmd"` ,该消息格式如下:

```
Header header
autoware_msgs/SteerCmd steer_cmd
autoware_msgs/AccelCmd accel_cmd
autoware_msgs/BrakeCmd brake_cmd
autoware_msgs/LampCmd lamp_cmd
autoware_msgs/Gear gear_cmd
int32 mode
geometry_msgs/TwistStamped twist_cmd
autoware_msgs/ControlCommand ctrl_cmd
int32 emergency
```

然而`"/twist_cmd"`and`"/ctrl_cmd"`等消息都是不同步的，即不会同时出现在`"/vehicle_cmd"`中，如下所示：

```cpp
void TwistGate::autoCmdTwistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& input_msg)
{
  health_checker_ptr_->CHECK_RATE("topic_rate_twist_cmd_slow", 8, 5, 1, "topic twist_cmd subscribe rate slow.");
  health_checker_ptr_->CHECK_MAX_VALUE("twist_cmd_linear_high", input_msg->twist.linear.x,
    DBL_MAX, DBL_MAX, DBL_MAX, "linear twist_cmd is too high");

  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->twist;

    checkState();
  }
}


void TwistGate::ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.ctrl_cmd = input_msg->cmd;

    checkState();
  }
}
```

由此可见`"/twist_gate"` 仅是为了gazebo处理消息方便而对不同的消息包装了一个外壳。

## Other method

- todo

## TODO

- analyze `core_planning\twist_gate\src\twist_gate_node.cpp`
