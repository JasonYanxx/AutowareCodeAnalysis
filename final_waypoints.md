## Data structure of "/final_waypoints"

`"\final_waypoints"` is a part of final output of planning module. specifically, it is published by `velocity_set` node, whose source code is given below:

- path:core_planning\waypoint_planner\src\velocity_set\velocity_set.cpp

```cpp
void changeWaypoints(const VelocitySetInfo& vs_info, const EControl& detection_result, int closest_waypoint,
                     int obstacle_waypoint, const ros::Publisher& final_waypoints_pub, VelocitySetPath* vs_path)
{
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE)
  { // STOP for obstacle/stopline
    ...
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints()); // publish "/final_waypoints"
  }
  else if (detection_result == EControl::DECELERATE)
  {  // DECELERATE for obstacles
    ...
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else
  {  // ACCELERATE or KEEP
    ...
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_set");
  ...
  while (ros::ok())
  {
    ros::spinOnce();
    ...
    changeWaypoints(vs_info, detection_result, closest_waypoint,
                    obstacle_waypoint, final_waypoints_pub, &vs_path);
    ...
    vs_path.resetFlag();
    loop_rate.sleep();
  }
  return 0;
}
```

whereas `vs_path` in `vs_path->getTemporalWaypoints()` is an instance of the class `class VelocitySetPath`, whose definition is given below:

```cpp
class VelocitySetPath
{
 private:
  ...
  autoware_msgs::Lane temporal_waypoints_;
  ...
 public:
  ...
  autoware_msgs::Lane getTemporalWaypoints() const
  {
    return temporal_waypoints_;
  }
  ...
};
```

where the return type of `getTemporalWaypoints()`  is `autoware_msgs::Lane`, i.e., the message type published by `final_waypoints_pub` in the `velocity_set` node is `autoware_msgs::Lane`, whose definition is given below:

- messages\autoware_msgs\msg\Lane.msg

```
Header header
int32 increment
int32 lane_id
Waypoint[] waypoints

uint32 lane_index
float32 cost
float32 closest_object_distance
float32 closest_object_velocity
bool is_blocked
```

whereas `Waypoint.msg` is given below:

- messages\autoware_msgs\msg\Waypoint.msg

```
# global id
int32 gid 
# local id
int32 lid
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped twist
DTLane dtlane
int32 change_flag
WaypointState wpstate

uint32 lane_id
uint32 left_lane_id
uint32 right_lane_id
uint32 stop_line_id
float32 cost
float32 time_cost

# Lane Direction
# FORWARD        = 0
# FORWARD_LEFT       = 1
# FORWARD_RIGHT      = 2
# BACKWARD        = 3 
# BACKWARD_LEFT      = 4
# BACKWARD_RIGHT    = 5
# STANDSTILL       = 6
uint32 direction
```

whereas `DTLane.msg` and `WaypointState.msg` is given below:

- messages\autoware_msgs\msg\DTLane.msg

```
float64 dist
float64 dir
float64 apara
float64 r
float64 slope
float64 cant
float64 lw
float64 rw
```

- messages\autoware_msgs\msg\WaypointState.msg

```
int32 aid
uint8 NULLSTATE=0

# lanechange
uint8 lanechange_state

# bilinker
uint8 steering_state
uint8 STR_LEFT=1
uint8 STR_RIGHT=2
uint8 STR_STRAIGHT=3
uint8 STR_BACK=4

uint8 accel_state

uint8 stop_state
# 1 is stopline, 2 is stop which can only be released manually.
uint8 TYPE_STOPLINE=1
uint8 TYPE_STOP=2

uint8 event_state
uint8 TYPE_EVENT_NULL = 0
uint8 TYPE_EVENT_GOAL = 1
uint8 TYPE_EVENT_MIDDLE_GOAL = 2
uint8 TYPE_EVENT_POSITION_STOP = 3
uint8 TYPE_EVENT_BUS_STOP = 4
uint8 TYPE_EVENT_PARKING = 5
```

## Connection with the control module

The most important data related to control module in `"/final_waypoints"` is `Waypoint[] waypoints` , in which the most important components are:

```
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped twist
```

The control module computes the **next target point** based on these `waypoints` and calculates the **longitudinal acceleration and centripetal acceleration** to reach the target.  Please refer to [`control.md`](control.md) for details.

## TODO

- what is the meaning of each variable in those `.msg` files?
  - actually, only `Waypoint[] waypoints`  matters
