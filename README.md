# rviz_marker_visualizer
## About

This a **ROS package** that publishes markers in RViz.

## Installation

In your workspace/src:

```
git clone https://github.com/Amos-Chen98/rviz_marker_visualizer.git
cd ..
catkin build
```

## Usage

```
roslaunch roslaunch rviz_marker_visualizer run_visualizer.launch
```

Configurable parameters are in the `run_visualizer.launch`:

```xml
<?xml version="1.0"?>
<launch>
    <arg name="pose_topic" default="/move_base_simple/goal" />
    <arg name="marker_type" default="sphere" /> <!-- line OR sphere -->

    <node pkg="rviz_marker_visualizer" type="visualizer_node.py" name="visualizer_node" output="screen">
        <remap from="pose_topic" to="$(arg pose_topic)" />
        <param name="marker_type" value="$(arg marker_type)" />
    </node>

    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find rviz_marker_visualizer)/rviz/rviz_config.rviz" />
</launch>
```

* `pose_topic`: You are supposed to set this argument to the topic of your robot's pose.

* `marker_type`: Should be "sphere" or "line".
  * "sphere": It will draw discrete markers of each point received.
  * "line": It will draw lines connecting each point received.
