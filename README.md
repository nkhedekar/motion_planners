# motion_planners
A ROS package with python implementations of a few common planning algorithms

# Example Usage
Clone, build and source:

`cd catkin_ws/src`

`git clone https://github.com/nkhedekar/motion_planners.git`

`cd ..`

`catkin build`

`source devel/setup.bash`

Run:
(On seperate terminals)

`roscore`

`rosrun motion_planners rrt_star.py`

`rosrun motion_planners master_planner.py`

`rviz`

Add a MarkerArray to rviz and set the appropriate topic

**Note: Collision checking and sample space need to be implemented as required**
