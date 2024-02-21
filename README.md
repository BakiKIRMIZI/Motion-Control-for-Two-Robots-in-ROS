# Design of Motion Control for Two Robots in ROS Turtlesim Environment

Turtlesim will be used [for detailed information see http://wiki.ros.org/turtlesim and "ROS Robot Programming Series" https://www.youtube.com/@FieldRoboLab]. 

There will be two robots in the Turtlesim environment. [You can add a new robot to the environment using the "spawn" service in Turtlesim.] The state vector of the robots is represented by [x, y, θ]T. Here, x and y represent the position of the robot in the 2D environment, and θ represents the orientation angle. Robot-1 will start at [0,0,0]T, while Robot-2 will start at a randomly determined state within the workspace.
A motion controller (ROS node) will be written for each robot.

---

+ The motion controller for Robot-1 should be designed to follow Robot-2 from a specified distance. Robot-1 can access the state information of Robot-2. PID control will be used as the controller.
+ The motion controller for Robot-2 should be designed according to the following motion strategy:
  + Robot-2 moves at a constant speed in the current orientation angle.
  + When Robot-2 reaches the boundary of the workspace, it will randomly choose one of the following two strategies to move: i) It moves like the reflection of light from a mirror. That is, the angle of arrival equals the angle of departure. ii) It randomly selects the turning angle.
+ The (linear and angular) velocities of the robots and the following distance should be read from a JSON file initially as parameters.
