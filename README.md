# ROS-follower
A simple ROS package where a swarm of robots follow a leader. Both the leader and the robots are visualized as cubes. Done as a part of the Software Architechture for Robotics course at Ecole Centrale de Nantes.

![demo](imgs/output.gif)
---

### Usage:

  1. Clone the repository onto your local machine : `git clone https://github.com/sampreets3/ROS-follower.git`
  1. Add the contents of the **src** directory into your the src directory in your ROS workspace
  1. Open up a terminal, navigate to your ROS workspace, and run `catkin build` or `catkin_make`, depending on which system you used to build your workspace
  1. Once the packages have been built, open a new terminal and source the setup file of your ROS workspace : `source ~/<your-ros-workspace>/devel/setup.bash`
  1. Run the launch file in the master_and_dog package : `roslaunch master_and_dog master_and_dog.launch` Using the Tab key to autocomplete is a good indication that everything is in order
  1. In another terminal, run `rviz` to see the visual output.

If everything is well-configured, you will see a blue marker that is tracing a circular trajectory, and two red markers that are following the blue marker. The blue marker is the master, and the two red markers are the follower swarm.
---

### Planned Updates:

  * Introduce individual URDF models for leader and follower robots
  * Make provisions for other trajectories
  * Introduce a hardware-based model, complete with driver code
---

### Contact:

You can contact me at my email address: `sampreets3@gmail.com`
