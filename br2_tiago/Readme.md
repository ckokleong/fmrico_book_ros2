# tiago simulated Setup

1. Download the repos with the tiago simulated

```
cd ~/catkin_ws/src
vcs import . < master_ros2/br2_tiago/third_parties.rosinstall
```

2. Try to install from packages as much dependencies as possible

```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Compile the workspace

```
cd ~/catkin_ws
catkin_make
```

4. Source the workspace or open a new terminal (if sources in .bashrc)

```
source ~/catkin_ws/devel/setup.bash
```

5. Let's launch the tiago simulated

On terminal 1:
```
roslaunch br2_tiago sim.launch
```

You should have listened a sound that indicates that the driver successfully communicated with the robot. Check permissions otherwise

On terminal 2:
```
rostopic list
```
You should be seeing the topics

