# ROS Workspace

This folder is the ROS workspace, which should be used to build the walter bot.

It contains a reference to other repositories needed to build the software. This way, we can track the exact version of the packages used in each release to ensure package compatibility.

# Fetch dependencies

Before building, you need to fetch this project's dependencies. The best way to do this is by using rosdep:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# run this in the ROS directory
rosdep install --from-paths src --ignore-src
```

# How to Build

Just build as any other ROS workspace it can be build using the command: `catkin make`
