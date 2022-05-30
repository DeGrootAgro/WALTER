# Setup

# Manual

1. Set robot on baseplate and give it the base plate command
2. Control robot to drive around boundry while using slam
3. Specify no go zones

# Automatic

1. Robot will automatically fill in the rest of the map using slam
2. Save map to server
3. Create path using Boustrophedon Planner
4. Translate path to coordinates path

# Running

Robot will folow line until its collection is full.
In case the collection mechanism is full it will drive straight to the docking platform.
After unloading it will pick up where it left of.

The line following is done using the gps rtk coordinates.
While driving the robot uses its lidar to detect possible obstacles and navigate around them using teb_local_planner.
