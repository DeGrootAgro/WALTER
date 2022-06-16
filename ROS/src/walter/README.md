# Setup

# Manual

1. Set robot on baseplate and give it the base plate command
2. Control robot to drive around boundry while using slam
3. Specify no go zones TODO

# Automatic

1. Robot will automatically fill in the rest of the map using slam TODO
2. Save map to server
3. Create path using full coverage path planner
4. Start autonomous robot

# Running

Robot will folow a line that covers the full map until its collection system is full.
In case the collection mechanism is full it will drive straight to the docking platform.
After unloading it will pick up where it left of.

The line following is done using the gps rtk coordinates.
While driving the robot uses its lidar to detect possible obstacles and navigate around them.

# TODO

- Build mockup sensor for collection system
- Build mockup sensor for battery percentage
- Include state_controller in walter
