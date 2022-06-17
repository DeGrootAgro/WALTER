# GPS mockup sensor

This is a mockup of a sensor that can be used with WALTER.

# ROS parameters

Below are all the avaiable parameters listed. These can be set in the launchfile or using the rosparam cli.
For setting it using cli the name of the node is required in front.

## Coordinates

- longitude
- latitude
- altitude

Are three different parameters used to define the starting coordinates of the gps mockup sensor. This value is a double. Defaults to 0.
These can be set by using rosparam in the following manner:

```
rosparam set /walter_gps_sensor_instance/longitude 4.897070
```

## Frequency

The frequency the mockup sensor sends data. THis parameter can be set with the gps_frequency namespace.
This value is a integer. Defaults to 1.

```
rosparam set /walter_gps_sensor_instance/frequency 100
```

## Topic

The topic the sensor data is published to is set with the namespace gps_topic. Value is a string. Defaults to gps/fix.

```
rosparam set /walter_gps_sensor_instance/topic "/gps/fix"
```
