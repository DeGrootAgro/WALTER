# GPS mockup sensor

This is a mockup of a sensor that can be used with WALTER.

# ROS parameters

All parameters follow the ros param naming convention by starting with the namespace

```
/walter/mockup_sensors/
```

Followed by the parameter. Below are all the avaiable parameters listed. These can be set in the launchfile or using the rosparam cli.

## Coordinates

- longitude
- latitude
- altitude

Are three different parameters used to define the starting coordinates of the gps mockup sensor. This value is a double. Defaults to 0.
These can be set by using rosparam in the following manner:

```
rosparam set /walter/mockup_sensors/longitude 4.897070
```

## Frequency

The frequency the mockup sensor sends data. THis parameter can be set with the gps_frequency namespace.
This value is a integer. Defaults to 1.

```
rosparam set /walter/mockup_sensors/gps_frequency 100
```

## Topic

The topic the sensor data is published to is set with the namespace gps_topic. Value is a string. Defaults to gps/fix.

```
rosparam set /walter/mockup_sensors/gps_topic "/gps/fix"
```
