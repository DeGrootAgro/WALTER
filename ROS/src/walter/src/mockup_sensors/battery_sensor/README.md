# GPS mockup sensor

This is a mockup of a sensor that can be used with WALTER.

# ROS parameters

Below are all the avaiable parameters listed. These can be set in the launchfile or using the rosparam cli.

## Battery usage

Battery usage is a parameter used to define how much battery is used each second. This value is a double. Defaults to 0.1
This can be set by using rosparam in the following manner:

```
rosparam set /walter_battery_sensor_instance/battery_usage 0.1
```

## Battery charging speed

Battery charging speed is a parameter used to define how much battery is charged each second. This value is a double. Defaults to 2.0
This can be set by using rosparam in the following manner:

```
rosparam set /walter_battery_sensor_instance/battery_charging_speed 2.0
```

## Battery action server topic

The topic the sensor data is published to is set with the namespace battery_topic. Value is a string. Defaults to /walter/mockup_sensors/battery_action_server_topic.

```
rosparam set /walter_battery_sensor_instance/action_server_topic
```

## Frequency

The frequency the mockup sensor sends data. THis parameter can be set with the gps_frequency namespace.
This value is a integer. Defaults to 1.

```
rosparam set /walter_battery_sensor_instance/battery_frequency 100
```

## Topic

The topic the sensor data is published to is set with the namespace battery_topic. Value is a string. Defaults to /walter/battery_percentage.

```
rosparam set /walter_battery_sensor_instance/battery_sensor_topic "/walter/battery_percentage"
```
