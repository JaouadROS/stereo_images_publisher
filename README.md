# stereo_images_publisher
This node reads stereo images from two different folders and publish them into two different topics. 

# Compiling
```
colcon build --packages-select publisher
```

# Running
```
ros2 launch publisher publish.launch.py
```

Check the launch file before running the package because in that file where you specify some arguments and also it will run rosbag by default.
