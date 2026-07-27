Launch the turtlebot3 gazebo 
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

If the simulation breaks
```
pkill -f gazebo
```

Turtlebot3 teleoperation
```
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

Build the package
```
colcon build --symlink-install
```

Run the slam node: 
```
ros2 run slam slam_node
```

