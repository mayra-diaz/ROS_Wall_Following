# Project 2 D2

# Launch
To launch the project and see the robot following a wall you should run the following command:

```
roslaunch stingray_sim robot_following_wall_launch.launch
```

To turn on or off the training mode, you should assign a boolean to the training variable located in `robot_movement.py`. It is currently in testing mode with the 48states-4actions qtable. To change this, the actions list should be changed and the qtable.json file that is read.
