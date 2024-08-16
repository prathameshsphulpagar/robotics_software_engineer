1) to run the logarithmic spiral code task 1
```
ros2 launch module_2_assignment task1.launch.py 
```
we can set our own radius to follow open the launch folder->task1.launch.py file inside that set the "given_radius"

2) to run the task 2
launch file that run turtlesim simulation and node simultaneously
```
ros2 launch module_2_assignment task2.launch.py 
```
3) to run task 3
3 mid 3 robot run back and forth
```
ros2 launch module_2_assignment task3.launch.py 
```

4) to run task 4
we can set parameters inside the turtlebot "{'angularz': 0.1},{'linearx': 0.1}" inside the launch file 
after then it changes its values dynamically
```
ros2 launch module_2_assignment task4.launch.py 
```