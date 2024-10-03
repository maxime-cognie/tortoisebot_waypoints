# tortoisebot_waypoints
---

This ROS2 package contains an action server to move the RB1 robot to a desired goal position.
And integrate all the tests to validate the behaviour of this program.

---

## Test

To test the program just launch the test process:   
`colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+`  

 * This test verify that when a goal is send to the action server,
the resulting behavior of the action server is the one intended.
Thus it will verify that the robot final position is near enough to the goal position. 
By invoking `colcon test`, three test will be processed.    
The first one will send a goal to the action server with goal position (0,0).   
Then the second goal to reached is (0.5,0.5).   
Finally the last goal is (1.5,0).   
The expected result is that the first two test will pass as the position is reachable by the robot
and the last test should fail as the goal isn't reachable