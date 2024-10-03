# tortoisebot_waypoints
---

This ROS1 package contains an action server to move the RB1 robot to a desired goal position.
And integrate all the tests to validate the behaviour of this program.

---

## Test

To test the program just launch the test process: 
`rostest tortoisebot_waypoints waypoints_test.test --reuse-master`  

 * This test verify that when a goal is send to the action server, the resulting behavior of the action server is the one intended.
Thus it will verify that the robot final position is near enough to the goal position 
To test different goal position and orientation you just have to adjust the according values inside the test launch file:
`/test/waypoints_test.test`