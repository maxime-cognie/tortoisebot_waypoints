set(TESTFILES tortoisebot_waypoints_test.cpp)
# Add a gtest executables
ament_add_gtest(${PROJECT_NAME}_test 
${TESTFILES}
	ENV
	GOAL_X=0.0
	GOAL_Y=0.0
	GOAL_YAW=0.0)
ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
target_link_libraries(${PROJECT_NAME}_test tortoisebot_waypoints_lib)

ament_add_gtest(${PROJECT_NAME}_success_test 
${TESTFILES}
	ENV
	GOAL_X=0.5
	GOAL_Y=0.5
	GOAL_YAW=0.0)
ament_target_dependencies(${PROJECT_NAME}_success_test ${dependencies})
target_link_libraries(${PROJECT_NAME}_success_test tortoisebot_waypoints_lib)

ament_add_gtest(${PROJECT_NAME}_abort_test 
${TESTFILES}
	ENV
	GOAL_X=-1.5
	GOAL_Y=0.0
	GOAL_YAW=0.0)
ament_target_dependencies(${PROJECT_NAME}_abort_test ${dependencies})
target_link_libraries(${PROJECT_NAME}_abort_test tortoisebot_waypoints_lib)

install(TARGETS
	${PROJECT_NAME}_test
	${PROJECT_NAME}_success_test
	${PROJECT_NAME}_abort_test
	# Install the generated test executable file to the path after DESTINATION
	DESTINATION lib/${PROJECT_NAME})