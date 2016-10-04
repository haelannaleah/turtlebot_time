# turtlebot_time
modifications:
    added remapping to the kobuki base launch file:
	<remap from="mobile_base/sensors/imu_data_raw" to="imu_data"/>
  	<remap from="odom" to="base/odom" />
    This allows the robot ekf package to work
    This happens outside this git repo on the user's machine.
