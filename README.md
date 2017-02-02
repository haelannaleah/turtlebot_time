# turtlebot_time

Note: April tags work best when the black background is surrounded by a white foreground.
It should have the black and white in the middle, then the black border, then a white border around the black border.

Note: When adding data to be fused to robot_localization, you must both publish to the topic specified in the launch file (in this case, 'apriltags') AND publish a transform from that frame to the world frame.
TODO: figure out the transform!