ual_ecar_odometry
==================

Subscribes to encoders and publishes `odom` and tf.

ROS parameters:
* `ODOM_PUBLISH_RATE` (default=10 Hz)  [Hz]
* `left_K` and `right_K`: Constant to convert ticks to meters [meters/tick]
* `wheels_dist`: Distance between wheels [meters]
* `rawlog_asf_prefix`: file path (default=none)
* `rawlog_obs_prefix`: file path (default=none)

ROS graph
=======================
