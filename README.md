# imu_xsarm_control
ROS package that facilitates control of the interbotix xsarms with a bno055 IMU. Currently only compatible with the interbotix wx200.

I modified the bno055 launch file to respawn the node when it is killed, which I'll need move to this package's launch file
to avoid external modifications.
