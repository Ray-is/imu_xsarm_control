import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time
import subprocess
from pynput import keyboard

class ArmController(Node):

    """
    Init method: initializes robot, IMU data, and keyboard
    """
    def __init__(self):
        super().__init__('imu_arm_control_node')

        ### PARAMS ###
        self.alpha = 0.055                      # scaling factor for controlling the arm
        self.micro_move_time = 0.1              # how long each micro-move should take - should be short
        self.large_move_time = 2.0              # how long a large move should take - should be long
        self.imu_timeout = 0.5                  # if we haven't recieved data for this long, restart the IMU node
        self.imu_watchdog_period = 1.0          # how frequently to check if we need to restart the IMU node

        # Initialize robot
        self.bot = InterbotixManipulatorXS(
            robot_model='wx200',
            group_name='arm',
            gripper_name='gripper',
        )
        robot_startup()
        self.target_positions = [0.0 for _ in range(5)]
        self.reset_robot()

        # Initialize IMU subscription and watchdog
        self.last_msg_time = time.time()
        self.enable_imu = True
        self.imu_subscription = self.create_subscription(
            Imu, 
            'bno055/imu', 
            self.listener_callback, 
            qos_profile=10
        )
        self.imu_watchdog = self.create_timer(self.imu_watchdog_period, self.imu_watchdog_cb)

        # Initialize keyboard
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()


    """
    Callback that runs when a key is pressed
    if key is space, toggle imu enable
    """
    def on_key_press(self, key):
        try:
            if key == keyboard.Key.space:
                if self.enable_imu:
                    self.get_logger().info("Disabling IMU input.")
                    self.enable_imu = False
                else:
                    self.get_logger().info("Re-enabling IMU input.")
                    self.enable_imu = True
        except AttributeError:
            pass


    """
    Callback that runs whenever data from the IMU is received.
    Sets the joint positions according to the Angular Velocity of the IMU.
    Updates self.last_msg_time.
    """
    def listener_callback(self, msg : Imu):
        if self.enable_imu:
            self.target_positions[0] += self.alpha * msg.angular_velocity.z
            self.target_positions[2] += self.alpha * msg.angular_velocity.y
            self.target_positions[3] += self.alpha * msg.angular_velocity.x
            self.bot.arm.set_joint_positions(self.target_positions, moving_time=self.micro_move_time, blocking=False)
        self.last_msg_time = time.time()


    """
    Callback that runs at a rate specified by self.imu_watchdog_period
    When the IMU disconnects or times out, we restart the IMU driver.
    """
    def imu_watchdog_cb(self):
        if time.time() - self.last_msg_time > self.imu_timeout:
            self.get_logger().warn("Restarting IMU Driver...")
            subprocess.run(['pkill', '-f', 'bno055'])  # stop the IMU node, and it will automatically be rerun with the respawn


    """
    Destroys IMU subscription,
    Moves the robot to sleep pose,
    calls robot_shutdown().
    """
    def robot_shutdown(self):
        self.get_logger().info("Shutting down...")
        self.enable_imu = False
        self.listener.stop()
        time.sleep(0.01)  # just to be safe
        robot_shutdown()


    """
    Moves the robot to home pose,
    then sets the target position to all zeros (home pose),
    then enables IMU input
    """
    def reset_robot(self):
        self.get_logger().info("Resetting the robot...")
        self.bot.arm.go_to_home_pose(moving_time=self.large_move_time)
        for i in range(len(self.target_positions)): self.target_positions[i] = 0.0
        self.enable_imu = True




def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.robot_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
