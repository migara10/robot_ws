import math
import time
import threading
import serial
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState


# ---- 6DOF arm calibration values ----
JOINT_CONFIG = {
    'base_to_link1':   {'channel': 0, 'urdf_min': -1.5708, 'urdf_max': 1.5708, 'servo_min': 0,  'servo_max': 160, 'home': 90, 'invert': False},
    'link1_to_link2':  {'channel': 1, 'urdf_min': -1.5708, 'urdf_max': 1.5708, 'servo_min': 35, 'servo_max': 135, 'home': 90, 'invert': False},
    'link2_to_link3':  {'channel': 2, 'urdf_min': -1.5708, 'urdf_max': 1.5708, 'servo_min': 0,  'servo_max': 180, 'home': 90, 'invert': True},
    'link3_to_link4':  {'channel': 3, 'urdf_min': -1.5708, 'urdf_max': 1.5708, 'servo_min': 0,  'servo_max': 180, 'home': 90, 'invert': True},
    'link4_to_link5':  {'channel': 4, 'urdf_min': -1.5708, 'urdf_max': 1.5708, 'servo_min': 0,  'servo_max': 180, 'home': 90, 'invert': False},
    'link5_to_gripper':{'channel': 5, 'urdf_min': 0.0,     'urdf_max': 1.0,    'servo_min': 40, 'servo_max': 75,  'home': 40, 'invert': False},
}

JOINT_ORDER = [
    'base_to_link1', 'link1_to_link2', 'link2_to_link3',
    'link3_to_link4', 'link4_to_link5', 'link5_to_gripper'
]

SERIAL_PORT = '/dev/ttyACM0'   # Arduino Mega port
BAUD_RATE = 115200


def rad_to_servo_deg(joint_name, rad_value):
    cfg = JOINT_CONFIG[joint_name]
    home_servo = cfg['home']

    if cfg.get('invert', False):
        rad_value = -rad_value

    if rad_value >= 0:
        urdf_range = cfg['urdf_max']
        servo_range = cfg['servo_max'] - home_servo
        if urdf_range == 0:
            return home_servo
        ratio = rad_value / urdf_range
        servo_deg = home_servo + ratio * servo_range
    else:
        urdf_range = cfg['urdf_min']
        servo_range = home_servo - cfg['servo_min']
        if urdf_range == 0:
            return home_servo
        ratio = rad_value / urdf_range
        servo_deg = home_servo - ratio * servo_range

    return max(cfg['servo_min'], min(cfg['servo_max'], servo_deg))


class HardwareBridgeNode(Node):
    def __init__(self):
        super().__init__('arm_hardware_bridge')

        # Serial connection with LOW TIMEOUT to avoid blocking execution loop
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.001)
            time.sleep(2.0)
            self.get_logger().info(f'Arduino connected on {SERIAL_PORT}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.serial_conn = None

        self.current_positions = {j: 0.0 for j in JOINT_ORDER}
        self.lock = threading.Lock()
        cb_group = ReentrantCallbackGroup()

        self.arm_action_server = ActionServer(
            self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_arm_trajectory, callback_group=cb_group
        )
        self.gripper_action_server = ActionServer(
            self, FollowJointTrajectory, 'gripper_controller/follow_joint_trajectory',
            execute_callback=self.execute_gripper_trajectory, callback_group=cb_group
        )

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(0.02, self.publish_joint_states)  # Updated to 50 Hz for real-time monitoring

        self.get_logger().info('Arm hardware bridge ready (High-Speed Mode).')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.name = list(JOINT_ORDER)
            msg.position = [self.current_positions[j] for j in JOINT_ORDER]
        self.joint_state_pub.publish(msg)

    def send_serial_command(self, joint_positions: dict):
        servo_degrees = []
        for j in JOINT_ORDER:
            rad = joint_positions.get(j, self.current_positions[j])
            servo_degrees.append(f"{rad_to_servo_deg(j, rad):.1f}")

        command_str = ','.join(servo_degrees) + '\n'
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(command_str.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')

        with self.lock:
            for j, rad in joint_positions.items():
                self.current_positions[j] = rad

    def execute_arm_trajectory(self, goal_handle):
        return self._execute_trajectory(goal_handle, is_gripper=False)

    def execute_gripper_trajectory(self, goal_handle):
        return self._execute_trajectory(goal_handle, is_gripper=True)

    def _execute_trajectory(self, goal_handle, is_gripper):
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points

        if not points:
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = result.SUCCESSFUL
            return result

        self.get_logger().info(f'Executing trajectory with {len(points)} points for {joint_names}')

        start_time = time.perf_counter()

        for point in points:
            positions = dict(zip(joint_names, point.positions))
            
            # Absolute target execution timestamp calculated by MoveIt
            target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

            # Precision wait to match exact MoveIt trajectory point timing
            while (time.perf_counter() - start_time) < target_time:
                time.sleep(0.001)  # 1ms high-precision micro sleep

            self.send_serial_command(positions)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        return result


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()