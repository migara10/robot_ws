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


# ---- ඔයාගේ 6DOF arm එකේ calibration values (physical servo limits) ----
# Format: joint_name -> (servo_channel, urdf_min_rad, urdf_max_rad, servo_min_deg, servo_max_deg, invert)
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
    """URDF joint value (radians) එකක් servo degree එකකට convert කරනවා.
    Home (0 rad) සැමවිටම servo home angle එකටම (90/40) map වෙනවා - asymmetric
    calibrated ranges (0-160, 35-135 වගේ) වලදීත් RViz simulation එකයි real
    arm එකයි align වෙන්න. 'invert' flag එකෙන් direction-reversed joints
    (elbow, wrist pitch වගේ) handle කරනවා."""
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

        # Serial connection to Arduino Mega
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2.0)  # Arduino reset වෙන්න සහ boot වෙන්න කාලය දෙනවා
            self.get_logger().info(f'Arduino connected on {SERIAL_PORT}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.serial_conn = None

        self.current_positions = {j: 0.0 for j in JOINT_ORDER}
        self.lock = threading.Lock()
        cb_group = ReentrantCallbackGroup()

        # Action servers - MoveIt controller config එකේ names/namespaces එකට match වෙන්න ඕන
        self.arm_action_server = ActionServer(
            self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_arm_trajectory, callback_group=cb_group
        )
        self.gripper_action_server = ActionServer(
            self, FollowJointTrajectory, 'gripper_controller/follow_joint_trajectory',
            execute_callback=self.execute_gripper_trajectory, callback_group=cb_group
        )

        # Joint state publishing (TF/RViz/MoveIt current-state monitoring වලට ඕන)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(0.05, self.publish_joint_states)  # 20 Hz

        self.get_logger().info('Arm hardware bridge ready.')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.name = list(JOINT_ORDER)
            msg.position = [self.current_positions[j] for j in JOINT_ORDER]
        self.joint_state_pub.publish(msg)

    def send_serial_command(self, joint_positions: dict):
        """joint_positions: {joint_name: radians} -> serial command string එකක් හදලා Arduinoට යවනවා"""
        servo_degrees = []
        for j in JOINT_ORDER:
            rad = joint_positions.get(j, self.current_positions[j])
            servo_degrees.append(round(rad_to_servo_deg(j, rad), 1))

        command_str = ','.join(str(d) for d in servo_degrees) + '\n'
        if self.serial_conn:
            try:
                self.serial_conn.write(command_str.encode())
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')
        else:
            self.get_logger().warn(f'No serial connection - would send: {command_str.strip()}')

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

        self.get_logger().info(f'Executing trajectory with {len(points)} points for {joint_names}')

        prev_time = 0.0
        for point in points:
            positions = dict(zip(joint_names, point.positions))
            self.send_serial_command(positions)

            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            wait_time = max(0.0, point_time - prev_time)
            prev_time = point_time
            time.sleep(wait_time)

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