#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6'
        ]
        self._client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

    def move_to(self, positions, duration_sec=2.0):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec))

        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'Sending goal: {positions}')
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Movement complete.')
        return True


def main(args=None):
    rclpy.init(args=args)
    mover = ArmMover()

    poses = [
        [0.5, 0.3, -0.3, 0.0, 0.2, 0.0],
        [-0.5, -0.2, 0.4, 0.5, -0.3, 0.5],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]

    for pose in poses:
        mover.move_to(pose, duration_sec=3.0)

    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()