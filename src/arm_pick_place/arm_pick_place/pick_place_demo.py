import time
import rclpy
from rclpy.logging import get_logger

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

OBJECT_X = 0.30
OBJECT_Y = 0.15
OBJECT_Z = 0.08
OBJECT_SIZE = 0.20

DROP_X = 0.30
DROP_Y = -0.15
DROP_Z = 0.08

GRIPPER_OPEN = 0.4
GRIPPER_CLOSED = 0.0

GRIPPER_LENGTH = 0.06   # link5 -> gripper_link Z offset (URDF eken)
PRE_GRASP_HEIGHT_OFFSET = 0.10


def get_plan_params(arm_robot):
    params = PlanRequestParameters(arm_robot, "ompl")
    params.planning_pipeline = "ompl"
    params.planning_time = 5.0
    params.planning_attempts = 10
    params.max_velocity_scaling_factor = 0.4
    params.max_acceleration_scaling_factor = 0.3
    return params


def add_dummy_object(planning_scene_monitor, logger):
    collision_object = CollisionObject()
    collision_object.header.frame_id = "base_link"
    collision_object.id = "dummy_object"

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [OBJECT_SIZE, OBJECT_SIZE, OBJECT_SIZE]

    box_pose = Pose()
    box_pose.position.x = OBJECT_X
    box_pose.position.y = OBJECT_Y
    box_pose.position.z = OBJECT_Z
    box_pose.orientation.w = 1.0

    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(box_pose)
    collision_object.operation = CollisionObject.ADD

    with planning_scene_monitor.read_write() as scene:
        scene.apply_collision_object(collision_object)
        scene.current_state.update()

    logger.info(f"Dummy object added at ({OBJECT_X}, {OBJECT_Y}, {OBJECT_Z})")


def remove_dummy_object(planning_scene_monitor, logger):
    collision_object = CollisionObject()
    collision_object.header.frame_id = "base_link"
    collision_object.id = "dummy_object"
    collision_object.operation = CollisionObject.REMOVE

    with planning_scene_monitor.read_write() as scene:
        scene.apply_collision_object(collision_object)
        scene.current_state.update()

    logger.info("Dummy object removed from planning scene (now held by gripper).")


def move_gripper(arm_robot, gripper_group, position, logger, label, plan_params):
    gripper_group.set_start_state_to_current_state()

    robot_model = gripper_group.get_start_state().robot_model
    goal_state = RobotState(robot_model)
    goal_state.set_joint_group_positions("gripper", [position])

    gripper_group.set_goal_state(robot_state=goal_state)
    plan_result = gripper_group.plan(single_plan_parameters=plan_params)
    if plan_result:
        logger.info(f"Gripper {label}...")
        arm_robot.execute(plan_result.trajectory, controllers=[])
    else:
        logger.error(f"Gripper {label} plan failed!")
    time.sleep(1.0)


def move_to_pose(arm_robot, arm_group, x, y, z, logger, label, plan_params):
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.position.x = x
    pose_goal.pose.position.y = y
    pose_goal.pose.position.z = z
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.707
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.707

    arm_group.set_start_state_to_current_state()
    arm_group.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link5")
    plan_result = arm_group.plan(single_plan_parameters=plan_params)

    if plan_result:
        logger.info(f"Moving: {label}...")
        arm_robot.execute(plan_result.trajectory, controllers=[])
        time.sleep(0.5)
        return True
    else:
        logger.error(f"Plan failed: {label}")
        return False


def main():
    rclpy.init()
    logger = get_logger("pick_place_demo")

    arm_robot = MoveItPy(node_name="pick_place_demo")
    time.sleep(3.0)  # joint_states ready wenna kalayak denawa
    arm_group = arm_robot.get_planning_component("arm")
    gripper_group = arm_robot.get_planning_component("gripper")
    planning_scene_monitor = arm_robot.get_planning_scene_monitor()

    plan_params = get_plan_params(arm_robot)

    add_dummy_object(planning_scene_monitor, logger)
    time.sleep(1.0)

    move_gripper(arm_robot, gripper_group, GRIPPER_OPEN, logger, "opening", plan_params)

    move_to_pose(arm_robot, arm_group, OBJECT_X, OBJECT_Y,
                 OBJECT_Z + PRE_GRASP_HEIGHT_OFFSET + GRIPPER_LENGTH,
                 logger, "pre-grasp (above object)", plan_params)

    move_to_pose(arm_robot, arm_group, OBJECT_X, OBJECT_Y,
                 OBJECT_Z + (OBJECT_SIZE / 2) + 0.02 + GRIPPER_LENGTH,
                 logger, "descend to grasp", plan_params)

    move_gripper(arm_robot, gripper_group, GRIPPER_CLOSED, logger, "closing (grasp)", plan_params)

    remove_dummy_object(planning_scene_monitor, logger)
    time.sleep(0.5)

    move_to_pose(arm_robot, arm_group, OBJECT_X, OBJECT_Y,
                 OBJECT_Z + PRE_GRASP_HEIGHT_OFFSET + GRIPPER_LENGTH,
                 logger, "lift", plan_params)

    move_to_pose(arm_robot, arm_group, DROP_X, DROP_Y,
                 DROP_Z + PRE_GRASP_HEIGHT_OFFSET + GRIPPER_LENGTH,
                 logger, "move to drop location", plan_params)

    move_to_pose(arm_robot, arm_group, DROP_X, DROP_Y,
                 DROP_Z + (OBJECT_SIZE / 2) + 0.02 + GRIPPER_LENGTH,
                 logger, "descend to drop", plan_params)

    move_gripper(arm_robot, gripper_group, GRIPPER_OPEN, logger, "opening (release)", plan_params)
    logger.info("Object released.")

    move_to_pose(arm_robot, arm_group, DROP_X, DROP_Y,
                 DROP_Z + PRE_GRASP_HEIGHT_OFFSET + GRIPPER_LENGTH,
                 logger, "lift after drop", plan_params)

    logger.info("Pick-and-place sequence complete!")
    rclpy.shutdown()


if __name__ == "__main__":
    main()