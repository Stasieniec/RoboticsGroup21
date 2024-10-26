#!/usr/bin/env python

import rospy
from locobot_simulation.msg import LogicalImage
from std_msgs.msg import Float64
from moveit_msgs.msg import Grasp, GripperTranslation
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from math import pi
import sys
import moveit_commander


def is_object_pickable(model):
    # right now only look for red cube for testing purposes
    pickable_objects_names = ["red_cube"]
    return model.type in pickable_objects_names


def camera_image_callback(msg):
    global pickable_objects
    global planning_scene_objects
    global planning_scene_interface
    new_pickable_objects = []
    for model in msg.models:
        if is_object_pickable(model):
            new_pickable_objects.append(model)
        object_name = model.type
        if object_name not in planning_scene_objects:
            # Add object to planning scene
            if object_name == "red_cube":
                add_object_to_planning_scene(
                    planning_scene_interface, model, object_name
                )
            # Add it to the dictionary
            planning_scene_objects[object_name] = model.pose
    pickable_objects = new_pickable_objects


def command_head_tilt(tilt_value, publisher_tilt):
    msg = Float64()
    msg.data = tilt_value
    publisher_tilt.publish(msg)


def adjust_tilt_value(curr_tilt_value, direction, speed):
    curr_tilt_value += direction * speed
    if curr_tilt_value >= 0.8:
        direction = -1
    elif curr_tilt_value <= 0.0:
        direction = 1
    return curr_tilt_value, direction


def create_grasps(object_name, obj_pose, gripper_width):
    grasps = []

    grasp = Grasp()
    grasp.grasp_quality = 1.0
    grasp.allowed_touch_objects = [object_name]

    # Use the namespaced frame_id
    frame_id = "locobot/camera_link"

    # Grasp pose
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = frame_id
    grasp_pose.pose.position.x = obj_pose.x
    grasp_pose.pose.position.y = obj_pose.y
    grasp_pose.pose.position.z = obj_pose.z + 0.02
    grasp_pose.pose.orientation.x = 0.0
    grasp_pose.pose.orientation.y = 0.0
    grasp_pose.pose.orientation.z = 0.0
    grasp_pose.pose.orientation.w = 1.0
    grasp.grasp_pose = grasp_pose

    # Pre-grasp approach
    pre_grasp_approach = GripperTranslation()
    pre_grasp_approach.direction.header.frame_id = frame_id
    pre_grasp_approach.direction.vector.x = 0.0
    pre_grasp_approach.direction.vector.y = 0.0
    pre_grasp_approach.direction.vector.z = -0.4
    pre_grasp_approach.min_distance = 0.05
    pre_grasp_approach.desired_distance = 0.1
    grasp.pre_grasp_approach = pre_grasp_approach

    # Post-grasp retreat
    post_grasp_retreat = GripperTranslation()
    post_grasp_retreat.direction.header.frame_id = frame_id
    post_grasp_retreat.direction.vector.x = 0.0
    post_grasp_retreat.direction.vector.y = 0.0
    post_grasp_retreat.direction.vector.z = 0.8 # go up high
    post_grasp_retreat.min_distance = 0.3
    post_grasp_retreat.desired_distance = 0.5
    grasp.post_grasp_retreat = post_grasp_retreat

    # Pre-grasp posture (open gripper)
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = frame_id
    pre_grasp_posture.joint_names = ["right_finger", "left_finger"]

    pre_grasp_point = JointTrajectoryPoint()
    pre_grasp_point.positions = [-0.035, 0.035]
    pre_grasp_point.time_from_start = rospy.Duration(secs=1, nsecs=0)

    pre_grasp_posture.points.append(pre_grasp_point)
    grasp.pre_grasp_posture = pre_grasp_posture

    # Grasp posture (close gripper)
    grasp_posture = JointTrajectory()
    grasp_posture.header.frame_id = frame_id
    grasp_posture.joint_names = ["right_finger", "left_finger"]

    grasp_point = JointTrajectoryPoint()
    finger_extensions_normal = gripper_width
    grasp_point.positions = [
        -finger_extensions_normal + 0.00,
        finger_extensions_normal - 0.00,
    ]
    rospy.loginfo("Gripper sizes:")
    rospy.loginfo(grasp_point.positions)
    grasp_point.time_from_start = rospy.Duration(secs=1, nsecs=0)

    grasp_posture.points.append(grasp_point)
    grasp.grasp_posture = grasp_posture

    grasps.append(grasp)

    return grasps


def add_object_to_planning_scene(planning_scene_interface, model, object_name):
    frame_id = "locobot/camera_link"

    # Define the object's pose
    object_pose = PoseStamped()
    object_pose.header.frame_id = frame_id
    object_pose.pose = model.pose

    # Define the object's size (length, width, height)
    size = (model.size.x, model.size.y, model.size.z)

    # Add the box to the planning scene
    planning_scene_interface.add_box(object_name, object_pose, size)
    rospy.loginfo("Added object to planning scene: %s", object_name)


def main():
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("project_node", anonymous=True)

    rospy.sleep(7)  # Wait to ensure everything is initialized

    # Initialize the PlanningSceneInterface
    global planning_scene_interface
    planning_scene_interface = PlanningSceneInterface(ns="/locobot")
    global planning_scene_objects
    planning_scene_objects = {}

    # Initialize MoveGroupCommander
    group = MoveGroupCommander(name="interbotix_arm", robot_description="locobot/robot_description", ns="/locobot")
    group.set_planning_time(20.0)
    group.set_goal_position_tolerance(1000)
    group.set_goal_orientation_tolerance(1000)
    group.set_support_surface_name("ground_plane")
    group.allow_replanning(True)

    # Camera-related setup
    global pickable_objects
    pickable_objects = []
    rospy.Subscriber(
        "/gazebo/locobot/camera/logical_camera_image",
        LogicalImage,
        camera_image_callback,
    )

    # Head-related setup
    pub_tilt = rospy.Publisher(
        "/locobot/tilt_controller/command", Float64, queue_size=10
    )
    tilt_value = 0.5
    direction = 1

    current_object_target = None

    # ROS-related setup
    rate = rospy.Rate(10)  # 10 Hz update rate
    # Main loop
    current_state = "searching"  # Start by searching for objects
    while not rospy.is_shutdown():
        if pickable_objects and current_state == "searching":
            current_state = "start_picking"
            model = pickable_objects[0]
            object_name = model.type
            current_object_target = {"model": model, "object_name": object_name}
            rospy.loginfo("Set current_object_target to: %s", object_name)
            # Wait for the planning scene to update
            rospy.sleep(1)

        if current_state == "searching":
            # If there are no pickable objects, tilt head
            command_head_tilt(tilt_value, pub_tilt)
            tilt_value, direction = adjust_tilt_value(
                curr_tilt_value=tilt_value, direction=direction, speed=0.01
            )

        if current_state == "start_picking":
            if not current_object_target:
                rospy.logfatal(
                    "Pickup started but no object has been set as target"
                )
            else:
                # Create the grasps
                grasps = create_grasps(
                    current_object_target["object_name"],
                    current_object_target["model"].pose.position,
                    min(
                        current_object_target["model"].size.x,
                        current_object_target["model"].size.y,
                        current_object_target["model"].size.z,
                    ),
                )
                rospy.loginfo(
                    "Attempting to pick up object: %s",
                    current_object_target["object_name"],
                )
                result = group.pick(
                    current_object_target["object_name"], grasps
                )
                if result:
                    rospy.loginfo("Pick succeeded")
                else:
                    rospy.logwarn("Pick failed")
                current_state = "picking"

        if current_state == "picking":
            # Continue picking
            rospy.sleep(10)
            current_state = "searching"

        if current_state == "holding":
            pass
            # set pose

        rate.sleep()  # Control the update rate

    rospy.spin()  # Keep the node running


if __name__ == "__main__":
    main()
