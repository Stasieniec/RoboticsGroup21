# Group 21 Robotics 2024 final project

## Introduction

The objective of this project was to implement a control pipeline for a robot arm to sort objects based on their color — red or blue — by moving each object to the corresponding colored mat. The robot needed to detect objects, identify their colors, and manipulate them appropriately.

Our team focused on the detection and manipulation of simple, graspable objects (small cube, small ball, and small cylinder). We successfully programmed the robot to detect these small objects using a logical camera, reach for a detected object, and grasp it using MoveIt for motion planning.

## Individual Contributions

- **Jules:**
  - something
- **Stanisław:**
  - something
- **Aditya:**
  - something
- **Fleur:**
  - something


## Solution

### Conceptual Overview

Our solution comprises several key components:

1. **Perception Setup:**
   - Subscribed to the logical camera topic `/gazebo/locobot/camera/logical_camera_image` to receive data about detected objects.
   - Implemented a callback function to process the camera data, filter for pickable objects, and update the planning scene with detected objects.

2. **Motion Planning with MoveIt:**
   - Configured MoveIt for controlling the robot arm, allowing for complex motion planning and execution.
   - Used `MoveGroupCommander` to plan and execute motions for reaching and grasping objects.
   - Defined grasp poses and pre-grasp/post-grasp motions for the gripper to ensure successful grasping.

3. **Grasping Mechanism:**
   - Created custom grasp configurations for the small red cube (and potentially other small objects).
   - Set up pre-grasp and grasp postures for the gripper, including finger positions adjusted to the object's size.

4. **State Machine Logic:**
   - Implemented a state machine with states like `searching`, `start_picking`, `picking`, and `holding` to manage the robot's actions.
   - In the `searching` state, the robot adjusts the camera tilt to scan for objects.
   - Upon detecting an object, the robot transitions to the `start_picking` state and attempts to grasp the object.

5. **Head Tilt Mechanism:**
   - Controlled the robot's head tilt to maximize the field of view of the logical camera, improving object detection capabilities.
   - Adjusted the tilt angle dynamically to scan the environment effectively.

### Workflow

1. **Initialization:**
   - Start the simulation environment and initialize ROS nodes and MoveIt components.
   - Set up subscribers and publishers for the camera and robot actuators.

2. **Object Detection:**
   - Continuously receive data from the logical camera.
   - Filter and identify pickable objects based on predefined criteria (e.g., object type and color).

3. **Planning Scene Update:**
   - Add detected objects to the MoveIt planning scene for collision avoidance during motion planning.

4. **Grasp Planning:**
   - Generate grasp configurations tailored to the detected object's size and shape.
   - Define pre-grasp and grasp postures for the gripper.

5. **Motion Execution:**
   - Use MoveIt to plan a path to the object's grasp pose.
   - Execute the motion plan to approach and grasp the object.

6. **Post-Grasp Actions:**
   - Once the object is grasped, plan and execute motions to place the object on the corresponding colored mat.
   - Transition back to the `searching` state to detect and manipulate the next object.

### Challenges and Limitations

- **Detection Limitations:**
  - The logical camera only provides data for objects within its field of view, requiring the robot to adjust its camera to detect all objects.

- **Picking up the objects:**
  - Our implementation does not handle picking up the grasped objects, due to time constraints.

- **Interaction with Complex Objects:**
  - Our implementation currently handles only simple, graspable objects.
  - Interaction with non-graspable objects (big ball and big cylinder) is not implemented due to time constraints.

## Documentation

### Files and Directories

- **`group21_project.launch`:**
  - The launch file that sets up the simulation environment and starts the necessary nodes for the project.
- **`pickplacetest.py`:**
  - The main Python script containing the ROS node that controls the robot arm for detecting and picking objects.

### Code Explanation

#### Launch File: `group21_project.launch`

This launch file initializes the simulation environment with the required parameters:

- **Simulation Environment:**
  - Launches the Gazebo simulation with the specified world containing all objects.
  - Sets up the robot model, camera settings, and trajectory controllers.

- **MoveIt Configuration:**
  - Starts the MoveIt move group for motion planning.
  - Configures the robot's arm and gripper for use with MoveIt.

- **Visualization:**
  - Optionally launches RViz for visualization of the robot and planning scene.

- **Node Execution:**
  - Starts the `pickplacetest.py` node, which contains the main logic for object detection and manipulation.

#### Main Script: `pickplacetest.py`

- **Imports and Initialization:**
  - Imports necessary ROS and MoveIt libraries for robot control and message handling.
  - Initializes `moveit_commander` and the ROS node.

- **Global Variables:**
  - `pickable_objects`: List to store detected pickable objects.
  - `planning_scene_interface`: Interface to interact with the planning scene in MoveIt.
  - `planning_scene_objects`: Dictionary to keep track of objects added to the planning scene.

- **Camera Callback Function:**
  - `camera_image_callback(msg)`:
    - Processes data from the logical camera.
    - Filters detected models to identify pickable objects (e.g., "red_cube").
    - Adds new objects to the planning scene if not already present.

- **Head Control Functions:**
  - `command_head_tilt(tilt_value, publisher_tilt)`:
    - Commands the robot's head to tilt to a specified value.
  - `adjust_tilt_value(curr_tilt_value, direction, speed)`:
    - Adjusts the tilt angle of the robot's head to search for objects efficiently.

- **Grasp Creation Function:**
  - `create_grasps(object_name, obj_pose, gripper_width)`:
    - Generates a list of grasp configurations for the specified object.
    - Defines grasp poses, pre-grasp approaches, and post-grasp retreats.
    - Configures the gripper's pre-grasp and grasp postures based on the object's dimensions.

- **Planning Scene Update Function:**
  - `add_object_to_planning_scene(planning_scene_interface, model, object_name)`:
    - Adds detected objects to the MoveIt planning scene as collision objects.
    - Ensures the robot's motion planning accounts for these objects to avoid collisions.

- **Main Control Loop:**
  - Implements a state machine to manage the robot's actions:
    - **`searching`:**
      - The robot tilts its head to scan the environment for objects.
    - **`start_picking`:**
      - Upon detecting an object, prepares to pick it up by generating grasp configurations.
    - **`picking`:**
      - Attempts to execute the pick action using MoveIt.
      - If successful, transitions to the next state; otherwise, logs a warning.
    - **`holding`:**
      - Placeholder for future implementation, such as moving the object to the colored mat.
  - Includes error handling and logging to provide feedback on the robot's actions and any issues encountered.

- **ROS Spin and Shutdown:**
  - Keeps the node running and responsive to ROS callbacks.
  - Ensures proper shutdown and cleanup when the node is terminated.

### How to Run the Code

1. **Setup:**
   - Ensure all necessary packages are built in the `catkin_ws` workspace.
   - Source the workspace: `source ~/catkin_ws/devel/setup.bash`.

2. **Launch the Simulation:**
   - Run the launch file: `roslaunch locobot_moveit group21_project.launch`.

3. **Monitor the Robot:**
   - Optionally, launch RViz to visualize the robot and planning scene.
   - Observe the robot's actions as it searches for and attempts to pick up objects.

### Dependencies

- **ROS (Robot Operating System):**
  - Compatible with ROS Noetic, depending on the system setup.
- **MoveIt:**
  - Used for motion planning and execution.
- **Gazebo:**
  - Simulation environment for testing the robot in a virtual world.
- **Python:**
  - The script is written in Python and requires Python 3.8.10, consistent with the ROS version.
- **Robot Packages:**
  - `locobot_simulation` and `locobot_moveit` packages for robot description and MoveIt configurations.

### Notes

- **Camera Field of View:**
  - Ensure the robot's head is properly initialized to avoid issues with the camera's field of view.
  - The head tilt mechanism is crucial for detecting objects within the environment.

- **Gripper Configuration:**
  - Verify the robot's gripper dimensions and limits to match the object sizes for successful grasping.
  - Adjust the gripper's pre-grasp and grasp postures as needed for different objects.

- **Collision Avoidance:**
  - Keeping the planning scene updated with collision objects is essential to prevent the robot from planning paths that collide with objects or the environment.

- **Future Improvements:**
  - Implement interaction with complex, non-graspable objects (big ball and big cylinder) by developing pushing or knocking strategies.
  - Enhance the state machine to include actions like moving the object to the corresponding mat after picking.
