### ROS Node: reem_move_arm_action
Copyright (c) 2013, David Butterworth, PAL Robotics S.L. 
<br>
<br>
An Action Server to move the left or right arm to a specific pose.

Moves the arm directly using the FollowJointTrajectory Action, or by calling move_arm for collision-free planning.
<br>

<br>
This node is required for some key scenarios:

 - The robot's arms are initially in a state of collision with the body, and move_arm will often refuse to plan a move. <br>
   Therefore, this node can be called to do a direct-move from the 'home' position to the 'init' position.

 - For tabletop manipulation, it is useful to be able to easily request a 'pre-grasp' pose for the arms, where they are above the table.
<br>

<br>
Poses: <br>
 - home (all joints zero) <br>
 - init&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;(arms at side of body) <br>
 - home_to_init (arms at side of body)  un-safe move, no collision planning <br>
 - surrender <br>
 - hands_up1 <br>
 - hands_up2 <br>
 - hands_up3 <br>
 - hand_forward <br>
 - surrender2 <br>
 - elbow_back (pre-grasp position for tabletop manipulation) <br>
 - grasp_lift (lift object from table)  un-safe move, no collision planning
<br>

<br>
ToDo: - This node is still just a binary for testing. Turn it into a proper ROS Action Server!

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; - Sometimes move_arm fails during trajectory filtering, a maximum of 2 times. <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Need to add a 3x retry loop to this node, so this node doesn't fail.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; - Make joint poses easily configurable via YAML file.


&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Modify move_arm node to output which links are in collision, for debugging purposes, and for higher level planning.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; - Currently poses are specified for 7 joints, and right_arm_torso is padded with the extra joints. Could do this in reverse, so <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; poses are defined with all joints, and the left_arm will ignore the extra joint positions.
<br>

<br>
**Required ROS packages:** <br>
reem_common     (DavidB-PAL fork, not yet merged with Master 15/3/13) <br>
reem_simulation (DavidB-PAL fork, not yet merged with Master 15/3/13) <br>
reem_arm_navigation (DavidB-PAL fork, not yet merged with Master 15/3/13) 

reem_manipulation_worlds <br>
reem_head_scan_action <br>
pointcloud_snapshotter <br>
reem_perception_launch
<br>

<br>
**Usage:** <br>

To test the arm poses:

$ roslaunch reem_gazebo reem_empty_world.launch  <br>
$ roslaunch reem_arm_navigation reem_arm_navigation.launch  <br>
First, move both arms away from collision with the body:  <br>
$ rosrun reem_move_arm_action move --arm=left --pose=home_to_init <br>
$ rosrun reem_move_arm_action move --arm=right --pose=home_to_init <br>
Move the arm into a specific pose: <br>
$ rosrun reem_move_arm_action move --arm=left --pose=hand_forward
<br>

<br>
To test moving the arm, using the OpenNI sensor for collision-free planning:

$ export USE_RGBD_SENSOR=true <br>
$ roslaunch reem_manipulation_worlds reem_high_white_table_manipulation.launch <br>
$ roslaunch reem_perception_launch perception_movearm.launch sim:=true use_snapshotter:=true <br>
Build a Collision Map: <br>
$ rosrun reem_head_scan_action scan_table <br>
Move both arms away from collision with the body: <br>
$ rosrun reem_move_arm_action move --arm=left --pose=home_to_init <br>
$ rosrun reem_move_arm_action move --arm=right --pose=home_to_init <br>
Move the arm into a specific pose, whilst avoiding the table: <br>
$ rosrun reem_move_arm_action move --arm=left --pose=elbow_back




