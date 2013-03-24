/*
 *  An Action Server to move the left or right arm to a specific pose.
 *
 *  Moves the arm directly using the FollowJointTrajectory Action,
 *  or by calling move_arm for collision-free planning.
 *
 *
 *  This node is required for some key scenarios:
 *  
 *   - The robot's arms are initially in a state of collision with the body,
 *     and move_arm will often refuse to plan a move.
 *     Therefore, this node can be called to do a direct-move from the 'home'
 *     position to the 'init' position.
 *  
 *   - For tabletop manipulation, it is useful to be able to easily request a
 *     'pre-grasp' pose for the arms, where they are above the table.
 *  
 *  
 *  Poses:
 *   - home (all joints zero)
 *   - init         (arms at side of body)
 *   - home_to_init (arms at side of body)  un-safe move, no collision planning
 *   - surrender
 *   - hands_up1
 *   - hands_up2
 *   - hands_up3
 *   - hand_forward
 *   - surrender2
 *   - elbow_back (pre-grasp position for tabletop manipulation)
 *   - grasp_lift (lift object from table)  un-safe move, no collision planning
 *  
 *  
 *  ToDo: - This node is still just a binary for testing. Turn it into a proper ROS Action Server!
 *  
 *        - Sometimes move_arm fails during trajectory filtering, a maximum of 2 times.
 *          Need to add a 3x retry loop to this node, so this node doesn't fail.
 *  
 *        - Make joint poses easily configurable via YAML file.
 *  
 *  
 *        - Modify move_arm node to output which links are in collision, for debugging
 *          purposes, and for higher level planning.
 *  
 *        - Currently poses are specified for 7 joints, and right_arm_torso is padded
 *          with the extra joints. Could do this in reverse, so poses are defined with
 *          all joints, and the left_arm will ignore the extra joint positions.
 *
 *  Author: David Butterworth
 */

/*
 * Copyright (c) 2013, David Butterworth, PAL Robotics S.L.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Boost, command line arguments
#include <boost/program_options.hpp>

// Define arm poses:
//
// Currently there are 11 poses, describing the 7 joints
// in either arm.
//
// To add a new pose, add its name to *vinit[],
// increase the size of pose_joint_angles[11][7],
// and add a new row of joint angles to that same array.

// end()
template<typename T, size_t N>
T * end(T (&ra)[N])
{
    return ra + N;
}
const char *vinit[] =
{
    "home","home_to_init","init","surrender","hands_up1","hands_up2",  "hands_up3","hand_forward","surrender2","elbow_back","grasp_lift"
};
std::vector<std::string> pose_names(vinit, end(vinit));

double pose_joint_angles[11][7] =
{
    // home
    {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
    },

    // home_to_init, arms at side of body (un-safe move, no collision planning)
    {
        -0.4,
        0.1,
        -0.1,
        0.6109,
        0.0,
        0.0,
        0.0
    },

    // init
    {
        -0.4,
        0.1,
        -0.1,
        0.6109,
        0.0,
        0.0,
        0.0
    },

    // surrender
    {
        0.0,
        1.5708,
        1.5708,
        1.5708,
        0.0,
        0.0,
        0.0
    },

    // hands_up1
    {
        1.2,
        0.0,
        -1.6,
        1.4,
        -0.5,
        0.0,
        0.0
    },

    // hands_up2
    {
        0.9,
        0.2,
        -1.6,
        1.3,
        -0.5,
        0.0,
        0.0
    },

    // hands_up3
    {
        1.7,
        -0.1,
        -1.7,
        1.5708,
        0.0,
        0.0,
        0.0
    },

    // hand_forward
    {
        0.86,
        0.0,
        0.0,
        0.74,
        -0.07,
        0.0,
        0.0
    },

    // surrender2
    {
        0.0,
        0.86,
        1.5,
        2.26,
        0.0,
        0.0,
        0.0
    },

    // elbow_back, pre-grasp position for arm
    {
        1.6,
        2.05,
        -1.64,
        2.20,
        1.3,
        0.0,
        0.0
    },

    // grasp_lift, lift object from table (un-safe move, no collision planning)
    {
        1.85,
        2.05,
        -1.64,
        1.85,
        1.3,
        0.0,
        0.0
    }

    /*
      // New pose name
      {
        ,
        ,
        ,
        ,
        ,
        ,

      },

    */

};

class REEMMoveArmAction
{
public:
    // Constructor
    REEMMoveArmAction(std::string desired_arm_, std::string desired_pose_);

private:
    std::string desired_pose_;
    std::string desired_arm_;
};

// Constructor
REEMMoveArmAction::REEMMoveArmAction(std::string desired_arm_, std::string desired_pose_)
{
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_("~");

    printf("Starting REEM MoveArm Action for '%s' arm, with goal pose '%s'...  \n", desired_arm_.c_str(), desired_pose_.c_str() );

    // Wait for valid clock
    // (important when using simulated clock)
    if (!ros::Time::waitForValid(ros::WallDuration(5.0)))
    {
        ROS_FATAL("Timed-out waiting for valid time.");
        return;
    }

    // Based on specified arm, set the MoveArm server name & Planning Group name
    std::string movearm_server_name;
    std::string planning_group_name;
    std::string joint_trajectory_server_name;
    if (desired_arm_.compare("left") == 0)
    {
        movearm_server_name = "move_left_arm";
        planning_group_name = "left_arm";
        joint_trajectory_server_name = "left_arm_controller/follow_joint_trajectory";
    }
    else if (desired_arm_.compare("right") == 0)
    {
        movearm_server_name = "move_right_arm_torso";
        planning_group_name = "right_arm_torso";
        //movearm_server_name = "move_right_arm";
        //planning_group_name = "right_arm";
        joint_trajectory_server_name = "right_arm_torso_controller/follow_joint_trajectory";
    }

    // Get index number of joint angles for this pose
    unsigned int pose_index = 0;
    for (unsigned int i=0; i < pose_names.size(); i++)
    {
        //printf("Pose %d %s \n", i, pose_names[i].c_str() );
        if (pose_names[i].compare(desired_pose_) == 0)
        {
            pose_index = i;
            break;
        }
    }

    // For special poses, move without collision checking/planning
    //if (desired_pose_.compare("home_to_init") == 0)
    if ( (desired_pose_.compare("home_to_init") == 0) || (desired_pose_.compare("grasp_lift") == 0) )
    {
        // Wait for Joint Trajectory Action Client, for sending motion planning requests
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joint_trajectory_client(joint_trajectory_server_name, true);
        ROS_INFO("Waiting for /%s...", joint_trajectory_server_name.c_str() );
        if (!joint_trajectory_client.waitForServer(ros::Duration(5.0)))
        {
            ROS_ERROR("Timed-out waiting for the FollowJointTrajectory Action Server.");
            return;
        }
        ROS_INFO("Connected to FollowJointTrajectory Action Server.");

        // Prepare the joint trajectory command
        control_msgs::FollowJointTrajectoryGoal traj_command;
        //traj_command.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.01);
        traj_command.trajectory.points.resize(1); // trajectory has 1 single waypoint

        traj_command.trajectory.joint_names.push_back("arm_"+desired_arm_+"_1_joint");
        traj_command.trajectory.joint_names.push_back("arm_"+desired_arm_+"_2_joint");
        traj_command.trajectory.joint_names.push_back("arm_"+desired_arm_+"_3_joint");
        traj_command.trajectory.joint_names.push_back("arm_"+desired_arm_+"_4_joint");
        traj_command.trajectory.joint_names.push_back("arm_"+desired_arm_+"_5_joint");
        traj_command.trajectory.joint_names.push_back("arm_"+desired_arm_+"_6_joint");
        traj_command.trajectory.joint_names.push_back("arm_"+desired_arm_+"_7_joint");

        traj_command.trajectory.points[0].positions.resize(7);
        traj_command.trajectory.points[0].positions[0] = pose_joint_angles[pose_index][0];
        traj_command.trajectory.points[0].positions[1] = pose_joint_angles[pose_index][1];
        traj_command.trajectory.points[0].positions[2] = pose_joint_angles[pose_index][2];
        traj_command.trajectory.points[0].positions[3] = pose_joint_angles[pose_index][3];
        traj_command.trajectory.points[0].positions[4] = pose_joint_angles[pose_index][4];
        traj_command.trajectory.points[0].positions[5] = pose_joint_angles[pose_index][5];
        traj_command.trajectory.points[0].positions[6] = pose_joint_angles[pose_index][6];

        // if using the right arm torso controller, add the 2 extra joint names
        if (movearm_server_name.compare("move_right_arm_torso") == 0)
        {
            traj_command.trajectory.joint_names.push_back("torso_1_joint");
            traj_command.trajectory.joint_names.push_back("torso_2_joint");
            traj_command.trajectory.points[0].positions.resize(9);
            traj_command.trajectory.points[0].positions[7] = 0.0;
            traj_command.trajectory.points[0].positions[8] = 0.0;
        }

        // Set the velocity
        traj_command.trajectory.points[0].time_from_start = ros::Duration(2.0); // set variable  , 1.0 is too low and violates velocity constraints

        // Send FollowJointTrajectory Action request
        if (nh_.ok())
        {
            bool finished_within_time = false;
            joint_trajectory_client.sendGoal(traj_command); // goal

            // FollowJointTrajectory is good, it returns aborted or succeeded, whereas the old JointTraj action immediately
            // returns a result of pending!
            finished_within_time = joint_trajectory_client.waitForResult(ros::Duration(15.0));
            if (!finished_within_time)
            {
                joint_trajectory_client.cancelGoal();
                ROS_ERROR("Timed-out attempting joint trajectory.");
            }
            else
            {
                actionlib::SimpleClientGoalState state = joint_trajectory_client.getState();
                bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
                if (success)
                {
                    ROS_INFO("Joint Trajectory Action finished. Result code: %s", state.toString().c_str() );
                }
                else
                {
                    ROS_ERROR("Joint Trajectory Action failed with error code: %s", state.toString().c_str() );
                    ROS_ERROR("(if Action was ABORTED, maybe the velocity (goal time) was too high and joint constraints were violated)");
                }
            }

        }

    }

    // Move to arm pose, using collision planning
    else
    {
        // Wait for MoveArm Action Client, for sending motion planning requests
        actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm_client(movearm_server_name, true);
        ROS_INFO("Waiting for /%s...", movearm_server_name.c_str() );
        if (!move_arm_client.waitForServer(ros::Duration(5.0)))
        {
            ROS_ERROR("Timed-out waiting for the move_arm action server.");
            return;
        }
        ROS_INFO("Connected to move_arm action server.");

        // Prepare motion plan request with joint-space goal
        arm_navigation_msgs::MoveArmGoal goal;
        goal.motion_plan_request.group_name = planning_group_name;
        std::vector<std::string> names(7);
        names[0] = "arm_"+desired_arm_+"_1_joint";
        names[1] = "arm_"+desired_arm_+"_2_joint";
        names[2] = "arm_"+desired_arm_+"_3_joint";
        names[3] = "arm_"+desired_arm_+"_4_joint";
        names[4] = "arm_"+desired_arm_+"_5_joint";
        names[5] = "arm_"+desired_arm_+"_6_joint";
        names[6] = "arm_"+desired_arm_+"_7_joint";

        // if using the right arm torso controller, add the 2 extra joint names
        if (movearm_server_name.compare("move_right_arm_torso") == 0)
        {
            names.resize(9);
            names[7] = "torso_1_joint";
            names[8] = "torso_2_joint";
        }

        goal.motion_plan_request.num_planning_attempts = 1;
        goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

        goal.motion_plan_request.planner_id= std::string("");
        goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
        goal.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

        for (unsigned int i = 0 ; i < goal.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
        {
            goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
            goal.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
            goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.05;
            goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.05;
        }

        // Get joint angles for this pose
        goal.motion_plan_request.goal_constraints.joint_constraints[0].position = pose_joint_angles[pose_index][0];
        goal.motion_plan_request.goal_constraints.joint_constraints[1].position = pose_joint_angles[pose_index][1];
        goal.motion_plan_request.goal_constraints.joint_constraints[2].position = pose_joint_angles[pose_index][2];
        goal.motion_plan_request.goal_constraints.joint_constraints[3].position = pose_joint_angles[pose_index][3];
        goal.motion_plan_request.goal_constraints.joint_constraints[4].position = pose_joint_angles[pose_index][4];
        goal.motion_plan_request.goal_constraints.joint_constraints[5].position = pose_joint_angles[pose_index][5];
        goal.motion_plan_request.goal_constraints.joint_constraints[6].position = pose_joint_angles[pose_index][6];

        // if using the right arm torso controller, send 2 extra joint angles
        if (movearm_server_name.compare("move_right_arm_torso") == 0)
        {
            goal.motion_plan_request.goal_constraints.joint_constraints[7].position = 0.0;
            goal.motion_plan_request.goal_constraints.joint_constraints[8].position = 0.0;
        }

        // Send motion plan request
        if (nh_.ok())
        {
            bool finished_within_time = false;
            move_arm_client.sendGoal(goal);
            finished_within_time = move_arm_client.waitForResult(ros::Duration(15.0));
            if (!finished_within_time)
            {
                move_arm_client.cancelGoal();
                ROS_ERROR("Timed-out achieving joint-space goal.");
            }
            else
            {
                actionlib::SimpleClientGoalState state = move_arm_client.getState();
                bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
                if (success)
                {
                    ROS_INFO("Action finished. Result code: %s", state.toString().c_str() );
                }
                else
                {
                    ROS_ERROR("Action failed with error code: %s", state.toString().c_str() );
                }
            }
        }


    } // end else pose
}

int main(int argc, char **argv)
{
    // Init the ROS node
    ros::init (argc, argv, "reem_move_arm_action");

    // error codes
    const size_t ERROR_IN_COMMAND_LINE = 1;
    const size_t SUCCESS = 0;
    const size_t ERROR_UNHANDLED_EXCEPTION = 2;

    // Parse command line options
    std::string desired_pose;
    std::string desired_arm;
    try
    {
        namespace po = boost::program_options;
        po::options_description desc("Options");
        desc.add_options()
        // Optional options
        ("help", "Print help messages")
        // Required options
        ("arm",  po::value<std::string>(&desired_arm), "Which arm ('left' or 'right')")
        ("pose", po::value<std::string>(&desired_pose), "Pose...")
        ;

        po::variables_map vm;
        try
        {
            po::store(po::parse_command_line(argc, argv, desc),
                      vm); // can throw

            // --help option
            if ( vm.count("help")  )
            {
                std::cout << "Basic Command Line Parameter App" << std::endl
                          << desc << std::endl;
                return SUCCESS;
            }

            po::notify(vm); // throws on error, so do after help in case
            // there are any problems
        }
        catch(po::error& e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
            std::cerr << desc << std::endl;
            return ERROR_IN_COMMAND_LINE;
        }

        if ( vm.count("arm")  )
        {
            // Check if arm was specified
            if ( !((desired_arm.compare("left") == 0) || (desired_arm.compare("right") == 0)) )
            {
                printf("ERROR: You must specify which arm, 'left' or 'right' \n");
                return ERROR_IN_COMMAND_LINE;
            }

        }
        else
        {
            printf("ERROR: You must specify which arm, 'left' or 'right' \n");
            return ERROR_IN_COMMAND_LINE;
        }

        if ( vm.count("pose")  )
        {
            // Check if specified pose is in pose_list
            bool match_found = false;
            for (unsigned int i=0; i < pose_names.size(); i++)
            {
                //printf("Pose %d %s \n", i, pose_names[i].c_str() );
                if (pose_names[i].compare(desired_pose) == 0) match_found = true;
            }
            if (match_found == false)
            {
                printf("ERROR: Specified pose '%s' is invalid, options are: \n", desired_pose.c_str()  );
                printf("       ");
                for (unsigned int i=0; i < pose_names.size(); i++)
                {
                    printf("'%s'  ", pose_names[i].c_str() );
                }
                printf(" \n");

                return ERROR_IN_COMMAND_LINE;
            }
        }
        else
        {
            // No pose specified, or doesn't match pose_list
            printf("ERROR: You must specify a desired goal pose: \n");
            printf("       ");
            for (unsigned int i=0; i < pose_names.size(); i++)
            {
                printf("'%s'  ", pose_names[i].c_str() );
            }
            printf(" \n");

            return ERROR_IN_COMMAND_LINE;
        }

        // run node
        REEMMoveArmAction run(desired_arm, desired_pose);
        ros::shutdown();
        return 0;
    }
    catch(std::exception& e)
    {
        std::cerr << "Unhandled Exception reached the top of main: "
                  << e.what() << ", application will now exit" << std::endl;
        return ERROR_UNHANDLED_EXCEPTION;

    }
} // end main()

