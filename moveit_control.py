#!/usr/bin/env python

# This module uses the MoveIt tutorials controller class, which is freely
# available using the BSD license below

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import pandas as pd
import tf.transformations as tr

try:
    from math import pi, tau, dist, fabs, cos, sin, tan, radians
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sin, tan, sqrt, radians

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

TAKE_PICTURE = False
PICTURE_ID = 0

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_pose_goal(self, pose_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        #move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        #pose_goal = geometry_msgs.msg.Pose()
        #pose_goal.orientation.w = 1.0
        #pose_goal.position.x = x
        #pose_goal.position.y = y
        #pose_goal.position.z = z

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

def img_callback(msg):
    '''This is a callback function used to save images from a ROS topic'''

    global TAKE_PICTURE
    global PICTURE_ID

    try:
        # Convert ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        print('CvBridgeError')
    else:
        # Save OpenCV2 image as png
        if TAKE_PICTURE:
            print("Saved an image!")
            print("")
            cv2.imwrite('whatever' + str(PICTURE_ID) + '.png', cv2_img)
            TAKE_PICTURE = False

def parse_command(command):
    '''This function takes a command in our custom format and parses it to a pose in world space'''

    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = -1 * command['dist'] * cos(radians(command['yaw'])) * cos(radians(command['pitch'])) #+ command['xoff']
    pose_goal.position.y = -1 * command['dist'] * sin(radians(command['yaw'])) * cos(radians(command['pitch'])) #+ command['yoff']
    pose_goal.position.z = command['dist'] * sin(radians(command['pitch']))  #+ command['zoff']

    goal_q = tr.quaternion_from_euler(radians(command['yaw']), radians(command['pitch']) - pi/2, 0, 'rzyx')

    pose_goal.orientation.x = goal_q[0]
    pose_goal.orientation.y = goal_q[1]
    pose_goal.orientation.z = goal_q[2]
    pose_goal.orientation.w = goal_q[3]

    offset = [command['xoff'], command['yoff'], command['zoff'], 0]
    goal_q_star = tr.quaternion_conjugate(goal_q)

    offset_tr = tr.quaternion_multiply(goal_q, offset)
    offset_tr = tr.quaternion_multiply(offset_tr, goal_q_star)

    pose_goal.position.x += offset_tr[0]
    pose_goal.position.y += offset_tr[1]
    pose_goal.position.z += offset_tr[2]

    return pose_goal

def main():
    '''This is the main control loop for our imaging system'''

    global TAKE_PICTURE
    global PICTURE_ID

    try:
        #rospy.init_node('image_listener')

        print("")
        print("----------------------------------------------------------")
        print("Welcome to the Imager!")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")

        # 'yaw', 'pitch', 'dist', 'roll', 'xoff', 'yoff', 'zoff'
        commands = pd.read_csv('commands.csv')

        controller = MoveGroupPythonInterfaceTutorial()

        #  Subscriber for saving images
        img_topic = "/camera1/image_raw"
        rospy.Subscriber(img_topic, Image, img_callback)

        #starting_pose = tutorial.move_group.get_current_pose().pose

        # Starting pose of the end effector, should correspond to the camera pointing
        # down at a suitable rest position

        starting_pose = geometry_msgs.msg.Pose()

        starting_pose.position.x = 0.3
        starting_pose.position.y = 0
        starting_pose.position.z = 0.6

        starting_pose.orientation.x = -0.9238795
        starting_pose.orientation.y = 0.3826834
        starting_pose.orientation.z = 0
        starting_pose.orientation.w = 0

        controller.go_to_pose_goal(starting_pose)

        start_q = [starting_pose.orientation.x, starting_pose.orientation.y,
                    starting_pose.orientation.z, starting_pose.orientation.w]

        pose_goal = geometry_msgs.msg.Pose()

        print("============ Reading from commands.csv:")
        print("")

        data = bmaskgen.init_df()

        for i in range( len(commands.index) ):
            pose_goal = parse_command(commands.iloc[i])

            #pose_goal.orientation.w = 1.0
            goal_q = [pose_goal.orientation.x, pose_goal.orientation.y,
                        pose_goal.orientation.z, pose_goal.orientation.w]

            # Correct end effector orientation
            goal_q = tr.quaternion_multiply(goal_q, start_q)

            pose_goal.orientation.x = goal_q[0]
            pose_goal.orientation.y = goal_q[1]
            pose_goal.orientation.z = goal_q[2]
            pose_goal.orientation.w = goal_q[3]

            # Compensate for object pose
            pose_goal.position.x += 0.75
            pose_goal.position.y += 0.0
            pose_goal.position.z += 0.024

            print(pose_goal)
            print("")

            input("============ Press `Enter` to execute command ...")

            controller.go_to_pose_goal(pose_goal)

            TAKE_PICTURE = True

            time.sleep(0.5)

            PICTURE_ID += 1
            TAKE_PICTURE = False

            data.loc[len(data.index)] = ['whatever' + str(i), 'part']


        PICTURE_ID = 0

        data.to_csv('bmask.csv', index=False)

        input("============ Press `Enter` to return to starting pose ...")
        tutorial.go_to_pose_goal(starting_pose)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
