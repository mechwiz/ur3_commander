#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import MoveGroupActionFeedback
from std_msgs.msg import Int8

class move2pnt:
    # Initialize class
    def __init__(self):
        # setup movie commander
        moveit_commander.roscpp_initialize(sys.argv)
        # setup robot, scene, and move group
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('manipulator')
        # setup some configs
        self.group.set_goal_position_tolerance(0.05)
        self.group.set_planner_id('RRTConnectkConfigDefault')
        self.group.set_planning_time(5)
        self.group.set_num_planning_attempts(5)
        # set status to ready
        self.status = 0

        # Publishers and subscribers
        self.sub = rospy.Subscriber('ar_point',Point,self.pntCb)
        self.statsub = rospy.Subscriber('/move_group/feedback',MoveGroupActionFeedback,self.stateCb)
        self.statepub = rospy.Publisher('robot_state',Int8,queue_size=10)

    # Get the current status of the move_group
    def stateCb(self,data):
        self.status = data.status.status
        self.statepub.publish(self.status)

    # Move to the received position if possible
    def pntCb(self,data):
        pose_target = Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = data.x
        pose_target.position.y = data.y
        pose_target.position.z = data.z

        self.statepub.publish(1)
        self.group.clear_pose_targets()
        self.group.set_pose_target(pose_target)
        self.group.set_start_state_to_current_state()
        plan1 = self.group.plan()
        rospy.sleep(0.5)
        if self.status == 3:
            rospy.loginfo("============ Waiting while RVIZ displays plan...")
            rospy.sleep(5)
            self.group.execute(plan1)
            rospy.loginfo("============ Waiting while Robot executes plan...")
            rospy.sleep(5)
            self.status = 0
            self.statepub.publish(self.status)
        elif self.status == 4:
            rospy.loginfo("============ Plan out of reach...")
            rospy.sleep(2)
            self.status = 0
            self.statepub.publish(self.status)

if __name__=='__main__':
    rospy.init_node('move2pos')
    mod = move2pnt()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
