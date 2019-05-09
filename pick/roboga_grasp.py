#!/usr/bin/env python

import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
   
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    right_arm = MoveGroupCommander("arm")
    right_gripper = MoveGroupCommander("gripper")
    rospy.sleep(1)
    right_arm.allow_replanning(True)
    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("part")
    scene.remove_attached_object("claw_base", "part")

    print("Current position")
    print(right_arm.get_current_pose())

    while(True):
        print("Going to default position")
        right_arm.set_named_target("default")
        right_arm.go()
        
        print("Opening gripper")
        right_gripper.set_joint_value_target([0.0, 0.0])
        right_gripper.go()
    
        rospy.sleep(1)

        '''
        # publish a demo scene
        p = PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        
        # add a table
        p.pose.position.x = 0.42
        p.pose.position.y = -0.2
        p.pose.position.z = 0.0
        scene.add_box("table", p, (0.5, 1.5, 0.3))

        # add an object to be grasped
        p.pose.position.x = 0.11
        p.pose.position.y = 0.22
        p.pose.position.z = 0.05
        scene.add_box("part", p, (0.015, 0.05, 0.015))
    
        rospy.sleep(1)
        '''

        grasps = []
        
        g = Grasp()
        g.id = "test"
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = "base_cylinder"
        
        #First grab position
        print("1st - grab position")
        grasp_pose.pose.position.x = 0.130465574219
        grasp_pose.pose.position.y = 0.171727879517
        grasp_pose.pose.position.z = 0.091324779826
        grasp_pose.pose.orientation.x = -0.929337637511
        grasp_pose.pose.orientation.y = 0.312980256889
        grasp_pose.pose.orientation.z = -0.0625227051047
        grasp_pose.pose.orientation.w = 0.185649739157
    
        right_arm.set_pose_target(grasp_pose)
        right_arm.go()
        
        rospy.sleep(1)

        #Close gripper
        print("closing gripper -> picking the object")
        right_gripper.set_joint_value_target([0.0, 1.3])
        right_gripper.go()
        rospy.sleep(1)


        #Intermediary position
        print("2nd - intermediary position")
        grasp_pose.pose.position.x = 0.0338358181953
        grasp_pose.pose.position.y = 0.111980460465
        grasp_pose.pose.position.z = 0.348853572587
        grasp_pose.pose.orientation.x = -0.609562743085
        grasp_pose.pose.orientation.y = 0.090081013576
        grasp_pose.pose.orientation.z = -0.115141285087
        grasp_pose.pose.orientation.w = 0.779141295083
    
        right_arm.set_pose_target(grasp_pose)
        right_arm.go()
    
        rospy.sleep(1)

        #Release position
        print("3rd - Release position")
        grasp_pose.pose.position.x = -0.123723280852
        grasp_pose.pose.position.y = 0.139932524285
        grasp_pose.pose.position.z = 0.24127610413
        grasp_pose.pose.orientation.x = -0.792556844105
        grasp_pose.pose.orientation.y = -0.300130191062
        grasp_pose.pose.orientation.z = 0.187988468572
        grasp_pose.pose.orientation.w = 0.496423058448
        
    
        right_arm.set_pose_target(grasp_pose)
        right_arm.go()
        rospy.sleep(1)

        print("opening gripper - dropping the object")
        right_gripper.set_joint_value_target([0.0, 0.0])
        right_gripper.go()
        rospy.sleep(1)

        print("Going to default position")
        right_arm.set_named_target("default")
        right_arm.go()
        rospy.sleep(1)

        #Release position
        print("4rd - pick position")
        grasp_pose.pose.position.x = -0.155810030834
        grasp_pose.pose.position.y = 0.143864276059
        grasp_pose.pose.position.z = 0.164545369077
        grasp_pose.pose.orientation.x = -0.824385127273
        grasp_pose.pose.orientation.y = -0.360874104029
        grasp_pose.pose.orientation.z = 0.174869610059
        grasp_pose.pose.orientation.w = 0.399474232524
    
        right_arm.set_pose_target(grasp_pose)
        right_arm.go()
        rospy.sleep(1)

        #Close gripper
        print("closing gripper -> picking the object")
        right_gripper.set_joint_value_target([0.0, 1.3])
        right_gripper.go()
        rospy.sleep(1)

        #Intermediary position
        print("5nd - intermediary position")
        grasp_pose.pose.position.x = -0.200185911705
        grasp_pose.pose.position.y = 0.12837491995
        grasp_pose.pose.position.z = 0.29266275809
        grasp_pose.pose.orientation.x = -0.556086093048
        grasp_pose.pose.orientation.y = -0.303999442285
        grasp_pose.pose.orientation.z = 0.371046974474
        grasp_pose.pose.orientation.w = 0.678731713524
    
        right_arm.set_pose_target(grasp_pose)
        right_arm.go()
    
        rospy.sleep(1)

        #First grab position
        print("6st - release position")
        grasp_pose.pose.position.x = 0.10415480839
        grasp_pose.pose.position.y = 0.139909615101
        grasp_pose.pose.position.z = 0.209532787431
        grasp_pose.pose.orientation.x = -0.792150664116
        grasp_pose.pose.orientation.y = 0.262481974837
        grasp_pose.pose.orientation.z = -0.173309169081
        grasp_pose.pose.orientation.w = 0.523033909168
    
        right_arm.set_pose_target(grasp_pose)
        right_arm.go()
        
        rospy.sleep(1)

        print("opening gripper - dropping the object")
        right_gripper.set_joint_value_target([0.0, 0.0])
        right_gripper.go()
        rospy.sleep(1)

        print("Going to default position")
        right_arm.set_named_target("default")
        right_arm.go()
        rospy.sleep(1)


    print("shutdown")
    #rospy.spin()
    roscpp_shutdown()

