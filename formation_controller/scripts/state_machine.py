#!/usr/bin/env python3

import rospy
import smach
from nav_msgs.msg import Path
import roslaunch
from tf import transformations
import math
from copy import deepcopy

# define state Parse_path
class Get_path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['formation_path_received'])
        
    def execute(self, userdata):
        active_robots = rospy.get_param("/active_robots", 2)
        robot_names = rospy.get_param("/robot_names", ["mir1", "mir2"])
        relative_positions_x = rospy.get_param("/relative_positions_x", [0, 0])
        relative_positions_y = rospy.get_param("/relative_positions_y", [0, 1])
        formatin_plan_topic = rospy.get_param("/formation_plan_topic","/voronoi_planner/formation_plan")
        rospy.loginfo('Witing for path')

        path = rospy.wait_for_message(formatin_plan_topic, Path)
        rospy.loginfo('formation path received')
        start_pose = path.poses[0].pose

        for i in range(1,active_robots):
            # compute the target pose 
            target_pose = deepcopy(start_pose)
            theta = transformations.euler_from_quaternion([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w])[2]
            target_pose.position.x += relative_positions_x[i] * math.cos(theta) - relative_positions_y[i] * math.sin(theta)
            target_pose.position.y += relative_positions_x[i] * math.sin(theta) + relative_positions_y[i] * math.cos(theta)
            target_pose_ = [target_pose.position.x, target_pose.position.y, theta]

            # set the target pose on the parameter server
            rospy.set_param("/" + robot_names[i] + "/move_to_start_pose/target_pose",target_pose_)

            # launch the move_to_start_pose node                
            process = launch_ros_node("move_to_start_pose","formation_controller","move_to_start_pose.py", "", robot_names[i])

            # wait for the node to finish
            while process.is_alive():
                rospy.sleep(0.1)
                pass
            rospy.loginfo(robot_names[i] + " in start pose")

        rospy.wait_for_message('ur_path', Path)
        rospy.loginfo('ur_path received')

        return 'paths_received'


# define state Compute_trajectory
class Compute_trajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trajectories_received'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Compute_trajectory')
        # rospy.wait_for_message('mir_trajectory', Path)
        # rospy.loginfo('mir_trajectory received')
        rospy.wait_for_message('ur_trajectory', Path)
        rospy.loginfo('ur_trajectory received')
        return 'trajectories_received'
        

class Move_MiR_to_start_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['mir_initialized'])
        

    def execute(self, userdata):
        rospy.set_param("/state_machine/move_mir_to_start_pose",True)
        rospy.loginfo('Executing state Move_MiR_to_start_pose')
        while not rospy.get_param("/state_machine/mir_initialized") and not rospy.is_shutdown():
            rospy.sleep(0.1)
            pass
        return 'mir_initialized'


class Move_UR_to_start_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ur_initialized'])
        

    def execute(self, userdata):
        rospy.set_param("/state_machine/move_ur_to_start_pose",True)
        rospy.loginfo('Executing state Move_UR_to_start_pose')
        while not rospy.get_param("/state_machine/ur_initialized") and not rospy.is_shutdown():
            rospy.sleep(0.1)
            pass
        return 'ur_initialized'

class Follow_trajectory(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        

    def execute(self, userdata):
        rospy.set_param("/state_machine/follow_trajectory",True)
        while not rospy.get_param("/state_machine/done") and not rospy.is_shutdown():
            rospy.sleep(0.1)
        pass
        return 'done'

def launch_ros_node(node_name, package_name, node_executable, node_args="", namespace="/"):
    
    package = package_name
    executable = node_executable
    name = node_name
    node = roslaunch.core.Node(package=package, node_type=executable, name=name, namespace=namespace,
                                    machine_name=None, args=node_args, output="screen")
    
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    return process


# main
def main():
    rospy.init_node('smach_example_state_machine')
    init_ros_param_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Get_path', Get_path(), 
                               transitions={'formation_path_received':'Compute_trajectory'})
        smach.StateMachine.add('Compute_trajectory', Compute_trajectory(), 
                               transitions={'trajectories_received':'Move_MiR_to_start_pose'})
        smach.StateMachine.add('Move_MiR_to_start_pose', Move_MiR_to_start_pose(), 
                               transitions={'mir_initialized':'Move_UR_to_start_pose'})
        smach.StateMachine.add('Move_UR_to_start_pose', Move_UR_to_start_pose(),
                                 transitions={'ur_initialized':'Follow_trajectory'})
        smach.StateMachine.add('Follow_trajectory', Follow_trajectory(),
                                transitions={'done':'outcome5'})
    # Execute SMACH plan
    outcome = sm.execute()


def init_ros_param_server():
    rospy.set_param("/state_machine/move_mir_to_start_pose",False)
    rospy.set_param("/state_machine/move_ur_to_start_pose",False)
    rospy.set_param("/state_machine/paths_received",False)
    rospy.set_param("/state_machine/trajectories_received",False)
    rospy.set_param("/state_machine/ur_trajectory_received",False)
    rospy.set_param("/state_machine/mir_trajectory_received",False)
    rospy.set_param("/state_machine/mir_initialized",False)
    rospy.set_param("/state_machine/ur_initialized",False)
    rospy.set_param("/state_machine/follow_trajectory",False)
    rospy.set_param("/state_machine/done",False)




if __name__ == '__main__':
    main()