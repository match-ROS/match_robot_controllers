#!/usr/bin/env python3

import rospy
import smach
from nav_msgs.msg import Path

# define state Parse_path
class Get_path(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['paths_received'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Parse_path')
        rospy.wait_for_message('mir_path', Path)
        rospy.loginfo('mir_path received')
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


# main
def main():
    rospy.init_node('smach_example_state_machine')
    init_ros_param_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Parse_path', Parse_path(), 
                               transitions={'paths_received':'Compute_trajectory', 
                                            'trajectories_received':'outcome4'})
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