#!/usr/bin/env python3

import rospy




class Formation_controller():

    def __init__(self):
        
        pass


    def run(self):
        pass

if __name__ == "__main__":
    try:
        rospy.init_node('move_to_start_pose')
        rospy.loginfo('move_to_start_pose node started')
        exe = Formation_controller()
        exe.run()
    except rospy.ROSInterruptException:
        pass