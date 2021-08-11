#!/usr/bin/env python

# ROS Stuff
import rospy

# Stretch Stuff
import hello_helpers.hello_misc as hm

class StretchTestMotion(hm.HelloNode):
    def __init__(self):
        self.rate = 10.0
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'stretch_test_motion', 'stretch_test_motion', wait_for_first_pointcloud=False)
        rospy.Rate(self.rate)

if __name__ == '__main__':
    try:
        print("Starting StretchTestMotion Node!")
        node = StretchTestMotion()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')