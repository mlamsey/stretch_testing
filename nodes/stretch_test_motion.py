#!/usr/bin/env python

# ROS Stuff
import rospy

# Stretch Stuff
import hello_helpers.hello_misc as hm

# Utilities
import math

class StretchTestMotion(hm.HelloNode):
    home_pose = {
        'wrist_extension': 0.1,
        'joint_wrist_roll': 0.0,
        'joint_wrist_pitch': -math.pi/2.0,
        'joint_wrist_yaw': 0.0,
        'gripper_aperture': 0.0,
        'joint_lift': 0.75
    }

    def __init__(self):
        self.rate = 10.0
        hm.HelloNode.__init__(self)

    def wave(self, n_waves):
        print("StretchTestMotion::wave()")

        # Create trajectory
        center_pose = dict(self.home_pose)
        center_pose.update({'joint_wrist_pitch': 0})
        wave_amp = 0.2
        left_pose = dict(self.home_pose) # copy home pose
        left_pose.update({'joint_wrist_pitch': 0.1, 'joint_wrist_yaw': wave_amp, 'joint_wrist_roll': -wave_amp}) # set wave left
        right_pose = dict(left_pose)
        right_pose.update({'joint_wrist_yaw': -wave_amp, 'joint_wrist_roll': wave_amp}) # set wave right

        # Execute motion
        self.move_to_pose(center_pose)
        for i in range(n_waves):
            self.move_to_pose(left_pose)
            self.move_to_pose(right_pose)
        self.move_to_pose(center_pose)
        self.home()
        
    def home(self):
        self.move_to_pose(self.home_pose)

    def main(self):
        print("StretchTestMotion::main()")
        hm.HelloNode.main(self, 'stretch_test_motion', 'stretch_test_motion', wait_for_first_pointcloud=False)
        rospy.Rate(self.rate)

        # Test motion
        self.wave(2)

if __name__ == '__main__':
    try:
        node = StretchTestMotion()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')