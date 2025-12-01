#!/usr/bin/python3
from geometry_msgs.msg import Pose2D

from enum import Enum
import math
import rospy

import hello_helpers.hello_misc as hm

class State(Enum):
    IDLE = 0
    TEST_DRIVE = 1
    RETURN_HOME = 2
    LOOP_MOVE = 3
    MANUAL_MOVE = 4

def print_idle_menu():
    print("========== Idle Menu ==========")
    print("1: Start Test Drive")
    print("2: Return Home")
    print("3: Loop Move")
    print("4: Manually move base")
    print("q: Quit")


class StretchBaseNavigator(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

        # pub / sub
        self.subscriber_laser_pose2d = rospy.Subscriber(
            '/pose2D',
            Pose2D,
            self.callback_laser_pose2d,
            queue_size=1
        )

        # state
        self.state = State.IDLE
        self.current_pose_xya = None
        self.rate = None

    def callback_laser_pose2d(self, msg: Pose2D):
        # rospy.loginfo("Received Pose2D: x=%f, y=%f, theta=%f", msg.x, msg.y, msg.theta)
        self.current_pose_xya = [
            msg.x,
            msg.y,
            msg.theta
        ]
    
    def state_idle(self) -> State:
        print_idle_menu()
        user_input = input("Select an option: ")
        if user_input == '1':
            return State.TEST_DRIVE
        elif user_input == '2':
            return State.RETURN_HOME
        elif user_input == '3':
            return State.LOOP_MOVE
        elif user_input == '4':
            return State.MANUAL_MOVE
        elif user_input == 'q':
            rospy.signal_shutdown("User requested shutdown.")
            return State.IDLE
        else:
            print("Invalid option. Staying in IDLE state.")
            return State.IDLE

    def state_test_drive(self) -> State:
        rospy.loginfo("Starting test drive...")
        poses = [
            {'rotate_mobile_base': -1},
            {'translate_mobile_base': 0.3},
            {'rotate_mobile_base': 2.},
        ]
        
        for pose in poses:
            self.move_to_pose(
                pose,
                return_before_done=False
            )

        return State.IDLE

    def state_return_home(self) -> State:
        rospy.loginfo("Returning home...")
        self.control_move_to(0.0, 0.0, 0.0)

        return State.IDLE
    
    def state_loop_move(self) -> State:
        rospy.loginfo("Starting looped movement...")
        
        # generate square loop waypoints
        side_length = 0.5  # meters
        waypoints = [
            (side_length, 0.0, 0.0),
            (side_length, side_length, math.pi/2),
            (0.0, side_length, math.pi),
            (0.0, 0.0, -math.pi/2),
            (0., 0., 0.0)
        ]

        for x_goal, y_goal, a_goal in waypoints:
            self.control_move_to(x_goal, y_goal, a_goal)

        return State.IDLE

    def state_manual_move(self) -> State:
        rospy.loginfo("Manual move mode. Enter 'q' to quit.")
        while not rospy.is_shutdown():
            user_input = input("Enter target (x y theta) or 'q': ")
            if user_input.lower() == 'q':
                break
            try:
                inputs = user_input.split()
                if len(inputs) != 3:
                    raise ValueError("Expected three values: x y theta")
                x_str, y_str, a_str = inputs
                x_goal = float(x_str)
                y_goal = float(y_str)
                a_goal = float(a_str)
                self.control_move_to(x_goal, y_goal, a_goal)
            except ValueError:
                print("Invalid input. Please enter x, y, theta or 'q' to quit.")
        
        return State.IDLE
    
    #####
    def control_move_to(
        self,
        x_goal: float,
        y_goal: float,
        a_goal: float,
        rotate_eps: float = 0.05,
        translate_eps: float = 0.02
    ):
        """
        Point-to-point motion controller with rotate → translate → rotate
        and intelligent backward motion to minimize rotation.

        Args:
            x_goal, y_goal: absolute target position in world frame
            a_goal: desired final heading (radians)
            rotate_eps: angular tolerance (rad)
            translate_eps: positional tolerance (m)
        """

        rospy.loginfo("==== Starting P2P move to (%.3f, %.3f, %.3f rad) ====",
                    x_goal, y_goal, a_goal)

        # ------------------------------------------------------------------
        # Step 0: get current pose
        # ------------------------------------------------------------------
        x, y, a = self.current_pose_xya
        dx = x_goal - x
        dy = y_goal - y
        dist = math.hypot(dx, dy)

        if dist < translate_eps:
            rospy.loginfo("Already within position tolerance of goal. Only rotating.")
            # Just do final heading alignment
            delta_angle_final = self._normalize_angle(a_goal - a)
            self.control_rotate_by(delta_angle_final, eps=rotate_eps)
            return

        # ------------------------------------------------------------------
        # Step 1: determine whether to drive forward or backward
        # ------------------------------------------------------------------
        angle_to_goal = math.atan2(dy, dx)
        delta = self._normalize_angle(angle_to_goal - a)

        # If we would need to rotate more than 90°, go backwards
        if abs(delta) > math.pi / 2:
            rospy.loginfo("Choosing BACKWARD motion to minimize rotation.")

            # Face AWAY from goal (add pi to goal direction)
            angle_drive = self._normalize_angle(angle_to_goal + math.pi - a)

            # Negative distance = drive backwards
            signed_dist = -dist
        else:
            rospy.loginfo("Choosing FORWARD motion.")
            angle_drive = delta
            signed_dist = +dist

        rospy.loginfo("Rotate by %.3f rad before translation.", angle_drive)
        self.control_rotate_by(angle_drive, eps=rotate_eps)

        # ------------------------------------------------------------------
        # Step 2: translate signed distance (forward or backward)
        # ------------------------------------------------------------------
        rospy.loginfo("Translate %.3f meters.", signed_dist)
        self.control_translate_by(signed_dist, eps=translate_eps)

        # ------------------------------------------------------------------
        # Step 3: rotate to final heading
        # ------------------------------------------------------------------
        _, _, a_after = self.current_pose_xya
        delta_angle_final = self._normalize_angle(a_goal - a_after)

        rospy.loginfo("Final rotate by %.3f rad to match desired heading.", delta_angle_final)
        self.control_rotate_by(delta_angle_final, eps=rotate_eps)

        rospy.loginfo("==== P2P move complete ====")

    # ------------------------------------------------------
    # Angle normalization helper
    # ------------------------------------------------------
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


    def control_rotate_by(self, angle_rad: float, eps: float = 0.02):
        rospy.loginfo("Rotating base by %.2f radians", angle_rad)

        a0 = self.current_pose_xya[2]
        a1 = a0 + angle_rad
        while abs(a1 - a0) > eps and not rospy.is_shutdown():
            rospy.loginfo("Current angle: %.2f, Target angle: %.2f", a0, a1)
            self.rate.sleep()

            a0 = self.current_pose_xya[2]
            self.move_to_pose(
                {'rotate_mobile_base': a1 - a0},
                return_before_done=False
            )

    def control_translate_by(
        self,
        distance_m: float,
        eps: float = 0.05,
        max_step: float = 0.05,   # max delta per iteration (meters)
        kp: float = 0.7,          # shrink step as error decreases
        timeout_s: float = 8.0,
    ):
        """
        Translate the robot by distance_m meters using repeated small
        position-delta commands (not velocity control).

        Args:
            distance_m: total signed distance to travel
            eps: acceptable final error (meters)
            max_step: maximum per-iteration delta command (meters)
            kp: proportional factor that reduces step size near goal
            timeout_s: emergency timeout
        """

        rospy.loginfo("Translating base by %.2f m (delta commands)", distance_m)

        # Initial pose
        x0, y0, a0 = self.current_pose_xya

        # Fixed target pose
        x_target = x0 + distance_m * math.cos(a0)
        y_target = y0 + distance_m * math.sin(a0)

        def dist_to_target(x, y):
            return math.hypot(x_target - x, y_target - y)

        start_t = rospy.Time.now().to_sec()
        last_dist = dist_to_target(x0, y0)

        while not rospy.is_shutdown():

            # Timeout check
            if rospy.Time.now().to_sec() - start_t > timeout_s:
                rospy.logwarn("Timeout reached, aborting translation.")
                break

            # Read current pose
            x, y, a = self.current_pose_xya
            dist = dist_to_target(x, y)

            # Check goal
            if dist <= eps:
                rospy.loginfo("Reached target within %.3f m", eps)
                break

            # Overshoot detection (ignore very small noise)
            if dist > last_dist + 0.003:
                rospy.logwarn("Overshoot detected — stopping further commands.")
                break

            # Compute step size (proportional, clamped)
            step = kp * dist
            step = max(-max_step, min(step, max_step))

            # Forward/backward sign based on original desired direction
            step *= math.copysign(1.0, distance_m)

            # Issue delta command
            self.move_to_pose(
                {"translate_mobile_base": step},
                return_before_done=False,
            )

            last_dist = dist
            self.rate.sleep()

        # Stop movement (safety)
        self.move_to_pose({"translate_mobile_base": 0.0})


    #####
    def main(self):
        print("StretchTestMotion::main()")
        hm.HelloNode.main(
            self,
            'stretch_test_motion',
            'stretch_test_motion',
            wait_for_first_pointcloud=False
            )
        
        # config
        self.rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.state == State.IDLE:
                self.state = self.state_idle()
            elif self.state == State.TEST_DRIVE:
                self.state = self.state_test_drive()
            elif self.state == State.RETURN_HOME:
                self.state = self.state_return_home()
            elif self.state == State.LOOP_MOVE:
                self.state = self.state_loop_move()
            elif self.state == State.MANUAL_MOVE:
                self.state = self.state_manual_move()
            else:
                rospy.logerr("Unknown state: %s", str(self.state))
                self.state = State.IDLE

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = StretchBaseNavigator()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')