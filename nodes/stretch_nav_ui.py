#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
import sys
import select
import tf

from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest

# Stretch Stuff
import hello_helpers.hello_misc as hm


def nonblocking_input(prompt):
    print(prompt)
    while True:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.readline().strip()
        rospy.sleep(0.05)  # callbacks run here


def make_goal(x, y, theta_deg, frame="map"):
    """
    Create a MoveBaseGoal with position (x,y) and yaw = theta_deg.
    """
    theta_rad = theta_deg * 3.1415926535 / 180.0

    q = tf.transformations.quaternion_from_euler(0, 0, theta_rad)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    return goal

def switch_to_position_mode():
    rospy.wait_for_service('/switch_to_position_mode')
    try:
        switch_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        req = TriggerRequest()   # empty request for Trigger service
        resp = switch_mode(req)
        if resp.success:
            rospy.loginfo("Successfully switched to position mode: %s", resp.message)
        else:
            rospy.logwarn("Service call failed: %s", resp.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call error: %s", str(e))

def switch_to_navigation_mode():
    rospy.wait_for_service('/switch_to_navigation_mode')
    try:
        switch_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
        req = TriggerRequest()   # empty request for Trigger service
        resp = switch_mode(req)
        if resp.success:
            rospy.loginfo("Successfully switched to navigation mode: %s", resp.message)
        else:
            rospy.logwarn("Service call failed: %s", resp.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call error: %s", str(e))

def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

#################################
class NavNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

        self.subscriber_amcl = rospy.Subscriber(
            "/amcl_pose",
            PoseWithCovarianceStamped,
            self.callback_amcl_pose
        )

        # state
        self.current_amcl_pose = None
        self.current_xya = np.array([0., 0., 0.])  # x, y, angle

    def callback_amcl_pose(self, msg):
        self.current_amcl_pose = msg.pose.pose
        self.current_xya = np.array([
            self.current_amcl_pose.position.x,
            self.current_amcl_pose.position.y,
            tf.transformations.euler_from_quaternion([
                self.current_amcl_pose.orientation.x,
                self.current_amcl_pose.orientation.y,
                self.current_amcl_pose.orientation.z,
                self.current_amcl_pose.orientation.w
            ])[2]
        ])

        print(f"XYA: {self.current_xya}")

    def run_open_loop_to_x(self, x_target):
        """
        Open-loop drive the mobile base to the specified x position.
        """
        self.move_to_pose(
            {
                "translate_mobile_base": x_target
            },
            return_before_done=False
        )

    def main(self):
        hm.HelloNode.main(self, 'stretch_nav_ui', 'stretch_nav_ui', wait_for_first_pointcloud=False)
        # rospy.init_node("simple_nav_goal_sender")

        # client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        # rospy.loginfo("Waiting for move_base action server...")
        # client.wait_for_server()
        # rospy.loginfo("Connected to move_base!")

        print("\n=== SIMPLE NAV CONTROL INTERFACE ===")

        while not rospy.is_shutdown():
            switch_to_navigation_mode()
            print("\nSelect an option:")
            print("1) Open-loop drive to X")
            print("2) Send XYθ Nav Goal")
            print("q) Quit")

            try:
                choice = nonblocking_input("Enter choice: ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print("\nShutting down.")
                break

            if choice == "q":
                print("Goodbye.")
                break

            elif choice == "1":
                # OPEN-LOOP DRIVE
                switch_to_position_mode()
                try:
                    x_target = float(nonblocking_input("Enter X target (meters): "))
                except ValueError:
                    print("Invalid number.")
                    continue

                self.run_open_loop_to_x(x_target)

            elif choice == "2":
                # NAV GOAL
                target = np.array([0., 0., 0.])

                # get current amcl pose
                if self.current_amcl_pose is None:
                    print("Waiting for AMCL pose...")
                    while self.current_amcl_pose is None and not rospy.is_shutdown():
                        rospy.sleep(0.1)

                error = self.current_xya - target
                print(f"Current XYA:")
                print(f"x: {self.current_xya[0]:.3f},\ny: {self.current_xya[1]:.3f},\na: {self.current_xya[2]:.3f} rad")

                # ------------------------------------------------------------------
                # Step 1: determine whether to drive forward or backward
                # ------------------------------------------------------------------
                dx = error[0]
                dy = error[1]
                a = error[2]
                dist = np.linalg.norm(error[0:2])
                angle_to_goal = np.arctan2(dy, dx)
                print(f"Distance to goal: {dist:.3f} m, Angle to goal: {angle_to_goal:.3f} rad")
                delta = _normalize_angle(angle_to_goal - a)
                print(f"Delta angle to goal: {delta:.3f} rad")

                # If we would need to rotate more than 90°, go backwards
                if abs(delta) > np.pi / 2:
                    rospy.loginfo("Choosing BACKWARD motion to minimize rotation.")

                    # Face AWAY from goal (add pi to goal direction)
                    angle_drive = _normalize_angle(angle_to_goal + np.pi - a)

                    # Negative distance = drive backwards
                    signed_dist = -dist
                else:
                    rospy.loginfo("Choosing FORWARD motion.")
                    angle_drive = delta
                    signed_dist = +dist

                # temp: flip direction
                signed_dist = -signed_dist

                rospy.loginfo("Rotate by %.3f rad before translation.", angle_drive)
                # self.control_rotate_by(angle_drive, eps=rotate_eps)
                switch_to_position_mode()
                self.move_to_pose(
                    {
                        "rotate_mobile_base": angle_drive
                    },
                    return_before_done=False
                )

                # ------------------------------------------------------------------
                # Step 2: translate signed distance (forward or backward)
                # ------------------------------------------------------------------
                rospy.loginfo("Translate %.3f meters.", signed_dist)
                self.move_to_pose(
                    {
                        "translate_mobile_base": signed_dist
                    },
                    return_before_done=False
                )

                # ------------------------------------------------------------------
                # Step 3: rotate to final heading
                # ------------------------------------------------------------------
                switch_to_navigation_mode()
                rospy.sleep(0.1)
                _, _, a_after = self.current_xya
                delta_angle_final = _normalize_angle(target[2] - a_after)

                rospy.loginfo("Final rotate by %.3f rad to match desired heading.", delta_angle_final)
                switch_to_position_mode()
                self.move_to_pose(
                    {
                        "rotate_mobile_base": delta_angle_final
                    },
                    return_before_done=False
                )

                rospy.loginfo("==== P2P move complete ====")

                # try:
                #     user_input = input("Enter x y theta_deg: ")
                #     parts = user_input.split()
                #     if len(parts) != 3:
                #         print("Please enter three values: x y theta_deg")
                #         continue

                #     x = float(parts[0])
                #     y = float(parts[1])
                #     theta_deg = float(parts[2])
                # except ValueError:
                #     print("Invalid input.")
                #     continue

                # goal = make_goal(x, y, theta_deg)
                # rospy.loginfo("Sending nav goal: x=%.2f y=%.2f theta=%g deg",
                #             x, y, theta_deg)
                # client.send_goal(goal)

                # rospy.loginfo("Waiting for result...")
                # client.wait_for_result()
                # result = client.get_state()
                # rospy.loginfo("Result state: %d", result)

            else:
                print("Invalid choice. Please select 1, 2, or q.")


if __name__ == "__main__":
    NavNode().main()
