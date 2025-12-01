#!/usr/bin/env python3
import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest

# Stretch Stuff
import hello_helpers.hello_misc as hm

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

class NavNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def run_open_loop_to_x(self, x_target):
        """
        Placeholder for an open-loop drive routine.
        Replace this with your real control code.
        """
        # rospy.logwarn("OPEN-LOOP DRIVE PLACEHOLDER: driving to x = %.2f", x_target)

        self.move_to_pose(
            {
                "translate_mobile_base": x_target
            },
            return_before_done=False
        )

        # rospy.logwarn("Open-loop drive NOT implemented. Replace placeholder with real code.")


    def main(self):
        hm.HelloNode.main(self, 'stretch_nav_ui', 'stretch_nav_ui', wait_for_first_pointcloud=False)
        # rospy.init_node("simple_nav_goal_sender")

        client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        client.wait_for_server()
        rospy.loginfo("Connected to move_base!")

        print("\n=== SIMPLE NAV CONTROL INTERFACE ===")

        while not rospy.is_shutdown():
            print("\nSelect an option:")
            print("1) Open-loop drive to X")
            print("2) Send XYθ Nav Goal")
            print("q) Quit")

            try:
                choice = input("Enter choice: ").strip().lower()
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
                    x_target = float(input("Enter X target (meters): "))
                except ValueError:
                    print("Invalid number.")
                    continue

                self.run_open_loop_to_x(x_target)

            elif choice == "2":
                # NAV GOAL
                switch_to_navigation_mode()
                try:
                    user_input = input("Enter x y theta_deg: ")
                    parts = user_input.split()
                    if len(parts) != 3:
                        print("Please enter three values: x y theta_deg")
                        continue

                    x = float(parts[0])
                    y = float(parts[1])
                    theta_deg = float(parts[2])
                except ValueError:
                    print("Invalid input.")
                    continue

                goal = make_goal(x, y, theta_deg)
                rospy.loginfo("Sending nav goal: x=%.2f y=%.2f theta=%g deg",
                            x, y, theta_deg)
                client.send_goal(goal)

                rospy.loginfo("Waiting for result...")
                client.wait_for_result()
                result = client.get_state()
                rospy.loginfo("Result state: %d", result)

            else:
                print("Invalid choice. Please select 1, 2, or q.")


if __name__ == "__main__":
    NavNode().main()
