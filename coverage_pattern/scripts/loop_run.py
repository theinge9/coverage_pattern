#! /usr/bin/env python3
"""
Room loop and use PublishPoint to define pattern area
"""
import sys

import rospy
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from coverage_pattern.srv import pattern, patternRequest

class LoopAndClean:
    def __init__(self):
        rospy.init_node('loop_and_clean')

        self.last_arrived_room = None
        #self.clicked_points = []
        self.service_count = 0

        # Subscriibe to Clicked points topic
        rospy.Subscriber('/clicked_point', PointStamped, self.PointReceived)

        # Connect to move_base
        self.move_base = SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the move_base action server to come up")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base")

    def PointReceived(self, point):
        rospy.loginfo("Point received: (%f, %f)", point.point.x, point.point.y)
        #self.clicked_points.append(point.point)

        # Service request with the clicked points
        request = patternRequest()
        request.points = [point]
        print(request.points)

        # Call the service 
        try:
            # Service client for pattern movement
            self.pattern_client = rospy.ServiceProxy('/pattern_movement', pattern)
            rospy.wait_for_service('/pattern_movement')
            response = self.pattern_client(request)
            rospy.loginfo("Service response: %s", response.result)
            if response.result == "Pattern Running Completed.":
                self.service_count = 1
            #return response.result
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            #return None
        
    def create_waypoint(self, name, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = theta
        goal.target_pose.pose.orientation.w = 1.0
        return{"name": name, "goal": goal}
    
    def move(self, waypoint):
        rospy.loginfo(f"Sending goal: {waypoint['name']}")
        self.move_base.send_goal(waypoint["goal"])
        self.move_base.wait_for_result()
        result_state = self.move_base.get_state()
        rospy.loginfo(f"Result State: {result_state}")
        return result_state == 3

    def wait_for_input(self):
        choice = True
        while choice:
            user_input = input("Do you want to clean this room? Enter 'yes' to clean, 'no' to skip and 'end' to finish : ")
            if user_input == "yes":
                rospy.loginfo ("Please use PublishPoint Tool to define running area...")
                rospy.sleep(15)
                while self.service_count == 0:
                    rospy.loginfo("Pattern is running...")
                    rospy.sleep(8)
                    if self.service_count == 1:
                        self.return_to_room_waypoint()
                        #choice = False
            elif user_input == "no":
                rospy.loginfo("Skipping and continuing to next room")
                choice = False
            elif user_input == "end":
                rospy.loginfo("Ending the process")
                sys.exit()
            else:
                rospy.loginfo("Invalid input. Please enter'yes', 'no', 'end'")
        #return user_input.lower() == 'yes'

    def return_to_room_waypoint(self):
        if self.last_arrived_room:
            rospy.loginfo(f"Returning to last arrived room: {self.last_arrived_room['name']}")
            success = self.move(self.last_arrived_room)
            if success:
                rospy.loginfo(f"Arrived back at {self.last_arrived_room['name']}.")
                self.service_count = 0
                self.send_waypoint()
            else:
                rospy.logwarn(f"Failed to return to {self.last_arrived_room['name']}.")

    def send_waypoint(self):
        waypoints = [
            self.create_waypoint("Room 1 and 2", -4.0, 0.01, 0.0),
            self.create_waypoint("Room 3 and 4", 0.1, 0.01, 0.0),
        ]

        try:
            while not rospy.is_shutdown():
                for waypoint in waypoints:
                    rospy.loginfo(f"Moving to {waypoint['name']}")
                    success = self.move(waypoint)
                    if success:
                        rospy.loginfo(f"Arrived at {waypoint['name']}.")
                        self.last_arrived_room = waypoint
                        choice = self.wait_for_input()
                        if choice == False:
                            continue
                    else:
                        rospy.logwarn(f"Failed to reach {waypoint['name']}.")
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    roomlooping = LoopAndClean()
    roomlooping.send_waypoint()
    rospy.spin()