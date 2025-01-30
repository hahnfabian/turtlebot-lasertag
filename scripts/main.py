#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Point, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from config import MOVE_BASE_PARAMS, INITIAL_POSE, CNN_DETECTION_CONFIDENCE_THRESHOLD, CNN_DETECTION_URL, CNN_REQUEST_COOLDOWN, PATROL_POSES
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import requests
import cv2
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from geometry_msgs.msg import Twist, Vector3
import traceback
from std_srvs.srv import Empty
import message_filters

import roslib
roslib.load_manifest("rpc_game_master")
from rpc_game_client.srv import PlayerScore



class TurtleBot:
    def __init__(self):
        rospy.loginfo("[TurtleBot] Initializing TurtleBot...")
        
        # Publishers
        self.pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self._wait_for_connections(self.pose_publisher, "pose_publisher")
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self._wait_for_connections(self.cmd_vel_pub, "cmd_vel_pub")


        # Move base client
        self.move_to_goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[TurtleBot] Waiting for move_base action server...")
        self.move_to_goal_client.wait_for_server()
        rospy.loginfo("[TurtleBot] Connected to move_base action server.")
        self.stop_patrol = False

        # Set parameters and initial pose
        self.set_move_base_params()
        self.set_initial_pose(self.pose_publisher, INITIAL_POSE)

        # Image submission utils
        rospy.wait_for_service('/rpc_score')
        self.should_submit_image = False 
        self.earliest_possible_submission_time = rospy.Time.now()
        self.camera_info = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=3)

        # CNN detection utils
        self.cnn_detection_url = CNN_DETECTION_URL
        self.cnn_confidence_threshold = CNN_DETECTION_CONFIDENCE_THRESHOLD
        self.cnn_last_request = rospy.Time.now()
        self.cnn_request_cooldown = CNN_REQUEST_COOLDOWN
        self.cv_bridge = CvBridge()
        self.last_cnn_detected_turtlebot = None
        self.last_cnn_detected_turtlebot_time = rospy.Time.now()

        # Detection subscribers
        self.tag_detections_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detections_callback) 
        self.image_raw_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.cnn_detection_callback)
        self.image_compressed_subscriber = rospy.Subscriber( "/camera/rgb/image_color/compressed", CompressedImage, self.submit_image_callback)

        # Start patrolling
        rospy.sleep(4)
        self.last_detected_bot = rospy.Time.now()
        rospy.Timer(rospy.Duration(1), self.check_should_patrol())
        self.patrolling = False
        self.patrol()

    def check_should_patrol(self):
        rospy.loginfo(f"Checking if patrol is needed. Last detection time: {self.last_detected_bot}, Current time: {rospy.Time.now()}")
        
        if rospy.Time.now() - self.last_detected_bot >= rospy.Duration(3):
            rospy.logwarn(f"No turtlebot detected for over 3 seconds. Last seen at {self.last_detected_bot}. Running patrol...")
            self.patrol()
        else:
            rospy.loginfo("Turtlebot detected recently. No need to patrol.")

    def submit_image_callback(self, image):
        if self.should_submit_image and (rospy.Time.now() >= self.earliest_possible_submission_time):
            self.should_submit_image = False 

            try:
                score = rospy.ServiceProxy('/rpc_score', PlayerScore)
                header = Header()
                header.stamp = rospy.Time.now()
                response = score(image, self.camera_info)
                rospy.loginfo("[submit_image_callback] Scored: %s (%s+%s+%s)" % (response.scor.total, response.score.align, response.score.center, response.score.distance))

                self.set_earliest_possible_submission_time(rospy.Time.now() + rospy.Duration(10))

                self.patrol()

            except Exception as e:
                rospy.logerr(f"[submit_image_callback] Service call failed: {e}")


    def set_earliest_possible_submission_time(self, time): 
        self.earliest_possible_submission_time = time


    def cnn_detection_callback(self, image):
        detected_turtlebots = []
        
        if (rospy.Time.now() - self.cnn_last_request).to_sec() < self.cnn_request_cooldown:
            return
        
        self.cnn_last_request = rospy.Time.now()

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            _, img_encoded = cv2.imencode('.jpg', cv_image)
            img_bytes = img_encoded.tobytes()
            files = {'file': ('image.jpg', img_bytes, 'image/jpeg')}

            response = requests.post(self.cnn_detection_url, files=files)
            
            if response.status_code == 200:
                response_json = response.json()
                if "detections" in response_json and isinstance(response_json["detections"], list):
                    for detection in response_json["detections"]:
                        if detection["confidence"] >= self.cnn_confidence_threshold:
                            x_center = detection["bounding_box"]["x_center"]
                            y_center = detection["bounding_box"]["y_center"]
                            height = detection["bounding_box"]["height"]
                            width = detection["bounding_box"]["width"]
                            confidence = detection["confidence"]
                            detected_turtlebots.append({"confidence": confidence, "x_center": x_center, "y_center": y_center, "height": height, "width": width})

                            # rospy.loginfo(f"confidence: {confidence}, x_center: {x_center}, y_center: {y_center}, height: {height}, width: {width}")
                            rospy.loginfo(f"confidence: {confidence}")
                    if detected_turtlebots:
                        self.cancel_goal()
                        self.last_cnn_detected_turtlebot = detected_turtlebots[0]
                        self.last_cnn_detected_turtlebot_time = rospy.Time.now()
                        self.last_detected_bot = rospy.Time.now()

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")


    def tag_detections_callback(self, apriltag_detection_array_msg):
        try:
            if apriltag_detection_array_msg.detections:
                self.cancel_goal()
                self.last_detected_bot = rospy.Time.now()

                detection = apriltag_detection_array_msg.detections[0]
                id = detection.id
                covariance = detection.pose.pose.covariance
                position = detection.pose.pose.pose.position
                orientation = detection.pose.pose.pose.orientation

                cmd_vel = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))            

                if abs(position.x) <= 0.2: 
                    self.should_submit_image = True
                    return
                
                if position.x > 0.2:
                    cmd_vel.angular.z = -1.5 *(0.5 / position.z)
                elif position.x < -0.2:
                    cmd_vel.angular.z = 1.5* (0.5 / position.z)

                if position.z > 0.7:
                    cmd_vel.linear.x = 0.3* position.z

                self.send_velocity_command(cmd_vel)

            elif self.last_cnn_detected_turtlebot and (rospy.Time.now() - self.last_cnn_detected_turtlebot_time <= rospy.Duration(1)):

                tb = self.last_cnn_detected_turtlebot
                image_center = 640/2
                x_offset = tb['x_center'] - image_center
                rospy.loginfo(f"[last_cnn_detected_turtlebot] x_offset: {x_offset}")

                if x_offset > 50: # rechts
                    cmd_vel = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))            
                    cmd_vel.angular.z = -0.5 #is this right????
                    cmd_vel.linear.x = 0.1
                    self.send_velocity_command(cmd_vel)
                elif x_offset < -50:
                    cmd_vel = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))            
                    cmd_vel.angular.z = 0.5 
                    cmd_vel.linear.x = 0.1
                    self.send_velocity_command(cmd_vel)

        except Exception as e:
            rospy.logerr(f"[tag_detections_callback] Error processing AprilTag detection: {e}")

    
    def send_velocity_command(self, twist_msg):
        try:
            # rospy.loginfo(f"Sending velocity command: Linear: {twist_msg.linear}, Angular: {twist_msg.angular}")
            self.cmd_vel_pub.publish(twist_msg)
        
        except Exception as e:
            rospy.logerr(f"Failed to send velocity command: {str(e)}")
            rospy.logerr(f"Error traceback: {traceback.format_exc()}")


    def clear_costmaps(self):
        # Wait for the service to be available
        rospy.wait_for_service('/move_base/clear_costmaps')
        
        try:
            # Create a service proxy for '/move_base/clear_costmaps'
            clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmaps_service()
            # Call the service
            rospy.loginfo("Costmaps cleared successfully")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    

    def _wait_for_connections(self, publisher, name, timeout=30):
        """
        Waits for subscribers to connect to the given publisher.

        :param publisher: The ROS Publisher object to check
        :param name: Name of the publisher, for logging purposes
        :param timeout: Maximum time to wait for subscribers in seconds
        """
        start_time = rospy.Time.now()
        while publisher.get_num_connections() == 0:
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time > timeout:
                rospy.logerr(f"[{name}] Timeout waiting for subscribers.")
                return
            rospy.loginfo(f"[{name}] Waiting for subscribers to connect...")
            rospy.sleep(1)


    def set_move_base_params(self):
        """
        Sets move_base parameters using values from the configuration.
        """
        try:
            for param, value in MOVE_BASE_PARAMS.items():
                rospy.set_param(param, value)
                # rospy.loginfo(f"[set_move_base_params] Setting param {param} to {value}.")
            rospy.loginfo("[set_move_base_params] Move base parameters set successfully.")
        except Exception as e:
            rospy.logerr(f"[set_move_base_params] Failed to set parameters: {e}")


    def set_initial_pose(self, pose_publisher, initial_pose, timeout=5):
        """
        Publishes the initial pose to the given publisher.

        :param pose_publisher: ROS Publisher for initial pose
        :param initial_pose: Dictionary containing 'x', 'y', 'orientation', and 'covariance'
        :param timeout: Maximum time to wait for subscribers in seconds
        """
        # Wait for subscribers to connect
        self._wait_for_connections(pose_publisher, "Initial Pose Publisher", timeout)

        # Create and populate the initial pose message
        try:
            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.pose.pose.position = Point(
                x=initial_pose["x"],
                y=initial_pose["y"],
                z=0.0
            )
            quaternion = Quaternion(*initial_pose["orientation"])
            initial_pose_msg.pose.pose.orientation = quaternion
            initial_pose_msg.pose.covariance = initial_pose["covariance"]
            initial_pose_msg.header = Header()
            initial_pose_msg.header.stamp = rospy.Time.now()
            initial_pose_msg.header.frame_id = "map"

            # Publish the initial pose
            pose_publisher.publish(initial_pose_msg)
            rospy.loginfo("[Initial Pose Publisher] Initial pose published successfully.")
        except KeyError as e:
            rospy.logerr(f"[Initial Pose Publisher] Missing key in initial_pose dictionary: {e}")
        except Exception as e:
            rospy.logerr(f"[Initial Pose Publisher] Failed to publish initial pose: {e}")


    def move_to_goal(self, client, x, y, orientation):
        """
        Sends a goal to the move base client and waits for the result.

        :param client: MoveBaseAction client
        :param x: Target x-coordinate
        :param y: Target y-coordinate
        :param orientation: Target orientation as a tuple (x, y, z, w)
        """
        try:
            # Define the goal message
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation = Quaternion(*orientation)

            # Send the goal
            rospy.loginfo(f"[move_to_goal] Sending goal to ({x}, {y}) with orientation {orientation}.")
            client.send_goal(goal)

            # Wait for the result
            client.wait_for_result()
            result_state = client.get_state()

            if result_state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("[move_to_goal] Goal reached successfully.")
            else:
                rospy.logwarn(f"[move_to_goal] Goal failed with state: {result_state}.")
        except Exception as e:
            rospy.logerr(f"[move_to_goal] Error while sending goal: {e}")


    def cancel_goal(self):
        """
        Cancels the current goal if one is active.
        """
        try:
            if self.move_to_goal_client.get_state() in [
                actionlib.GoalStatus.ACTIVE, 
                actionlib.GoalStatus.PENDING
            ]:
                self.move_to_goal_client.cancel_goal()
                rospy.loginfo("[stop_goal] Current goal has been canceled.")
            
            self.stop_patrol = True
            self.patrolling = False

        except Exception as e:
            rospy.logerr(f"[stop_goal] Error while canceling goal: {e}")
    

    def move_to_initial_position(self):
        """
        Moves the robot back to the initial position.
        """
        rospy.loginfo("[move_to_initial_position] Moving to initial position...")
        self.move_to_goal(self.move_to_goal_client, INITIAL_POSE["x"], INITIAL_POSE["y"], INITIAL_POSE["orientation"])
        rospy.loginfo("[move_to_initial_position] Movement complete.")


    def patrol(self, places=PATROL_POSES):
        self.stop_patrol = False
        self.patrolling = True
        client = self.move_to_goal_client
        

        for place in places:
            if self.stop_patrol:
                rospy.loginfo("[patrol] Patrol canceled, stopping movement.")
                break
            self.clear_costmaps()
            
            x = place['x']
            y = place['y']
            orientation = place['orientation']
            rospy.loginfo(f"Moving to place: {x}, {y}, orientation: {orientation}")
            self.move_to_goal(client, x, y, orientation)
            result = client.wait_for_result()  # This will block until result is available
            if result:
                if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"Successfully reached the goal at ({x}, {y}) with orientation {orientation}")
                else:
                    rospy.logwarn(f"Failed to reach the goal at ({x}, {y}) with orientation {orientation}")
            else:
                rospy.logwarn(f"Timeout while waiting to reach the goal at ({x}, {y})")

        rospy.loginfo("Patrol completed!" if not self.stop_patrol else "[patrol] Patrol stopped due to cancellation.")


def main():
    """
    Main function to initialize the TurtleBot and start moving to goals.
    """
    rospy.init_node("turtlebot_node", anonymous=True)

    try:
        rospy.loginfo("[TurtleBot Node] Initializing TurtleBot...")
        turtlebot = TurtleBot()
        rospy.loginfo("[TurtleBot Node] TurtleBot initialized. Spinning...")
        rospy.spin()


    except rospy.ROSInterruptException:
        rospy.loginfo("[TurtleBot Node] Node interrupted. Shutting down gracefully.")
    except Exception as e:
        rospy.logerr(f"[TurtleBot Node] Unexpected error: {e}")
    finally:
        rospy.loginfo("[TurtleBot Node] Cleanup complete. Exiting.")


if __name__ == "__main__":
    main()

