import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from std_msgs.msg import Float64MultiArray
import math


class TrajectoryActionClient(Node):

	def __init__(self):

		super().__init__('points_publisher_node_action_client')
		self.action_client = ActionClient (self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
		
		# subscribe to the topic that publishes the end effector position
		# consider topic name and message type as /end_effector_pos and (X, Y, Z) respectively
		self.subscription = self.create_subscription(Float64MultiArray, '/end_effector_pos', self.listener_callback, 10)

	# Inverse Kinematics for 6R robot
	# Get the joint angles from the end effector position
	def listener_callback(self, msg):
		# get the end effector position from the message
		x = msg.data[0]
		y = msg.data[1]
		z = msg.data[2]

		# get the joint angles from the end effector position
		# consider the joint angles as theta1, theta2, theta3, theta4, theta5, theta6
		theta1 = math.atan2(y, x)
		r = math.sqrt(x**2 + y**2)
		d = math.sqrt((r - 0.5)**2 + z**2)
		theta2 = math.atan2(z, r - 0.5)
		theta3 = math.acos((d**2 - 0.7**2 - 0.7**2) / (-2 * 0.7 * 0.7))
		theta4 = math.atan2(z - 0.7 * math.sin(theta3), r - 0.5 - 0.7 * math.cos(theta3))
		theta5 = math.atan2(y, x)
		theta6 = math.atan2(-math.sin(theta1) * math.cos(theta5) * math.sin(theta2 + theta3 + theta4) + math.cos(theta1) * math.cos(theta2 + theta3 + theta4), math.cos(theta1) * math.cos(theta5) * math.sin(theta2 + theta3 + theta4) + math.sin(theta1) * math.cos(theta2 + theta3 + theta4))

		# publish the joint angles to the topic /joint_angles
		# consider topic name and message type as /joint_angles and (theta1, theta2, theta3, theta4, theta5, theta6) respectively
		self.publisher = self.create_publisher(Float64MultiArray, '/joint_angles', 10)
		msg = Float64MultiArray()
		msg.data = [theta1, theta2, theta3, theta4, theta5, theta6]
		self.publisher.publish(msg)

		# send the joint angles to the action server
		self.send_goal()

	def send_goal(self, joint_angles):

		points = []

		point1_msg = JointTrajectoryPoint()
		point1_msg.positions = joint_angles
		point1_msg.time_from_start = Duration(seconds=2.0).to_msg()

		points.append(point1_msg)

		joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
		goal_msg = FollowJointTrajectory.Goal()
		goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
		goal_msg.trajectory.joint_names = joint_names
		goal_msg.trajectory.points = points

		self.action_client.wait_for_server()
		self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
		self.send_goal_future.add_done_callback(self.goal_response_callback)

	
	def goal_response_callback(self, future):
		
		goal_handle = future.result()

		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected ')
			return

		self.get_logger().info('Goal accepted')

		self.get_result_future= goal_handle.get_result_async()
		self.get_result_future.add_done_callback(self.get_result_callback)

	def get_result_callback (self, future):
		
		result = future.result().result
		self.get_logger().info('Result: '+str(result))
		rclpy.shutdown()

		
	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback


def main(args=None):

	rclpy.init()
	
	action_client = TrajectoryActionClient()
	future = action_client.send_goal()
	rclpy.spin(action_client)

if __name__ == '__main__':
	main()


