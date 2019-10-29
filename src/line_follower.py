#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

import utils

# The topic to publish control commands to
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
error_tracking = []

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class LineFollower:

  '''
  Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
  '''
  def __init__(self, plan, pose_topic, plan_lookahead, translation_weight,
               rotation_weight, kp, ki, kd, error_buff_length, speed):
    # Store the passed parameters
    self.plan = plan
    self.pose_topic = pose_topic
    self.plan_lookahead = plan_lookahead
    # Normalize translation and rotation weights
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    # The error buff stores the error_buff_length most recent errors and the
    # times at which they were received. That is, each element is of the form
    # [time_stamp (seconds), error]. For more info about the data struct itself, visit
    # https://docs.python.org/2/library/collections.html#collections.deque
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.speed = speed

    # YOUR CODE HERE
    # Create a publisher to PUB_TOPIC
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=1)
    # Create a subscriber to pose_topic, with callback 'self.pose_cb'
    self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb)

  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''
  def compute_error(self, cur_pose):

    # Find the first element of the plan that is in front of the robot, and remove
    # any elements that are behind the robot. To do this:
    # Loop over the plan (starting at the beginning) For each configuration in the plan
        # If the configuration is behind the robot, remove it from the plan
        #   Will want to perform a coordinate transformation to determine if
        #   the configuration is in front or behind the robot
        # If the configuration is in front of the robot, break out of the loop

    while len(self.plan) > 0:

      # Transform coordinates of plan from initial axes to that of the car
      plan_from_car = self.transform_to_car(cur_pose, 0)
      #rospy.loginfo("plan from car")
      #rospy.loginfo(plan_from_car)

      # Break from loop if plan is in front of car, otherwise pop
      if plan_from_car[1][0] > 0: #and abs(plan_from_car[0][0]) < .5):
        #rospy.loginfo("time to break!!")
        break

      self.plan.pop(0)

    # Check if the plan is empty. If so, return (False, 0.0)
    if not self.plan:
      rospy.loginfo("-----ALL DONE WITH PLAN-----")
      return (False, 0.0)

    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index
    goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)

    # Compute the translation error between the robot and the configuration at goal_idx in the plan

    # Transform coordinates of plan at plan_lookahead to the car axes, set translation error
    #immediate_plan = self.transform_to_car(cur_pose, 0)
    goal_plan = self.transform_to_car(cur_pose, goal_idx)
    translation_error = goal_plan[0][0]

    # Compute the total error
    # Translation error was computed above
    # Rotation error is the difference in yaw between the robot and goal configuration
    #   Be careful about the sign of the rotation error

    # Finding rotation error - this does not completely work, so it is currently not being used below
    goal_rot = self.plan[goal_idx][2]
    cur_rot = cur_pose[2]
    if goal_rot < 0:
      goal_rot = np.pi + (np.pi - abs(goal_rot))
    if cur_rot < 0:
      cur_rot = np.pi + (np.pi - abs(cur_rot))
    rotation_error = goal_rot - cur_rot

    # Debugging
    #rospy.loginfo("goal idx")
    #rospy.loginfo(goal_idx)
    #rospy.loginfo("plan coords")
    #rospy.loginfo(self.plan[goal_idx])
    #rospy.loginfo("current pose")
    #rospy.loginfo(cur_pose)
    #rospy.loginfo("immediate plan")
    #rospy.loginfo(immediate_plan)
    #rospy.loginfo("goal plan")
    #rospy.loginfo(goal_plan)


    error = self.translation_weight * translation_error# + self.rotation_weight * rotation_error

    return True, error

  # Function to translate between map coordinate frame and car coordinate frame
  def transform_to_car(self, cur_pose, index):
    theta = cur_pose[2] - np.pi/2
    rot_mat = utils.rotation_matrix(theta)
    rot_mat[0,1] *= -1
    rot_mat[1,0] *= -1
    plan_coords = np.array([[self.plan[index][0]], [self.plan[index][1]]])
    trans_mat = np.array([[cur_pose[0] * -1], [cur_pose[1] * -1]])
    plan_from_car = np.matmul(rot_mat, plan_coords+trans_mat)

    return plan_from_car

  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() # Get the current time

    # Compute the derivative error using the passed error, the current time,
    # the most recent error stored in self.error_buff, and the most recent time
    # stored in self.error_buff

    # Computing the derivative error on every loop but the first
    if self.error_buff:
     deriv_error = (abs(error) - abs(self.error_buff[len(self.error_buff) - 1][0])) / (now - self.error_buff[len(self.error_buff) - 1][1])
    else:
     deriv_error = 0.0

    # Adjusting derivative error to contribute to the steering angle in the correct direction
    if deriv_error > 0 and error != 0.0:
     deriv_steer = (error / abs(error)) * deriv_error
    elif deriv_error < 0 and error != 0.0:
     deriv_steer = -1 * (error / abs(error)) * deriv_error
    else:
     deriv_steer = 0

    # Add the current error to the buffer
    self.error_buff.append((error, now))

    # Compute the integral error by applying rectangular integration to the elements
    # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/


    # Finding integral error by adding all errors in the buffer
    length = len(self.error_buff)
    integ_error = 0.0
    for i in range(length-1):
      mid = (self.error_buff[i][0] + self.error_buff[i+1][0]) / 2
      integ_error += mid


    # Compute the steering angle as the sum of the pid errors
    # Multiply by -1 because steering left is + and right is -


    #rospy.loginfo("error")
    #rospy.loginfo(error)
    #rospy.loginfo("derivative error")
    #rospy.loginfo(deriv_steer)
    #rospy.loginfo("integral error")
    #rospy.loginfo(integ_error)
    return -1*(self.kp*error + self.kd*deriv_steer + self.ki*integ_error)

  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
  '''
  def pose_cb(self, msg):
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])
    #rospy.loginfo("INITIAL POSE")
    #rospy.loginfo(cur_pose)
    success, error = self.compute_error(cur_pose)
    errornum = float (error)
    error_tracking.append(errornum)

    if not success:
      # We have reached our goal
      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops
      rospy.loginfo(error_tracking)

    delta = self.compute_steering_angle(error)

    #rospy.loginfo("steering angle")
    #rospy.loginfo(delta)

    # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed

    # Send the control message
    self.cmd_pub.publish(ads)

def main():

  rospy.init_node('line_follower', anonymous=True) # Initialize the node

  # Load these parameters from launch file
  # We provide suggested starting values of params, but you should
  # tune them to get the best performance for your system
  # Look at constructor of LineFollower class for description of each var
  # 'Default' values are ones that probably don't need to be changed (but you could for fun)
  # 'Starting' values are ones you should consider tuning for your system
  # YOUR CODE HERE
  plan_topic = '/planner_node/car_plan' # Default val.
  pose_topic = '/sim_car_pose/pose' # Default val.
  plan_lookahead = 5 # Starting val.
  translation_weight = 1.0 # Starting val: 1.0
  rotation_weight = 1.0 # Starting val: 0.0
  kp = 1.0 # Starting val: 1.0
  ki = 0.2 # Starting val: 0.0
  kd = 1.0 # Starting val: 0.0
  error_buff_length = 10 # Starting val: 10
  speed = 1.0 # Default val.

  raw_input("Press Enter to when plan available...")  # Waits for ENTER key press

  # Use rospy.wait_for_message to get the plan msg
  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]

  plan_in = rospy.wait_for_message(plan_topic, PoseArray)
  plan = []

  for msg in plan_in.poses:
    plan_step = np.array([msg.position.x, msg.position.y, utils.quaternion_to_angle(msg.orientation)])
    plan.append(plan_step)

  #rospy.loginfo(plan)
  #rospy.loginfo("END OF PLAN")

  # Create a LineFollower object
  lf = LineFollower(plan, pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, error_buff_length, speed)

  rospy.spin() # Prevents node from shutting down

if __name__ == '__main__':
  main()
