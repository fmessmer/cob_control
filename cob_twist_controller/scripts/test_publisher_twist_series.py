#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
 
def twistPub():
  rospy.init_node("test_publisher_twist_series", anonymous=True)
  
  pub = rospy.Publisher("twist_controller/command_twist_stamped", TwistStamped, queue_size=1)
  
  rospy.sleep(3.0)
  
  twist_msg =  TwistStamped()
  twist_msg.header.stamp = rospy.Time.now()
  twist_msg.header.frame_id = "arm_base_link"
  twist_msg.twist.linear.x = 0
  twist_msg.twist.linear.y = 0
  twist_msg.twist.linear.z = 0
  twist_msg.twist.angular.x = 0
  twist_msg.twist.angular.y = 0
  twist_msg.twist.angular.z = 0  
  
  rate = 50
  r = rospy.Rate(rate)
  
  for i in range(0, 2*rate, 1):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_base_link"
    twist_msg.twist.linear.x = 0
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = 0
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()
  
  for i in range(0, 5*rate, 1):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_base_link"
    twist_msg.twist.linear.x = -0.05
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = -0.03
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()
  
  for i in range(0, rate, 2):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_base_link"
    twist_msg.twist.linear.x = 0
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = 0
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()
 
  for i in range(0, 3*rate, 1):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_base_link"
    twist_msg.twist.linear.x = 0.07
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = 0
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()
    
  for i in range(0, rate, 2):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_base_link"
    twist_msg.twist.linear.x = 0
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = 0
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()

  for i in range(0, 7*rate, 1):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_base_link"
    twist_msg.twist.linear.x = 0
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = 0.05
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()
    
  for i in range(0, 2*rate, 1):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_base_link"
    twist_msg.twist.linear.x = 0
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = 0
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()
  
  
  print "done"

if __name__ == '__main__':
  try:
      twistPub()
  except rospy.ROSInterruptException: pass

