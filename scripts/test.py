#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node('rosimple_node', anonymous=True)
if True:
  import time

  pub = rospy.Publisher('rosimple', String, queue_size=10)
  time.sleep(1)
  pub.publish('Spider spider_standup')
  print(3)
  import time

  pub = rospy.Publisher('rosimple', String, queue_size=10)
  time.sleep(1)
  pub.publish('Spider going forward for 2 seconds')


