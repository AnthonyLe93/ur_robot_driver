#! /usr/bin/env python3
import rospy
import datetime

from ur_robot_driver.msg import ATI_mini45


def talker():
    pub = rospy.Publisher('ATI f/t readings', ATI_mini45, queue_size=10)
    rospy.init_node('ATI_pub', anonymous=True)
    r = rospy.Rate(10) # 10Hz

    msg = ATI_mini45

    msg.name.data = 'ATI_<enter_unique_ID>'
    force_value_dummy = datetime.datetime.now()
    print(force_value_dummy)

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass




