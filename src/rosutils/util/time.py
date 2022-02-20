import rospy

def now():
    """
        Return the current time in ROS.
    """
    return rospy.Time.now()